/*  OV2640 (ArduCAM Mini) + OLED SSD1351 preview + SD save (FULL JPEG) + HUD
    + BOOT SPLASH (bee) + sequential component self-test

    BOOT UX:
      - After SD init + OLED init: shows simple bee splash
      - Bottom line shows sequential checks:
          SD: OK / FAIL
          OLED: OK / FAIL
          CAM: OK / FAIL
      - On FAIL: shows a dedicated error screen with details and halts.

    IMPORTANT: shared SPI => never talk to SD while CAM_CS is LOW.
*/

#ifdef swap
#undef swap
#endif

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <ArduCAM.h>
#include "memorysaver.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

// ===================== PINY =====================
#define CAM_CS      6
#define OLED_CS     9
#define OLED_DC     8
#define OLED_RST    7
#define SD_CS       10
#define SHUTTER_PIN 5

// ===================== BATTERY ADC =====================
#define BAT_PIN   A0
static const float R1 = 28200.0f;     // 28.2k
static const float R2 = 100000.0f;    // 100k
static const float ADC_VREF = 3.3f;
static const int   ADC_MAX  = 4095;

// ===================== OLED / CAM =====================
static const int OLED_W = 128;
static const int OLED_H = 96;
static const int HUD_H  = 12;

static const int SRC_W = 160;
static const int SRC_H = 120;

// preview: 3:2 -> 126x84
static const int IMG_W  = 126;
static const int IMG_H  = 84;
static const int IMG_X0 = (OLED_W - IMG_W) / 2;  // 1
static const int IMG_Y0 = HUD_H;

// po obrocie: 120 x 160
static const int ROT_H  = SRC_W; // 160
static const int CROP_W = 120;
static const int CROP_H = 80;
static const int CROP_X0 = 0;
static const int CROP_Y0 = (ROT_H - CROP_H) / 2; // 40

static uint16_t frame[SRC_W * SRC_H];
static uint16_t lineBuf[IMG_W];

ArduCAM myCAM(OV2640, CAM_CS);
Adafruit_SSD1351 display(OLED_W, OLED_H, &SPI, OLED_CS, OLED_DC, OLED_RST);

// SPI settings
static const SPISettings camSPI(16000000, MSBFIRST, SPI_MODE0);

// SD: conservative for stability on shared SPI (you can try 8MHz later)
static const uint32_t SD_HZ = 16000000;

// ===================== PREVIEW FPS LIMIT =====================
static uint32_t lastPreviewMs = 0;
static const uint32_t PREVIEW_PERIOD_MS = 125; // ~8 fps

// ===================== STATE =====================
static uint32_t lastFrameMs = 0;
static float fpsEMA = 0.0f;

static float lastVbat = 0.0f;
static uint32_t lastBatMs = 0;

static bool lastBtn = HIGH;
static uint32_t lastDebounceMs = 0;

static uint16_t photoCount = 0; // ile zdjęć zapisanych na karcie (wg nextIndex-1)
static uint8_t  flashBlackFrames = 0;

// ===================== SAVE ERROR DEBUG =====================
enum SaveErr : uint8_t {
  SAVE_OK = 0,
  E_TIMEOUT,
  E_LEN,
  E_OPEN,
  E_WRITE,
};

// ===================== CAM MODE =====================
enum CamMode : uint8_t { MODE_PREVIEW, MODE_PHOTO };
static CamMode camMode = MODE_PREVIEW;

// ===================== CAMINFO (SD metadata) =====================
static const char* INFO_PATH = "/CAMINFO.BIN";
static const uint16_t CAMINFO_MAGIC = 0xCAFE;

struct CamInfo {
  uint16_t magic;
  uint16_t nextIndex;    // 1..9999 (+1 po ostatnim)
  uint32_t totalShots;   // lifetime (na tej karcie)
};
static CamInfo camInfo = {0, 1, 0};

// ===================== helpers =====================
static void deselectAll() {
  digitalWrite(OLED_CS, HIGH);
  digitalWrite(CAM_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
}

// hard release: ensure bus is idle before SD ops (helps E_OPEN/E_WRITE)
static void releaseBusForSD() {
  SPI.endTransaction(); // safe even if none active
  digitalWrite(CAM_CS, HIGH);
  digitalWrite(OLED_CS, HIGH);
  digitalWrite(SD_CS, HIGH);
  delayMicroseconds(50);
}

static inline uint16_t pack565(uint8_t hi, uint8_t lo) {
  return (uint16_t(hi) << 8) | lo;
}

static void oledBlackFrame() {
  deselectAll();
  digitalWrite(OLED_CS, LOW);
  display.fillRect(0, 0, OLED_W, OLED_H, 0x0000);
  digitalWrite(OLED_CS, HIGH);
}

static void showErr(SaveErr e, uint32_t extra = 0) {
  deselectAll();
  digitalWrite(OLED_CS, LOW);
  display.fillScreen(0x0000);
  display.setTextColor(0xFFFF);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("SAVE FAIL");
  switch (e) {
    case E_TIMEOUT: display.println("E_TIMEOUT"); break;
    case E_LEN:     display.println("E_LEN");     break;
    case E_OPEN:    display.println("E_OPEN");    break;
    case E_WRITE:   display.println("E_WRITE");   break;
    default:        display.println("E_UNKNOWN"); break;
  }
  display.print("X=");
  display.println(extra);
  digitalWrite(OLED_CS, HIGH);
}

static float readBatteryVoltage() {
  (void)analogRead(BAT_PIN);
  (void)analogRead(BAT_PIN);

  long sum = 0;
  const int N = 24;
  for (int i = 0; i < N; i++) {
    sum += analogRead(BAT_PIN);
    delay(2);
  }
  float adc  = sum / float(N);
  float vadc = (adc / float(ADC_MAX)) * ADC_VREF;
  return vadc * (R1 + R2) / R2;
}

static bool waitCaptureDone(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    SPI.beginTransaction(camSPI);
    digitalWrite(CAM_CS, LOW);
    bool done = myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK);
    digitalWrite(CAM_CS, HIGH);
    SPI.endTransaction();
    if (done) return true;
    delay(1);
  }
  return false;
}

static void makeFilename(char* out, size_t outSize, uint16_t idx) {
  snprintf(out, outSize, "/IMG%04u.JPG", (unsigned)idx);
}

static uint16_t scanPhotoCount() {
  char path[16];
  uint16_t i = 1;
  for (; i <= 9999; i++) {
    makeFilename(path, sizeof(path), i);
    if (!SD.exists(path)) break;
  }
  return (uint16_t)(i - 1);
}

// ---- CAMINFO load/save
static bool loadCamInfo() {
  File f = SD.open(INFO_PATH, FILE_READ);
  if (!f) return false;

  CamInfo tmp;
  int n = f.read((uint8_t*)&tmp, sizeof(tmp));
  f.close();

  if (n != (int)sizeof(tmp)) return false;
  if (tmp.magic != CAMINFO_MAGIC) return false;
  if (tmp.nextIndex < 1 || tmp.nextIndex > 10000) return false;

  camInfo = tmp;
  return true;
}

static bool saveCamInfo() {
  File f = SD.open(INFO_PATH, FILE_WRITE);
  if (!f) return false;

  f.seek(0);
  int wr = f.write((uint8_t*)&camInfo, sizeof(camInfo));
  f.flush();
  f.close();

  return wr == (int)sizeof(camInfo);
}

// ---- Camera mode switching
static void camSetPreviewMode() {
  if (camMode == MODE_PREVIEW) return;

  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.set_format(BMP);
  myCAM.OV2640_set_JPEG_size(OV2640_160x120);
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  camMode = MODE_PREVIEW;
}

static void camSetPhotoMode() {
  if (camMode == MODE_PHOTO) return;

  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.set_format(JPEG);
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  camMode = MODE_PHOTO;
}

// ===================== BOOT UI =====================
static void bootStatusLine(const char* msg, bool ok) {
  deselectAll();
  digitalWrite(OLED_CS, LOW);

  // clear bottom strip
  const int h = 12;
  display.fillRect(0, OLED_H - h, OLED_W, h, 0x0000);
  display.setTextSize(1);
  display.setTextColor(0xFFFF);
  display.setCursor(0, OLED_H - h + 2);
  display.print(msg);
  display.print(": ");
  display.print(ok ? "OK" : "FAIL");

  digitalWrite(OLED_CS, HIGH);
  delay(180);
}

static void bootFailScreen(const char* title, const char* detail1, const char* detail2 = nullptr) {
  deselectAll();
  digitalWrite(OLED_CS, LOW);
  display.fillScreen(0x0000);
  display.setTextColor(0xFFFF);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("BOOT FAIL");
  display.println(title);
  display.println("");
  if (detail1) display.println(detail1);
  if (detail2) display.println(detail2);
  digitalWrite(OLED_CS, HIGH);
  while (1) delay(1000);
}

// Simple bee drawn with primitives (no bitmap tables)
static void drawBeeSplash() {
  const uint16_t Y = 0xFFE0;  // yellow
  const uint16_t K = 0x0000;  // black
  const uint16_t W = 0xFFFF;  // white
  const uint16_t G = 0xC618;  // gray (wings)

  deselectAll();
  digitalWrite(OLED_CS, LOW);
  display.fillScreen(0x0000);

  // Centered bee
  int cx = OLED_W / 2;
  int cy = 42;

  // Wings
  display.fillCircle(cx - 18, cy - 12, 10, G);
  display.fillCircle(cx + 18, cy - 12, 10, G);
  display.drawCircle(cx - 18, cy - 12, 10, W);
  display.drawCircle(cx + 18, cy - 12, 10, W);

  // Body (oval-ish)
  display.fillCircle(cx, cy, 16, Y);
  display.fillCircle(cx, cy + 8, 14, Y);

  // Stripes
  display.fillRect(cx - 16, cy - 6, 32, 5, K);
  display.fillRect(cx - 14, cy + 2, 28, 5, K);
  display.fillRect(cx - 10, cy + 10, 20, 5, K);

  // Head
  display.fillCircle(cx, cy - 18, 9, Y);
  display.drawCircle(cx, cy - 18, 9, K);

  // Eyes
  display.fillCircle(cx - 3, cy - 20, 2, K);
  display.fillCircle(cx + 3, cy - 20, 2, K);

  // Antennae
  display.drawLine(cx - 5, cy - 28, cx - 10, cy - 34, W);
  display.drawLine(cx + 5, cy - 28, cx + 10, cy - 34, W);

  // Tiny title
  display.setTextColor(0xFFFF);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("BEE CAM");

  digitalWrite(OLED_CS, HIGH);
}

// ===================== HUD (short labels) =====================
static bool printIfFitsAt(int &x, int y, int limitX, const char* s) {
  int16_t bx, by;
  uint16_t bw, bh;
  display.getTextBounds(s, 0, 0, &bx, &by, &bw, &bh);
  if (x + (int)bw <= limitX) {
    display.setCursor(x, y);
    display.print(s);
    x += (int)bw;
    return true;
  }
  return false;
}

static void drawHUD(float fps, float vbat, uint16_t imgCount, uint32_t totalShots) {
  display.fillRect(0, 0, OLED_W, HUD_H, 0x0000);
  display.setTextSize(1);
  display.setTextColor(0xFFFF);

  char vbuf[10];
  snprintf(vbuf, sizeof(vbuf), "%.2fV", vbat);

  int16_t x1, y1;
  uint16_t vw, vh;
  display.getTextBounds(vbuf, 0, 0, &x1, &y1, &vw, &vh);
  int vX = OLED_W - (int)vw - 1;

  display.setCursor(vX, 2);
  display.print(vbuf);

  int limitX = vX - 2;
  int x = 0;
  const int y = 2;

  char buf[24];

  snprintf(buf, sizeof(buf), "F%.1f ", fps);
  printIfFitsAt(x, y, limitX, buf);

  snprintf(buf, sizeof(buf), "I%u ", (unsigned)imgCount);
  printIfFitsAt(x, y, limitX, buf);

  snprintf(buf, sizeof(buf), "T%lu", (unsigned long)totalShots);
  printIfFitsAt(x, y, limitX, buf);
}

// ===================== preview render =====================
static void renderPreviewToOLED() {
  drawHUD(fpsEMA, lastVbat, photoCount, camInfo.totalShots);

  display.startWrite();
  display.setAddrWindow(IMG_X0, IMG_Y0, IMG_W, IMG_H);

  for (int y = 0; y < IMG_H; y++) {
    int ry = CROP_Y0 + (y * CROP_H) / IMG_H;
    for (int x = 0; x < IMG_W; x++) {
      int rx = CROP_X0 + (x * CROP_W) / IMG_W;
      int srcX = ry;
      int srcY = (SRC_H - 1) - rx;
      lineBuf[x] = frame[srcY * SRC_W + srcX];
    }
    display.writePixels(lineBuf, IMG_W, true);
  }
  display.endWrite();
}

// ===================== PREVIEW CAPTURE (FAST) =====================
static bool capturePreviewRawFast() {
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  if (!waitCaptureDone(2000)) return false;

  uint32_t len = 0;
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  len = myCAM.read_fifo_length();
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  const uint32_t expected = (uint32_t)SRC_W * SRC_H * 2;
  if (len < expected - 64 || len > expected + 2048) return false;

  SPI.beginTransaction(camSPI);
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  for (int i = 0; i < SRC_W * SRC_H; i++) {
    uint8_t hi = SPI.transfer(0x00);
    uint8_t lo = SPI.transfer(0x00);
    frame[i] = pack565(hi, lo);
  }
  myCAM.CS_HIGH();
  SPI.endTransaction();

  return true;
}

// ===================== FULL JPEG (UXGA) -> SD =====================
static SaveErr captureAndSaveFullJpeg(uint16_t newIndex) {
  camSetPhotoMode();
  delay(50);

  oledBlackFrame();

  // --- przechwytywanie JPEG ---
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.start_capture();
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  if (!waitCaptureDone(8000)) return E_TIMEOUT;

  // --- długość FIFO ---
  uint32_t length = 0;
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  length = myCAM.read_fifo_length();
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  length &= 0x07FFFFF;
  const uint32_t MAX_OK = 8UL * 1024UL * 1024UL;
  if (length < 1024 || length >= MAX_OK || length == 0x07FFFFF) {
    return E_LEN;
  }

  // --- zapis na SD ---
  char path[16];
  makeFilename(path, sizeof(path), newIndex);

  releaseBusForSD();
  File f = SD.open(path, FILE_WRITE);
  if (!f) return E_OPEN;

  static uint8_t buf[4096];
  uint32_t remaining = length;

  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.set_fifo_burst();

  while (remaining) {
    uint32_t n = (remaining > sizeof(buf)) ? sizeof(buf) : remaining;
    for (uint32_t i = 0; i < n; i++) {
      buf[i] = SPI.transfer(0x00);
    }

    digitalWrite(CAM_CS, HIGH);
    SPI.endTransaction();

    releaseBusForSD();
    if (f.write(buf, n) != n) {
      f.close();
      return E_WRITE;
    }

    remaining -= n;
    SPI.beginTransaction(camSPI);
    digitalWrite(CAM_CS, LOW);
    myCAM.set_fifo_burst();
  }

  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  f.flush();
  f.close();

  // ===============================================================
  // 🔄 SPI / OLED soft reinit (bez resetu kamery!)
  // ===============================================================

  deselectAll();
  SPI.endTransaction();
  SPI.end();
  delay(20);
  SPI.begin();
  deselectAll();

  // OLED szybki reinit
  digitalWrite(OLED_RST, LOW);
  delay(30);
  digitalWrite(OLED_RST, HIGH);
  delay(60);
  digitalWrite(OLED_CS, LOW);
  display.begin();
  display.fillScreen(0x0000);
  display.setCursor(10, 40);
  display.setTextColor(0xFFFF);
  display.setTextSize(1);
  display.println("Back...");
  digitalWrite(OLED_CS, HIGH);

  // --- tylko wyczyść FIFO i przywróć JPEG ---
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  myCAM.write_reg(0xFF, 0x01);
  myCAM.write_reg(0x12, 0x40);  // JPEG enable
  myCAM.OV2640_set_JPEG_size(OV2640_1600x1200);
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  // --- powrót do podglądu ---
  camSetPreviewMode();
  delay(120);
  capturePreviewRawFast(); // wymusza FIFO
  delay(50);

  return SAVE_OK;
}

// ===================== SETUP / LOOP =====================
void setup() {
  pinMode(OLED_CS, OUTPUT);
  pinMode(CAM_CS, OUTPUT);
  pinMode(SD_CS, OUTPUT);
  deselectAll();

  pinMode(SHUTTER_PIN, INPUT_PULLUP);

  SPI.begin();
  Wire.begin();

  delay(60);
  deselectAll();

  // ===== SD INIT FIRST (before OLED + CAM touch SPI) =====
  bool sdOk = SD.begin(SD_CS, SPI, SD_HZ);
  if (!sdOk) {
    // try slower once
    sdOk = SD.begin(SD_CS, SPI, 1000000);
  }

  // Now init OLED so we can show the boot UI (even if SD failed)
  deselectAll();
  digitalWrite(OLED_CS, LOW);
  display.begin();
  display.fillScreen(0x0000);
  display.setTextColor(0xFFFF);
  display.setTextSize(1);
  digitalWrite(OLED_CS, HIGH);

  // Bee splash + sequential checks
  drawBeeSplash();
  bootStatusLine("OLED", true);

  bootStatusLine("SD", sdOk);
  if (!sdOk) {
    bootFailScreen("SD", "init failed", "check 3.3V/CS/wires");
  }

  // ===== CAM SPI test =====
  SPI.beginTransaction(camSPI);
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  uint8_t t = myCAM.read_reg(ARDUCHIP_TEST1);
  SPI.endTransaction();

  bool camSpiOk = (t == 0x55);
  bootStatusLine("CAM SPI", camSpiOk);
  if (!camSpiOk) {
    bootFailScreen("CAM", "SPI test fail", "check wiring/CS/MISO");
  }

  // ===== CAM reset + init =====
  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  SPI.beginTransaction(camSPI);
  digitalWrite(CAM_CS, LOW);
  myCAM.InitCAM();
  digitalWrite(CAM_CS, HIGH);
  SPI.endTransaction();

  bootStatusLine("CAM", true);

  // ----- CAMINFO / counters -----
  bool ok = loadCamInfo();
  if (!ok) {
    uint16_t existing = scanPhotoCount();
    camInfo.magic = CAMINFO_MAGIC;
    camInfo.nextIndex = existing + 1;
    camInfo.totalShots = existing;
    saveCamInfo();
  }

  if (camInfo.nextIndex >= 1 && camInfo.nextIndex <= 9999) {
    char test[16];
    makeFilename(test, sizeof(test), camInfo.nextIndex);
    if (SD.exists(test)) {
      uint16_t existing = scanPhotoCount();
      camInfo.nextIndex = existing + 1;
      photoCount = existing;
      saveCamInfo();
    }
  }

  photoCount = camInfo.nextIndex - 1;

  // set preview mode ONCE
  camMode = MODE_PHOTO;
  camSetPreviewMode();

  // battery init
  lastVbat = readBatteryVoltage();
  lastBatMs = millis();

  lastFrameMs = millis();
  lastPreviewMs = 0;

  // short pause so you see the boot status
  delay(250);
  oledBlackFrame();
}

void loop() {
  deselectAll();
  uint32_t now = millis();

  // FPS estimate (based on loop timing)
  uint32_t dt = now - lastFrameMs;
  lastFrameMs = now;
  if (dt > 0) {
    float instFps = 1000.0f / float(dt);
    fpsEMA = (fpsEMA == 0.0f) ? instFps : (0.85f * fpsEMA + 0.15f * instFps);
  }

  // battery update (every 2s)
  if (now - lastBatMs >= 2000) {
    lastBatMs = now;
    float vb = readBatteryVoltage();
    lastVbat = 0.85f * lastVbat + 0.15f * vb;
  }

  // debounce
  bool btn = digitalRead(SHUTTER_PIN);
  if (btn != lastBtn) {
    lastDebounceMs = now;
    lastBtn = btn;
  }

  // shutter
  if (btn == LOW && (now - lastDebounceMs) > 30) {
    while (digitalRead(SHUTTER_PIN) == LOW) delay(5);

    uint16_t newIndex = photoCount + 1;
    SaveErr err = captureAndSaveFullJpeg(newIndex);

    camSetPreviewMode();

    if (err == SAVE_OK) {
      photoCount = newIndex;

      camInfo.nextIndex = newIndex + 1;
      camInfo.totalShots += 1;
      saveCamInfo();

      flashBlackFrames = 1;
    } else {
      uint32_t extra = 0;
      if (err == E_LEN) {
        SPI.beginTransaction(camSPI);
        digitalWrite(CAM_CS, LOW);
        extra = myCAM.read_fifo_length() & 0x07FFFFF;
        digitalWrite(CAM_CS, HIGH);
        SPI.endTransaction();
      }
      showErr(err, extra);
      delay(1200);
    }
  }

  // black flash after photo
  if (flashBlackFrames > 0) {
    oledBlackFrame();
    flashBlackFrames--;
    return;
  }

  // preview FPS limit
  if (now - lastPreviewMs < PREVIEW_PERIOD_MS) return;
  lastPreviewMs = now;

  camSetPreviewMode();

  if (capturePreviewRawFast()) {
    deselectAll();
    digitalWrite(OLED_CS, LOW);
    renderPreviewToOLED();
    digitalWrite(OLED_CS, HIGH);
  }
}
