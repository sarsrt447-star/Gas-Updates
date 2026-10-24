/*
 * ESP32 WROOM + MQ137 (ammonia) + ST7789 1.3" (240x240) + Passive Buzzer
 * Gas Monitor Pro  -  Made by @sars__88
 *
 * Libraries (Library Manager): Adafruit GFX Library, Adafruit ST7735 and ST7789 Library
 * Board: "ESP32 Dev Module"  (arduino-esp32 core 2.x or 3.x)
 *
 * WIRING
 *  Display : VCC->3V3  GND->GND  SCL->GPIO18  SDA->GPIO23  RES->GPIO17  DC->GPIO16  BLK->GPIO21  (no CS pin on the GMT130 module)
 *  MQ137   : VCC->5V (VIN)  GND->GND  AO->GPIO34 THROUGH A VOLTAGE DIVIDER (AO--10k--GPIO34--20k--GND)
 *  Buzzer  : (+)->GPIO26  (-)->GND   (passive buzzer)
 *  Button  : built-in BOOT button (GPIO0) = switch screens
 *
 * WiFi : connect to  "GasMonitor-Pro"  password  12345678  -> the web page opens automatically
 *        (or open http://192.168.4.1)
 */


#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <SPI.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Update.h>
#include <mbedtls/md.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>


// ---------------- Pins ----------------
#define TFT_SCK 18
#define TFT_MOSI 23
#define TFT_CS -1   // your GMT130 module has no CS pin (7 pins: GND VCC SCK SDA RES DC BLK)
#define TFT_DC 16
#define TFT_RST 17
#define TFT_BL 21
#define SENSOR_PIN 34
#define BUZZER_PIN 26
#define TFT_SPI_HZ 20000000   // 20 MHz: long jumper wires often fail at 40 MHz (black screen)
#define TFT_TEST 1            // 1 = red/green/blue test at start-up (set 0 when the screen works)
#define BTN_PIN 0


// ---------------- Sensor settings ----------------
#define VCC_SENSOR 5.0f        // MQ137 supply voltage
#define RL_KOHM 10.0f          // load resistor on the MQ module (usually 10k)
#define DIVIDER_FACTOR 1.5f    // divider: AO--10k--GPIO34--20k--GND  => (10k+20k)/20k = 1.5
#define WARMUP_MS 300000UL     // 5 minutes
// Datasheet model (NH3): ppm = 50 * ((R0/Rs) / S) ^ (1/n)
//   S = R0/Rs at 50 ppm (datasheet: >= 2), n = concentration slope (datasheet: R200/R50 <= 0.6  ->  n >= 0.37)
#define PPM_REF 50.0f
#define SENS_S 2.5f            // default S, can be calibrated from the web page with a known ppm
#define SLOPE_N 0.45f
#define PPM_MIN 5.0f           // below this the sensor cannot measure -> 0
#define PPM_MAX 500.0f         // upper limit of the sensor range (100 % = 500 ppm)


// ---------------- WiFi ----------------
const char* AP_SSID = "Gas-NH3";
const char* AP_PASS = "1234000";
IPAddress apIP(192, 168, 4, 1);


#define FW_VER "3.3"
#define FW_BUILD 33    // integer build number: raise it on every release (the device installs only a higher build)
// ---- remote update settings (change these two lines) ----
#define UPDATE_URL "https://USERNAME.github.io/REPO/version.json"   // your GitHub Pages address
#define OTA_KEY "CHANGE-THIS-SECRET-6826906-57-=767754-087--7-77Y8L,U9"                 // secret used to sign updates (same key in release_tool.html)
// ---------------------------------------------------------   // shown on the web page and at the bottom of the warm-up screen
#define W 240
#define H 240
#define C565(r, g, b) (((((int)(r)) & 0xF8) << 8) | ((((int)(g)) & 0xFC) << 3) | (((int)(b)) >> 3))
#define BLACK 0x0000
#define HDR C565(20, 30, 60)
#define DIM C565(34, 62, 82)
#define CYAN C565(0, 200, 255)
#define WHITE 0xFFFF


Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, TFT_RST);
WebServer server(80);
DNSServer dns;


// ---------------- State ----------------
float rs = 0, r0 = 1, ratio = 1, ppm = 0, pct = 0;
float minPpm = 9999, maxPpm = 0, sumPpm = 0;
uint32_t cntPpm = 0;
bool warm = true;
uint32_t warmStart = 0;
uint8_t screen = 0;
bool muted = false;
uint8_t brightness = 100;
uint8_t alarmThr = 20;
int lastStep = 0;
uint32_t beepEnd = 0, lastBeep = 0, lastAlarm = 0, lastSample = 0, lastUi = 0;
int lastWarmSec = -1;
float lgS = 0;
float sensS = SENS_S;
bool alarmOn = false;
Preferences prefs;
bool calibrating = false;
uint32_t calStart = 0;
float calSum = 0;
int calN = 0;
#define CAL_MS 10000UL
#define HN 228
uint8_t hist[HN];
int histCount = 0;
bool histDirty = true;
uint32_t lastHist = 0;


// ---------------- Backlight PWM (core 2.x / 3.x) ----------------
void setBacklight() {
  uint32_t duty = map(brightness, 0, 100, 0, 255);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(TFT_BL, duty);
#else
  ledcWrite(4, duty);
#endif
}
void initBacklight() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(TFT_BL, 5000, 8);
#else
  ledcSetup(4, 5000, 8);
  ledcAttachPin(TFT_BL, 4);
#endif
  setBacklight();
}


// ---------------- Buzzer ----------------
void beep(int f, int d) {
  if (muted) return;
  tone(BUZZER_PIN, f);
  beepEnd = millis() + d;
}
void handleBeep() {
  if (beepEnd && millis() > beepEnd) {
    beepEnd = 0;
    if (alarmOn) tone(BUZZER_PIN, 2600);  // keep the continuous alarm going
    else noTone(BUZZER_PIN);
  }
}
void note(int f, int d) {  // blocking, used for melodies
  tone(BUZZER_PIN, f);
  delay(d);
  noTone(BUZZER_PIN);
  delay(25);
}


// ---------------- Helpers ----------------
uint16_t levelColor(float p) {
  if (p < 30) return C565(0, 220, 120);
  if (p < 55) return C565(200, 220, 0);
  if (p < 80) return C565(255, 140, 0);
  return C565(255, 50, 60);
}
const char* statusText(float p) {
  if (p >= 99.5f) return "OVER";  // above the sensor range (500 ppm)
  if (p < 10) return "CLEAN AIR";
  if (p < 30) return "LOW";
  if (p < 55) return "MODERATE";
  if (p < 80) return "HIGH";
  return "DANGER";
}
void resetStats() {
  minPpm = 9999; maxPpm = 0; sumPpm = 0; cntPpm = 0;
}


// ---------------- Sensor ----------------
float readRs() {
  uint32_t s = 0;
  for (int i = 0; i < 16; i++) s += analogReadMilliVolts(SENSOR_PIN);
  float v = (s / 16.0f) / 1000.0f * DIVIDER_FACTOR;
  if (v < 0.02f) v = 0.02f;
  if (v >= VCC_SENSOR) v = VCC_SENSOR - 0.01f;
  return RL_KOHM * (VCC_SENSOR - v) / v;
}
void calibrate() {  // non-blocking, assumes clean air
  calibrating = true; calStart = millis(); calSum = 0; calN = 0;
  ppm = 0; pct = 0; lgS = 0; lastStep = 0; ratio = 1;
}
void sampleSensor() {
  rs = readRs();
  if (warm) return;
  if (calibrating) {
    calSum += rs; calN++;
    if (millis() - calStart >= CAL_MS) {
      r0 = calSum / calN; calibrating = false; ratio = 1;
      resetStats(); histCount = 0; beep(1500, 150);
    }
    return;
  }
  ratio = rs / r0;
  float L = constrain(log10f(1.0f / ratio), -0.3f, 3.0f);
  lgS = lgS * 0.8f + L * 0.2f;                         // smoothing (log domain)
  float rho = powf(10.0f, lgS);                        // R0 / Rs
  float c = PPM_REF * powf(rho / sensS, 1.0f / SLOPE_N);
  ppm = (c < PPM_MIN) ? 0.0f : min(c, PPM_MAX);
  pct = ppm / PPM_MAX * 100.0f;                        // % of the sensor range (0..500 ppm)
  if (ppm < minPpm) minPpm = ppm;
  if (ppm > maxPpm) maxPpm = ppm;
  sumPpm += ppm; cntPpm++;
  // buzzer synchronised with every rise / fall step (5 % each)
  int step = (int)(pct / 5.0f);
  if (step != lastStep && !alarmOn && millis() - lastBeep > 120) {
    if (step > lastStep) beep(1200 + step * 70, 70);  // rising -> higher pitch
    else beep(650, 70);                                // falling -> low pitch
    lastBeep = millis();
  }
  lastStep = step;
}


// ---------------- Display: common ----------------
float dPct = 0, dPpm = 0;
#define BG C565(12, 32, 48)
#define GRAY C565(150, 170, 190)
#define EDGE C565(60, 90, 120)
const char* devStatus() { return calibrating ? "CALIB." : statusText(pct); }
uint16_t curColor() { return calibrating ? C565(255, 190, 60) : levelColor(dPct); }
void drawHeader(const char* t) {
  tft.fillRect(0, 0, W, 28, HDR);
  tft.setTextSize(2);
  tft.setTextColor(CYAN);
  tft.setCursor(10, 7);
  tft.print(t);
  tft.drawFastHLine(0, 28, W, C565(0, 120, 160));
}
void drawDots() {
  tft.fillRect(80, 224, 80, 16, BG);
  for (int i = 0; i < 4; i++) {
    int x = W / 2 - 15 + i * 10;
    if (i == screen) tft.fillCircle(x, 232, 4, CYAN);
    else tft.fillCircle(x, 232, 2, C565(80, 90, 110));
  }
}
void printPad(const char* s, int width) {
  char b[24];
  snprintf(b, sizeof b, "%-*s", width, s);
  tft.print(b);
}
void label(int x, int y, const char* s) {
  tft.setTextSize(1); tft.setTextColor(GRAY, BG); tft.setCursor(x, y); tft.print(s);
}
void histPush() {
  if (histCount < HN) hist[histCount++] = (uint8_t)pct;
  else { memmove(hist, hist + 1, HN - 1); hist[HN - 1] = (uint8_t)pct; }
  histDirty = true;
}
void drawTrend(int x, int y, int w, int h) {
  int b = h / 3;
  tft.fillRect(x, y, w, b, C565(62, 58, 100));
  tft.fillRect(x, y + b, w, b, C565(64, 80, 72));
  tft.fillRect(x, y + 2 * b, w, h - 2 * b, C565(18, 86, 98));
  for (int d = 0; d < w; d += 8) {
    tft.drawFastHLine(x + d, y + b, 4, C565(140, 150, 170));
    tft.drawFastHLine(x + d, y + 2 * b, 4, C565(140, 150, 170));
  }
  tft.drawRect(x - 1, y - 1, w + 2, h + 2, EDGE);
  int n = min(histCount, w), px = x + w - 1, py = y + h - 2;
  if (n < 2) tft.drawFastHLine(x, y + h - 2, w, WHITE);
  for (int i = 0; i < n; i++) {
    int xx = x + w - n + i, yy = y + h - 2 - (hist[histCount - n + i] * (h - 4)) / 100;
    if (i) tft.drawLine(px, py, xx, yy, WHITE);
    px = xx; py = yy;
  }
  tft.fillCircle(px, py, 3, curColor());
}


// ---------------- Smooth helpers ----------------
void smoothValues() {  // eased copies used only for drawing
  float tp = calibrating ? 0 : pct, tq = calibrating ? 0 : ppm;
  dPct += (tp - dPct) * 0.18f; if (fabsf(tp - dPct) < 0.05f) dPct = tp;
  dPpm += (tq - dPpm) * 0.18f; if (fabsf(tq - dPpm) < 0.05f) dPpm = tq;
}
struct TS { char c[24]; uint16_t col; };
TS ts[6];
void resetTS() { for (int i = 0; i < 6; i++) { ts[i].c[0] = '\1'; ts[i].c[1] = 0; ts[i].col = 0; } }
void putText(int slot, int x, int y, int sz, uint16_t col, const char* s) {  // draws only when changed
  TS& t = ts[slot];
  if (t.col == col && strcmp(t.c, s) == 0) return;
  strncpy(t.c, s, 23); t.c[23] = 0; t.col = col;
  tft.setTextSize(sz); tft.setTextColor(col, BG); tft.setCursor(x, y); tft.print(s);
}
void centered(char* out, int n, int width, const char* s) {
  int lp = (width - (int)strlen(s)) / 2; if (lp < 0) lp = 0;
  snprintf(out, n, "%*s%-*s", lp, "", width - lp, s);
}
void blDuty(uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(TFT_BL, d);
#else
  ledcWrite(4, d);
#endif
}
void fadeBL(bool up) {  // smooth fade used when switching screens
  uint32_t m = map(brightness, 0, 100, 0, 255);
  for (int i = 0; i <= 8; i++) { blDuty(up ? m * i / 8 : m * (8 - i) / 8); delay(12); }
}


// ---------------- Screen 0: Classic (ring + bubbles + trend) ----------------
#define NS 60
int lastLit = -1;
struct Bub { float x, y, ph; int16_t ox, oy; uint8_t r; bool alive; };
Bub bub[8];
void ringSeg(int i, bool lit) {
  float a = (135.0f + i * 270.0f / (NS - 1)) * DEG_TO_RAD;
  tft.fillCircle(62 + (int)(44 * cosf(a)), 72 + (int)(44 * sinf(a)), 3, lit ? levelColor(i * 100.0f / NS) : DIM);
}
void s0Static() {
  tft.fillScreen(BG); resetTS();
  for (int i = 0; i < NS; i++) ringSeg(i, false);
  lastLit = 0;
  tft.drawRoundRect(124, 30, 110, 92, 8, EDGE);
  for (int i = 0; i < 8; i++) bub[i].alive = false;
  label(8, 130, "GAS TREND  NH3");
  label(4, 146, "HIGH"); label(4, 172, "MID"); label(4, 198, "LOW");
}
void s0Update() {
  uint16_t col = curColor(); char b[24];
  int lit = (int)(dPct * NS / 100.0f + 0.5f);
  if (lit != lastLit) {
    int step = lit > lastLit ? 1 : -1;  // fills / empties a few segments per frame
    int to = lastLit + step * 3;
    if ((step > 0 && to > lit) || (step < 0 && to < lit)) to = lit;
    for (int i = min(lastLit, to); i < max(lastLit, to); i++) ringSeg(i, i < to);
    lastLit = to;
  }
  snprintf(b, sizeof b, "%3d%%", (int)(dPct + 0.5f)); putText(0, 26, 52, 3, WHITE, b);
  snprintf(b, sizeof b, "%5.1f ppm", dPpm); putText(1, 35, 84, 1, GRAY, b);
  snprintf(b, sizeof b, "Rs %-6.1fk", rs); putText(2, 35, 112, 1, GRAY, b);
  putText(3, 126, 8, 2, col, (snprintf(b, sizeof b, "%-9s", devStatus()), b));
  int act = 1 + (int)(dPct / 14);
  float sp = 0.7f + dPct / 22.0f;
  for (int i = 0; i < 8; i++) {
    Bub& k = bub[i];
    if (k.alive) tft.drawCircle(k.ox, k.oy, k.r, BG);
    if (!k.alive && i < act && random(100) < 7) {
      k.alive = true; k.r = random(3, 7); k.x = random(140, 218); k.y = 116 - k.r; k.ph = random(628) / 100.0f;
    }
    if (k.alive) {
      k.y -= sp;
      if (k.y < 38 + k.r) k.alive = false;
      else {
        k.ox = (int)(k.x + 3 * sinf(k.ph + k.y * 0.08f)); k.oy = (int)k.y;
        tft.drawCircle(k.ox, k.oy, k.r, col);
      }
    }
  }
  if (histDirty) {
    drawTrend(38, 142, 196, 68);
    tft.setTextSize(1); tft.setTextColor(WHITE, BG); tft.setCursor(150, 130);
    snprintf(b, sizeof b, "%3d%% %5.1fppm", (int)(dPct + 0.5f), dPpm); tft.print(b);
  }
}


// ---------------- Screen 1: Big number ----------------
int lastLit1 = -1;
void s1Static() { tft.fillScreen(BG); resetTS(); lastLit1 = -1; label(10, 10, "GAS LEVEL"); label(190, 10, "NH3"); }
void s1Update() {
  uint16_t col = curColor(); char b[32];
  snprintf(b, sizeof b, "%3d", (int)(dPct + 0.5f)); putText(0, 14, 28, 10, col, b);
  putText(1, 200, 76, 4, col, "%");
  snprintf(b, sizeof b, "%6.1f", dPpm); putText(2, 10, 118, 3, WHITE, b);
  if (ts[3].c[0] == '\1') { tft.setTextSize(2); tft.setTextColor(GRAY, BG); tft.setCursor(124, 126); tft.print("PPM"); ts[3].c[0] = 'p'; ts[3].c[1] = 0; }
  int lit = (int)(dPct / 10);
  if (lit != lastLit1) {
    lastLit1 = lit;
    for (int i = 0; i < 10; i++) tft.fillRect(10 + i * 22, 156, 20, 14, i < lit ? levelColor(i * 10 + 5) : DIM);
  }
  char s[16]; centered(s, sizeof s, 13, calibrating ? "CALIBRATING" : statusText(pct));
  putText(4, 3, 180, 3, col, s);
  snprintf(b, sizeof b, "Rs %.1fk   R0 %.1fk    ", rs, r0); putText(5, 10, 212, 1, GRAY, b);
}


// ---------------- Screen 2: Data cards + trend ----------------
void card(int x, int y, const char* l) { tft.drawRoundRect(x, y, 112, 52, 8, EDGE); label(x + 8, y + 6, l); }
void s2Static() {
  tft.fillScreen(BG); resetTS();
  card(6, 6, "PPM"); card(122, 6, "PERCENT"); card(6, 62, "Rs  kOhm"); card(122, 62, "Rs / R0");
  label(6, 122, "GAS TREND  NH3");
}
void s2Update() {
  char b[16];
  const float v[4] = {dPpm, dPct, rs, ratio};
  for (int i = 0; i < 4; i++) {
    snprintf(b, sizeof b, i == 3 ? "%-7.2f" : "%-7.1f", v[i]);
    putText(i, 14 + (i % 2) * 116, 28 + (i / 2) * 56, 2, i == 1 ? curColor() : WHITE, b);
  }
  if (histDirty) drawTrend(6, 134, 228, 82);
}


// ---------------- Screen 3: Pulse ----------------
float ph = 0;
int prevR[3] = {30, 48, 66};
uint16_t lastCore = 0;
void s3Static() { tft.fillScreen(BG); resetTS(); lastCore = 0; label(10, 10, "PULSE"); label(190, 10, "NH3"); }
void s3Update() {
  uint16_t col = curColor(); char b[24];
  ph += 0.35f + dPct / 60.0f;
  for (int i = 0; i < 3; i++) {
    tft.drawCircle(120, 80, prevR[i], BG); tft.drawCircle(120, 80, prevR[i] + 1, BG);
    int r = 26 + (int)fmodf(ph + i * 15, 46.0f);
    prevR[i] = r;
    uint16_t c = r < 42 ? col : (r < 58 ? C565(70, 120, 140) : DIM);
    tft.drawCircle(120, 80, r, c); tft.drawCircle(120, 80, r + 1, c);
  }
  if (col != lastCore) { lastCore = col; tft.fillCircle(120, 80, 20, col); }
  snprintf(b, sizeof b, "%3d%%", (int)(dPct + 0.5f)); putText(0, 72, 160, 4, WHITE, b);
  char s[16]; centered(s, sizeof s, 13, devStatus());
  putText(1, 42, 196, 2, col, s);
  snprintf(b, sizeof b, "%7.1f ppm  ", dPpm); putText(2, 84, 216, 1, GRAY, b);
}


// ---------------- Screen switching ----------------
void changeScreen(uint8_t n) {
  fadeBL(false);
  screen = n % 4;
  switch (screen) {
    case 0: s0Static(); break;
    case 1: s1Static(); break;
    case 2: s2Static(); break;
    case 3: s3Static(); break;
  }
  drawDots();
  lastUi = 0;
  histDirty = true;
  switch (screen) {
    case 0: s0Update(); break;
    case 1: s1Update(); break;
    case 2: s2Update(); break;
    default: s3Update(); break;
  }
  histDirty = false;
  fadeBL(true);
}


// ---------------- Warm-up screen ----------------
void warmStatic() {
  tft.fillScreen(BLACK);
  drawHeader("WARM-UP");
  tft.setTextSize(2); tft.setTextColor(C565(255, 150, 0));
  tft.setCursor(34, 50); tft.print("HEATING SENSOR");
  tft.setTextSize(1); tft.setTextColor(C565(120, 130, 150));
  tft.setCursor(40, 196); tft.print("Please keep the sensor in clean air");
  tft.setCursor(52, 208); tft.print("Auto-calibration at the end");
  tft.setTextColor(GRAY); tft.setCursor(8, 226); tft.print("FW v" FW_VER);
  lastWarmSec = -1;
}
void warmUpdate() {
  uint32_t el = millis() - warmStart;
  int left = (el >= WARMUP_MS) ? 0 : (WARMUP_MS - el) / 1000;
  if (left == lastWarmSec) return;
  lastWarmSec = left;
  char b[16];
  tft.setTextSize(6); tft.setTextColor(WHITE, BLACK); tft.setCursor(30, 76);
  snprintf(b, sizeof b, "%02d:%02d", left / 60, left % 60); tft.print(b);
  float p = (float)el / WARMUP_MS; if (p > 1) p = 1;
  int bw = (int)(220 * p);
  tft.fillRect(10, 136, bw, 18, C565(255, 140 - (int)(p * 100), 0));
  tft.fillRect(10 + bw, 136, 220 - bw, 18, DIM);
  tft.setTextSize(2); tft.setTextColor(WHITE, BLACK); tft.setCursor(92, 162);
  snprintf(b, sizeof b, "%3d%%", (int)(p * 100)); tft.print(b);
}
void endWarmup() {
  warm = false;
  calibrate();
  note(1000, 120); note(1500, 120); note(2000, 200);
  changeScreen(screen);
}


// ---------------- Intro ----------------
void introTone() {
  note(523, 120); note(659, 120); note(784, 120); note(1047, 250);
}
void helloFade() {
  tft.fillScreen(BLACK);
  tft.setTextSize(5);
  for (int i = 0; i <= 255; i += 5) {
    tft.setTextColor(C565(0, i * 0.8, i));
    tft.setCursor(45, 100); tft.print("HELLO");
    delay(14);
  }
  delay(600);
  for (int i = 255; i >= 0; i -= 5) {
    tft.setTextColor(C565(0, i * 0.8, i));
    tft.setCursor(45, 100); tft.print("HELLO");
    delay(14);
  }
  tft.fillScreen(BLACK);
}


// ---------------- Mini games (controlled from the web page) ----------------
uint8_t gameMode = 0;  // 0 = off, 1 = Snake, 2 = Breakout, 3 = Flappy, 4 = Dodge
bool gOver = false;
int gScore = 0, gLastScore = -1, gBest[5] = {0, 0, 0, 0, 0};
int kMove = 0; char kKey = 0; bool kA = false;  // key events coming from the web page
uint32_t gNext = 0;


// non-blocking sound sequence (uses beep(), so mute and the gas alarm still work)
int sq[5][2]; int sqN = 0, sqI = 0; uint32_t sqNext = 0;   // each note = {frequency, duration ms}
void melody(const int n[][2], int c) { for (int i = 0; i < c; i++) { sq[i][0] = n[i][0]; sq[i][1] = n[i][1]; } sqN = c; sqI = 0; sqNext = 0; }
void updateSfx() {
  if (sqI < sqN && millis() >= sqNext) { beep(sq[sqI][0], sq[sqI][1]); sqNext = millis() + sq[sqI][1] + 25; sqI++; }
}
void loseSound() { static const int n[4][2] = {{600, 120}, {450, 120}, {330, 120}, {200, 320}}; melody(n, 4); }


void gBar(const char* name) {
  tft.fillScreen(BG);
  tft.fillRect(0, 0, W, 24, HDR);
  tft.setTextSize(2); tft.setTextColor(CYAN); tft.setCursor(6, 5); tft.print(name);
  gLastScore = -1;
}
void gDrawScore() {
  if (gScore == gLastScore) return;
  gLastScore = gScore;
  char b[12]; snprintf(b, sizeof b, "%5d", gScore);
  tft.setTextSize(2); tft.setTextColor(WHITE, HDR); tft.setCursor(150, 5); tft.print(b);
}
void gEnd() {
  gOver = true;
  if (gScore > gBest[gameMode]) gBest[gameMode] = gScore;
  loseSound();
  char b[24];
  tft.fillRoundRect(30, 96, 180, 100, 10, BG);
  tft.drawRoundRect(30, 96, 180, 100, 10, C565(235, 60, 60));
  tft.setTextSize(2); tft.setTextColor(C565(235, 60, 60), BG); tft.setCursor(48, 106); tft.print("GAME OVER");
  tft.setTextColor(WHITE, BG); tft.setCursor(48, 134);
  snprintf(b, sizeof b, "Score %d", gScore); tft.print(b);
  tft.setTextSize(1); tft.setTextColor(GRAY, BG); tft.setCursor(48, 160);
  snprintf(b, sizeof b, "Best %d", gBest[gameMode]); tft.print(b);
  tft.setCursor(48, 178); tft.print("Press A to play again");
}


// ----- 1) Snake -----
#define SC 15
uint8_t snX[SC * SC], snY[SC * SC];
int snLen, snDir, apX, apY;
uint32_t snDelay;
void snCell(int x, int y, uint16_t c) { tft.fillRect(x * 16 + 1, 25 + y * 16, 14, 14, c); }
void snApple() {
  bool ok;
  do {
    apX = random(SC); apY = random(13); ok = true;
    for (int i = 0; i < snLen; i++) if (snX[i] == apX && snY[i] == apY) ok = false;
  } while (!ok);
  tft.fillCircle(apX * 16 + 8, 32 + apY * 16, 6, C565(235, 60, 60));
  tft.fillRect(apX * 16 + 8, 25 + apY * 16, 2, 4, C565(90, 200, 90));
}
void snakeStart() {
  gBar("SNAKE");
  tft.drawRect(0, 24, 240, 210, EDGE);
  snLen = 3; snDir = 1; snDelay = 170;
  for (int i = 0; i < snLen; i++) { snX[i] = 7 - i; snY[i] = 6; snCell(snX[i], snY[i], i ? C565(0, 150, 110) : C565(0, 230, 140)); }
  snApple();
}
void snakeTick() {
  kMove = 0;
  if (kKey) {
    int nd = kKey == 'u' ? 0 : kKey == 'r' ? 1 : kKey == 'd' ? 2 : 3;
    if ((nd + 2) % 4 != snDir) snDir = nd;
    kKey = 0;
  }
  int nx = snX[0] + (snDir == 1) - (snDir == 3), ny = snY[0] + (snDir == 2) - (snDir == 0);
  if (nx < 0 || ny < 0 || nx >= SC || ny >= 13) { gEnd(); return; }
  bool eat = (nx == apX && ny == apY);
  for (int i = 0; i < snLen - (eat ? 0 : 1); i++) if (snX[i] == nx && snY[i] == ny) { gEnd(); return; }
  if (!eat) snCell(snX[snLen - 1], snY[snLen - 1], BG);
  else { snLen++; gScore += 10; beep(1400, 60); if (snDelay > 70) snDelay -= 6; }
  for (int i = snLen - 1; i > 0; i--) { snX[i] = snX[i - 1]; snY[i] = snY[i - 1]; }
  snX[0] = nx; snY[0] = ny;
  snCell(snX[1], snY[1], C565(0, 150, 110));
  snCell(nx, ny, C565(0, 230, 140));
  if (eat) snApple();
}


// ----- 2) Breakout -----
bool bk[6][8];
int bkLeft, pdx, bx, by, bvx, bvy;
uint16_t brickColor(int r) {
  static const uint16_t c[6] = {C565(235, 60, 60), C565(255, 140, 0), C565(240, 210, 0), C565(60, 200, 90), C565(40, 170, 230), C565(150, 100, 240)};
  return c[r];
}
void bkBricks() {
  for (int r = 0; r < 6; r++) for (int c = 0; c < 8; c++) { bk[r][c] = true; tft.fillRect(8 + c * 28 + 1, 40 + r * 14 + 1, 26, 12, brickColor(r)); }
  bkLeft = 48;
}
void breakoutStart() {
  gBar("BREAKOUT");
  pdx = 98; bx = 117; by = 196; bvx = 2; bvy = -3;
  bkBricks();
  tft.fillRect(pdx, 222, 44, 6, WHITE);
}
void breakoutTick() {
  kKey = 0;
  if (kMove) {
    tft.fillRect(pdx, 222, 44, 6, BG);
    pdx = constrain(pdx + kMove * 16, 0, W - 44); kMove = 0;
    tft.fillRect(pdx, 222, 44, 6, WHITE);
  }
  tft.fillRect(bx, by, 6, 6, BG);
  bx += bvx; by += bvy;
  if (bx < 0) { bx = 0; bvx = -bvx; beep(500, 15); }
  if (bx > W - 6) { bx = W - 6; bvx = -bvx; beep(500, 15); }
  if (by < 25) { by = 25; bvy = -bvy; beep(500, 15); }
  if (bvy > 0 && by + 6 >= 222 && by < 228 && bx + 6 >= pdx && bx <= pdx + 44) {
    by = 216; bvy = -bvy;
    bvx = constrain((bx + 3 - (pdx + 22)) / 5, -4, 4);
    if (bvx == 0) bvx = random(2) ? 1 : -1;
    beep(800, 25);
  }
  if (by > 240) { gEnd(); return; }
  int cx = bx + 3, cy = by + 3;
  if (cx >= 8 && cy >= 40) {
    int c = (cx - 8) / 28, r = (cy - 40) / 14;
    if (r < 6 && c < 8 && bk[r][c]) {
      bk[r][c] = false; tft.fillRect(8 + c * 28 + 1, 40 + r * 14 + 1, 26, 12, BG);
      bvy = -bvy; gScore += 10; bkLeft--; beep(1000 + r * 120, 30);
      if (bkLeft == 0) { bkBricks(); beep(2000, 150); }
    }
  }
  tft.fillRect(bx, by, 6, 6, C565(255, 220, 80));
}


// ----- 3) Flappy -----
float fy, fv;
int pX[2], pG[2]; bool pP[2];
void flappyStart() {
  gBar("FLAPPY");
  fy = 130; fv = -3; pX[0] = 240; pX[1] = 410;
  for (int i = 0; i < 2; i++) { pG[i] = random(60, 130); pP[i] = false; }
}
void flappyTick() {
  kMove = 0;
  if (kA || kKey == 'u') { fv = -5.4f; kA = false; kKey = 0; beep(900, 20); }
  kKey = 0;
  tft.fillCircle(60, (int)fy, 8, BG);
  fv += 0.55f; fy += fv;
  bool hit = (fy < 33 || fy > 232);
  for (int i = 0; i < 2; i++) {
    pX[i] -= 3;
    if (pX[i] < -40) { pX[i] = pX[1 - i] + 170; pG[i] = random(60, 130); pP[i] = false; }
    tft.fillRect(pX[i] + 34, 25, 3, 215, BG);
    tft.fillRect(pX[i], 25, 34, pG[i] - 25, C565(60, 190, 90));
    tft.fillRect(pX[i], pG[i] + 80, 34, 240 - pG[i] - 80, C565(60, 190, 90));
    if (!pP[i] && pX[i] + 34 < 52) { pP[i] = true; gScore++; beep(1500, 40); }
    if (68 > pX[i] && 52 < pX[i] + 34 && (fy - 8 < pG[i] || fy + 8 > pG[i] + 80)) hit = true;
  }
  tft.fillCircle(60, (int)fy, 8, C565(255, 210, 60));
  tft.fillCircle(63, (int)fy - 2, 2, WHITE);
  if (hit) gEnd();
}


// ----- 4) Dodge -----
int dpx, rkx[6], rky[6], rkw[6], rksp[6]; bool rka[6];
void dodgeStart() {
  gBar("DODGE");
  dpx = 108;
  for (int i = 0; i < 6; i++) rka[i] = false;
  tft.fillRect(dpx, 216, 24, 12, CYAN);
}
void dodgeTick() {
  kKey = 0;
  if (kMove) {
    tft.fillRect(dpx, 216, 24, 12, BG);
    dpx = constrain(dpx + kMove * 14, 0, W - 24); kMove = 0;
  }
  if (random(100) < 6 + gScore / 8) {
    for (int i = 0; i < 6; i++) if (!rka[i]) {
      rka[i] = true; rkw[i] = random(14, 34); rkx[i] = random(0, W - rkw[i]); rky[i] = 26; rksp[i] = random(3, 6) + gScore / 40;
      break;
    }
  }
  bool hit = false;
  for (int i = 0; i < 6; i++) if (rka[i]) {
    tft.fillRect(rkx[i], rky[i], rkw[i], rkw[i], BG);
    rky[i] += rksp[i];
    if (rky[i] >= 240) { rka[i] = false; gScore++; beep(1100, 25); continue; }
    if (rky[i] + rkw[i] > 216 && rky[i] < 228 && rkx[i] + rkw[i] > dpx && rkx[i] < dpx + 24) hit = true;
    tft.fillRect(rkx[i], rky[i], rkw[i], rkw[i], C565(255, 140 + i * 15, 40));
  }
  tft.fillRect(dpx, 216, 24, 12, CYAN);
  if (hit) gEnd();
}


// ----- game control -----
void gameStart() {
  kMove = 0; kKey = 0; kA = false; gOver = false; gScore = 0; gNext = 0; sqN = 0; sqI = 0;
  switch (gameMode) {
    case 1: snakeStart(); break;
    case 2: breakoutStart(); break;
    case 3: flappyStart(); break;
    default: dodgeStart(); break;
  }
  gDrawScore();
}
void gameLoop(uint32_t now) {
  if (gOver) {
    if (kA) { kA = false; gameStart(); }
    kMove = 0; kKey = 0;
    return;
  }
  if (now < gNext) return;
  gNext = now + (gameMode == 1 ? snDelay : gameMode == 2 ? 25 : 30);
  switch (gameMode) {
    case 1: snakeTick(); break;
    case 2: breakoutTick(); break;
    case 3: flappyTick(); break;
    default: dodgeTick(); break;
  }
  if (!gOver) gDrawScore();
}


// ---------------- Remote update (OTA) from GitHub Pages ----------------
// The device asks UPDATE_URL for a small version.json. If "build" is newer than FW_BUILD it can download the .bin,
// check its HMAC-SHA256 signature (made with OTA_KEY by release_tool.html) and install it. Unsigned files are rejected.
String wSsid, wPass, otaVer, otaUrl, otaSig, otaMsg;
bool otaAuto = true, otaHas = false, otaGo = false;
int otaBuild = 0, otaSize = 0;
uint32_t otaLast = 0;


void wifiBegin() {  // reconnect to the customer's home Wi-Fi (saved in flash)
  wSsid = prefs.getString("wssid", ""); wPass = prefs.getString("wpass", "");
  otaAuto = prefs.getBool("oauto", true);
  if (wSsid.length()) WiFi.begin(wSsid.c_str(), wPass.c_str());
}
String jStr(const String& b, const char* k) {
  int i = b.indexOf(String("\"") + k + "\""); if (i < 0) return "";
  i = b.indexOf(':', i); int q1 = b.indexOf('"', i);
  if (i < 0 || q1 < 0) return "";
  int q2 = b.indexOf('"', q1 + 1);
  return q2 < 0 ? "" : b.substring(q1 + 1, q2);
}
long jInt(const String& b, const char* k) {
  int i = b.indexOf(String("\"") + k + "\""); if (i < 0) return 0;
  i = b.indexOf(':', i);
  return i < 0 ? 0 : b.substring(i + 1).toInt();
}
bool otaCheck() {
  otaHas = false;
  if (WiFi.status() != WL_CONNECTED) { otaMsg = "no internet"; return false; }
  WiFiClientSecure c; c.setInsecure();  // authenticity comes from the HMAC signature, not from TLS
  HTTPClient h; h.setTimeout(10000); h.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!h.begin(c, String(UPDATE_URL) + "?r=" + String(millis()))) { otaMsg = "bad url"; return false; }
  int code = h.GET();
  if (code != 200) { otaMsg = "server " + String(code); h.end(); return false; }
  String body = h.getString(); h.end();
  otaBuild = jInt(body, "build"); otaVer = jStr(body, "version"); otaUrl = jStr(body, "url");
  otaSig = jStr(body, "sig"); otaSize = jInt(body, "size");
  otaHas = otaBuild > FW_BUILD && otaUrl.length() && otaSig.length() && otaSize > 0;
  otaMsg = otaHas ? "" : "up to date";
  return true;
}
void otaScreen(int p) {
  static int last = -2;
  if (p == -1) {
    tft.fillScreen(BG);
    tft.setTextSize(2); tft.setTextColor(CYAN, BG); tft.setCursor(60, 70); tft.print("UPDATING");
    tft.setTextSize(1); tft.setTextColor(GRAY, BG); tft.setCursor(46, 200); tft.print("Do not turn off the power");
    last = -2; return;
  }
  if (p == last) return;
  last = p;
  char b[8]; snprintf(b, sizeof b, "%3d%%", p);
  tft.setTextSize(4); tft.setTextColor(WHITE, BG); tft.setCursor(60, 110); tft.print(b);
  tft.fillRect(20, 160, p * 200 / 100, 14, CYAN);
  tft.fillRect(20 + p * 200 / 100, 160, 200 - p * 200 / 100, 14, DIM);
}
bool otaInstall() {
  if (WiFi.status() != WL_CONNECTED || !otaHas) { otaMsg = "no internet"; return false; }
  otaScreen(-1);
  WiFiClientSecure c; c.setInsecure();
  HTTPClient h; h.setTimeout(15000); h.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  if (!h.begin(c, otaUrl)) { otaMsg = "bad url"; return false; }
  int code = h.GET(), len = h.getSize();
  if (code != 200 || len != otaSize || !Update.begin(len)) { otaMsg = "download failed"; h.end(); return false; }
  mbedtls_md_context_t ctx; mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, (const unsigned char*)OTA_KEY, strlen(OTA_KEY));
  Stream* st = h.getStreamPtr();
  uint8_t buf[1024]; int got = 0; uint32_t lastData = millis(); bool bad = false;
  while (got < len && !bad) {
    int av = st->available();
    if (av > 0) {
      int n = st->readBytes(buf, min(av, (int)sizeof buf));
      if (Update.write(buf, n) != (size_t)n) { bad = true; break; }
      mbedtls_md_hmac_update(&ctx, buf, n); got += n; lastData = millis();
      otaScreen(got * 100 / len);
    } else if (millis() - lastData > 15000 || !h.connected()) break;
    else delay(2);
  }
  unsigned char out[32]; mbedtls_md_hmac_finish(&ctx, out); mbedtls_md_free(&ctx); h.end();
  char hex[65]; for (int i = 0; i < 32; i++) sprintf(hex + i * 2, "%02x", out[i]);
  if (bad || got != len || !otaSig.equalsIgnoreCase(hex)) {
    Update.abort(); otaMsg = (!bad && got == len) ? "bad signature" : "download failed"; return false;
  }
  if (!Update.end(true)) { otaMsg = "install failed"; return false; }
  otaScreen(100); delay(1000); ESP.restart();
  return true;
}
void otaLoop() {
  uint32_t now = millis();
  if (!warm && !gameMode && WiFi.status() == WL_CONNECTED && (otaLast == 0 ? now > 60000UL : now - otaLast > 21600000UL)) {
    otaLast = now; otaCheck();                 // first check 1 min after boot, then every 6 hours
    if (otaHas && otaAuto) otaGo = true;
  }
  if (otaGo) {
    otaGo = false;
    if (!otaInstall()) { if (warm) warmStatic(); else changeScreen(screen); }
  }
}


// ---------------- Web ----------------
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="ar" dir="rtl"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Gas Monitor</title>
<style>
:root{--bg:#0b1823;--cd:#122a3a;--bd:#1f3b52;--tx:#eaf2f8;--mu:#8ea4b7;--ac:#2fd3b0;--am:#f0b429}
*{box-sizing:border-box;margin:0}
body{background:var(--bg);color:var(--tx);font-family:system-ui,"Segoe UI",Tahoma,Arial,sans-serif;padding:18px;max-width:560px;margin:auto}
.top{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:18px}
h1{font-size:30px}.sub{color:var(--mu);margin-top:6px}
.tools{display:flex;align-items:center;gap:12px;color:var(--mu);font-size:14px}
.lg{border:1px solid var(--bd);background:none;color:var(--tx);border-radius:12px;padding:10px 16px;font-size:16px;cursor:pointer}
.dot{display:inline-block;width:12px;height:12px;border-radius:50%;background:#e5484d;margin-inline-end:6px}.dot.on{background:var(--ac)}
.card{background:var(--cd);border:1px solid var(--bd);border-radius:28px;padding:22px;margin-bottom:18px}
.hero{display:flex;justify-content:space-between;align-items:flex-end;direction:ltr;padding:14px 4px 0;gap:10px}
.big{font-size:clamp(44px,13vw,60px);font-weight:800;line-height:1;flex-shrink:0;white-space:nowrap}.big small{font-size:.5em;color:var(--mu)}
.pp{font-size:clamp(26px,8vw,36px);font-weight:800;line-height:1;min-width:0;overflow-wrap:anywhere}.pp small{display:block;font-size:15px;color:var(--mu);font-weight:400;margin-top:4px}
.chip{display:flex;align-items:center;gap:10px;padding:14px 22px;border-radius:99px;font-weight:700;width:max-content;max-width:100%;margin:20px 0;color:var(--ac);background:#2fd3b024}
.chip i{width:14px;height:14px;border-radius:50%;background:var(--ac)}
.bar{height:14px;border-radius:9px;background:#0c1a26;overflow:hidden;direction:ltr}.bar b{display:block;height:100%;width:0;background:var(--ac);transition:.5s}
.sc0{display:flex;justify-content:space-between;color:var(--mu);margin:10px 4px 22px;direction:ltr}
.meta{display:flex;justify-content:space-between;color:var(--mu);margin-bottom:10px}
canvas{width:100%;height:220px;display:block}
.note{display:none;background:#f0b42920;color:var(--am);padding:18px;border-radius:16px;margin-top:14px}
.stats{display:flex;border-top:1px solid var(--bd);margin-top:18px;padding-top:16px}
.stats div{flex:1;text-align:center;border-inline-start:1px solid var(--bd)}.stats div:first-child{border:0}
.stats b{display:block;font-size:30px;direction:ltr}.stats span{color:var(--mu);font-size:14px}
.row{display:flex;justify-content:space-between;padding:20px 4px;border-bottom:1px solid var(--bd)}
.row span{color:var(--mu)}.row b{font-size:20px;direction:ltr}
h2{font-size:24px;margin:26px 0 6px}.d{color:var(--mu);font-size:14px;margin-bottom:14px}
.g{display:grid;grid-template-columns:1fr 1fr;gap:14px}
.sc{border:1px solid var(--bd);border-radius:24px;padding:20px 10px;text-align:center;cursor:pointer;background:none;color:var(--tx);font-size:17px;font-weight:700}
.sc.a{border-color:var(--ac);background:#2fd3b01a}.sc svg{width:70px;height:70px;display:block;margin:0 auto 10px}
.f{display:flex;align-items:center;justify-content:space-between;gap:14px;padding:14px 0;color:var(--mu)}
.f input[type=range]{flex:1;max-width:220px;accent-color:var(--ac)}
.btns{display:flex;flex-wrap:wrap;gap:10px;margin-top:8px}
.btn{border:1px solid var(--bd);background:none;color:var(--tx);padding:12px 16px;border-radius:14px;font-size:15px;cursor:pointer}
.dp{display:grid;grid-template-columns:repeat(3,80px);grid-template-rows:repeat(3,66px);gap:10px;justify-content:center;direction:ltr;margin-bottom:14px}.dp button{border:1px solid var(--bd);background:#ffffff0d;color:var(--tx);border-radius:16px;font-size:26px;touch-action:none;user-select:none;-webkit-user-select:none}.dp .a{background:#2fd3b033;font-weight:800;font-size:22px}
footer{text-align:center;margin:30px 0 10px;font-size:18px;font-weight:700;direction:ltr}
footer span{display:inline-block;background:linear-gradient(90deg,#2fd3b0,#7c9cff,#ff6fd8,#2fd3b0);background-size:300% 100%;-webkit-background-clip:text;background-clip:text;color:transparent;animation:w 1.6s ease-in-out infinite,fl 4s linear infinite}
@keyframes w{0%,100%{transform:translateY(0)}50%{transform:translateY(-7px)}}@keyframes fl{to{background-position:300% 0}}
</style></head><body>
<div class="top"><div><h1 data-i="title"></h1><div class="sub" data-i="sub"></div></div>
<div class="tools"><button class="lg" id="lg" onclick="tg()">EN</button><span><span class="dot" id="od"></span><span id="ot"></span></span></div></div>
<div class="card">
<div class="hero"><div class="pp"><span id="ppm">0.0</span><small>ppm NH3</small></div><div class="big"><span id="pct">--</span><small>%</small></div></div>
<div class="chip" id="chip"><i id="cd1"></i><span id="ct"></span></div>
<div class="bar"><b id="bar"></b></div><div class="sc0"><span>0</span><span>25</span><span>50</span><span>75</span><span>100</span></div>
<div class="meta"><span data-i="l2"></span><span><span data-i="now"></span>: <span id="nv">0%</span></span></div>
<canvas id="cv"></canvas><div class="note" id="nt"></div>
<div class="stats"><div><b id="mn">0.0</b><span>ppm · <i data-i="mn" style="font-style:normal"></i></span></div><div><b id="av">0.0</b><span>ppm · <i data-i="av" style="font-style:normal"></i></span></div><div><b id="mx">0.0</b><span>ppm · <i data-i="mx" style="font-style:normal"></i></span></div></div></div>
<div class="row"><span data-i="rs"></span><b id="rs"></b></div>
<div class="row"><span data-i="r0"></span><b id="r0"></b></div>
<div class="row"><span data-i="up"></span><b id="up"></b></div>
<div class="row"><span data-i="cl"></span><b id="cl"></b></div>
<h2 data-i="scr"></h2><div class="d" data-i="scrd"></div>
<div class="g">
<button class="sc" onclick="setv('screen',0)"><svg viewBox="0 0 60 60" fill="none"><rect x="6" y="4" width="48" height="52" rx="12" stroke="#5b7388" stroke-width="2"/><path d="M18 28a12 12 0 0 1 24 0" stroke="#2fd3b0" stroke-width="4" stroke-linecap="round"/><path d="M14 46l9-6 8 4 8-8 8 5" stroke="#f0b429" stroke-width="3" stroke-linecap="round"/></svg><span data-i="s0"></span></button>
<button class="sc" onclick="setv('screen',1)"><svg viewBox="0 0 60 60" fill="none"><rect x="6" y="4" width="48" height="52" rx="12" stroke="#5b7388" stroke-width="2"/><text x="30" y="30" text-anchor="middle" font-size="20" font-weight="800" fill="#2fd3b0">68</text><path d="M16 38h5M24 38h5M32 38h5M40 38h5" stroke="#f0b429" stroke-width="4" stroke-linecap="round"/><path d="M15 47l10-3 8 2 10-4" stroke="#f0b429" stroke-width="2.5" stroke-linecap="round"/></svg><span data-i="s1"></span></button>
<button class="sc" onclick="setv('screen',2)"><svg viewBox="0 0 60 60" fill="none"><rect x="6" y="4" width="48" height="52" rx="12" stroke="#5b7388" stroke-width="2"/><rect x="14" y="12" width="14" height="10" rx="3" stroke="#8ea4b7" stroke-width="2"/><rect x="32" y="12" width="14" height="10" rx="3" stroke="#8ea4b7" stroke-width="2"/><rect x="14" y="26" width="14" height="10" rx="3" stroke="#8ea4b7" stroke-width="2"/><rect x="32" y="26" width="14" height="10" rx="3" stroke="#8ea4b7" stroke-width="2"/><path d="M14 47l10-4 8 3 14-5" stroke="#2fd3b0" stroke-width="3" stroke-linecap="round"/></svg><span data-i="s2"></span></button>
<button class="sc" onclick="setv('screen',3)"><svg viewBox="0 0 60 60" fill="none"><rect x="6" y="4" width="48" height="52" rx="12" stroke="#5b7388" stroke-width="2"/><circle cx="30" cy="30" r="6" fill="#2fd3b0"/><circle cx="30" cy="30" r="12" stroke="#2fd3b0" stroke-width="3"/><circle cx="30" cy="30" r="19" stroke="#2fd3b0" stroke-width="2" opacity=".5"/></svg><span data-i="s3"></span></button>
</div>
<h2 data-i="ctl"></h2>
<div class="card"><div class="f"><span data-i="bz"></span><input type="checkbox" id="bz" onchange="setv('mute',this.checked?0:1)" style="width:22px;height:22px;accent-color:#2fd3b0"></div>
<div class="f"><span data-i="br"></span><input type="range" id="br" min="5" max="100" onpointerdown="drag=1" onchange="drag=0;setv('bright',this.value)"></div>
<div class="f"><span data-i="al"></span><input type="range" id="at" min="10" max="100" onpointerdown="drag=1" onchange="drag=0;setv('alarm',this.value)"></div>
<div class="f"><span data-i="kn"></span><span style="display:flex;gap:8px"><input type="number" id="kp" placeholder="ppm" min="5" max="500" style="width:90px;background:none;border:1px solid var(--bd);color:var(--tx);border-radius:10px;padding:8px"><button class="btn" data-i="kb" onclick="kcal()"></button></span></div>
<div class="btns"><button class="btn" data-i="rc" onclick="if(confirm(t('rc')+'?'))api('cal')"></button><button class="btn" data-i="rz" onclick="api('reset')"></button><button class="btn" data-i="sk" onclick="api('skip')"></button></div></div>
<h2 data-i="upd"></h2>
<div class="card"><div class="f"><span id="ow"></span><span id="oc" style="direction:ltr"></span></div>
<div class="f"><input id="ws" list="wl" placeholder="Wi-Fi name" style="flex:1;background:none;border:1px solid var(--bd);color:var(--tx);border-radius:10px;padding:10px"><datalist id="wl"></datalist></div>
<div class="f"><input id="wp" type="password" placeholder="Wi-Fi password" style="flex:1;background:none;border:1px solid var(--bd);color:var(--tx);border-radius:10px;padding:10px"></div>
<div class="btns"><button class="btn" data-i="wsc" onclick="wscan()"></button><button class="btn" data-i="wsv" onclick="wsave()"></button></div>
<div class="f" style="margin-top:8px"><span id="om"></span></div>
<div class="btns"><button class="btn" data-i="uck" onclick="otaCheck()"></button><button class="btn" id="oi" data-i="uin" style="display:none;border-color:var(--ac)" onclick="otaInst()"></button></div>
<div class="f"><span data-i="uau"></span><input type="checkbox" id="oa" onchange="fetch('/api/otaauto?v='+(this.checked?1:0))" style="width:22px;height:22px;accent-color:#2fd3b0"></div></div>
<h2 data-i="gm"></h2><div class="d" data-i="gd"></div>
<div class="g"><button class="sc" onclick="game(1)" data-i="g1"></button><button class="sc" onclick="game(2)" data-i="g2"></button><button class="sc" onclick="game(3)" data-i="g3"></button><button class="sc" onclick="game(4)" data-i="g4"></button></div>
<div class="card" id="pad" style="display:none;margin-top:14px"><div class="dp">
<button style="grid-column:2;grid-row:1" onpointerdown="hold('u')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">&#9650;</button>
<button style="grid-column:1;grid-row:2" onpointerdown="hold('l')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">&#9664;</button>
<button style="grid-column:3;grid-row:2" onpointerdown="hold('r')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">&#9654;</button>
<button style="grid-column:2;grid-row:3" onpointerdown="hold('d')" onpointerup="rel()" onpointerleave="rel()" onpointercancel="rel()">&#9660;</button>
<button class="a" style="grid-column:3;grid-row:3" onpointerdown="hold('a')">A</button></div>
<div class="btns"><button class="btn" data-i="ex" onclick="game(0)"></button></div></div>
<footer id="ft"></footer><div id="ver" style="text-align:center;color:#5b7388;font-size:12px;margin-bottom:20px;direction:ltr"></div>
<script>
const $=i=>document.getElementById(i);let L='ar',drag=0,hist=[],D=null;
const T={ar:{title:'مراقب الغاز',sub:'حساس MQ-137 · ESP32',on:'متصل',off:'غير متصل',warm:'جاري تسخين الحساس',cal:'جاري المعايرة على هواء نظيف',s:'ث',l2:'آخر دقيقتين',now:'الآن',hi:'مرتفع',mid:'متوسط',lo:'منخفض',mn:'الأدنى',av:'المتوسط',mx:'الأعلى',rs:'مقاومة الحساس Rs',r0:'المقاومة النقية R0',up:'مدة التشغيل',cl:'الأجهزة المتصلة',scr:'واجهة الشاشة',scrd:'اختر شكل العرض على شاشة الجهاز',s0:'كلاسيكية',s1:'رقم كبير',s2:'بيانات',s3:'نبض',ctl:'التحكم',bz:'الصافرة',br:'السطوع',al:'حد الإنذار',rc:'إعادة المعايرة',rz:'تصفير الإحصائيات',sk:'تخطي التسخين',kn:'تركيز معروف (ppm)',kb:'معايرة',ov:'خارج المجال (أكثر من 500 ppm)',gm:'الألعاب',gd:'العب على شاشة الجهاز وتحكّم من هنا',g1:'الدودة والتفاحة',g2:'كسر الطوب',g3:'الطائر',g4:'التفادي',ex:'خروج من اللعبة',upd:'التحديث والواي فاي',wc:'متصل بشبكة',wn:'غير متصل بالإنترنت',wsv:'حفظ واتصال',wsc:'بحث عن الشبكات',uck:'تحقق من التحديث',uin:'تثبيت التحديث',uau:'تثبيت تلقائي',unw:'إصدار جديد متاح',uwk:'جاري التحديث... لا تفصل الكهرباء',ke:'ضع الحساس في غاز ثابت التركيز أولاً',nw:'يتم تسخين الحساس، أبقِه في هواء نظيف',lv:['هواء نظيف','منخفض','متوسط','مرتفع','خطر']},
en:{title:'Gas Monitor',sub:'MQ-137 sensor · ESP32',on:'Connected',off:'Offline',warm:'Warming up sensor',cal:'Calibrating in clean air',s:'s',l2:'Last 2 minutes',now:'Now',hi:'High',mid:'Medium',lo:'Low',mn:'Min',av:'Avg',mx:'Max',rs:'Sensor resistance Rs',r0:'Clean-air resistance R0',up:'Uptime',cl:'Connected devices',scr:'Screen layout',scrd:'Choose the display style on the device',s0:'Classic',s1:'Big number',s2:'Data',s3:'Pulse',ctl:'Controls',bz:'Buzzer',br:'Brightness',al:'Alarm level',rc:'Recalibrate',rz:'Reset stats',sk:'Skip warm-up',kn:'Known gas (ppm)',kb:'Calibrate',ov:'Over range (above 500 ppm)',gm:'Games',gd:'Play on the device screen and control from here',g1:'Snake',g2:'Breakout',g3:'Flappy',g4:'Dodge',ex:'Exit game',upd:'Updates & Wi-Fi',wc:'Connected to',wn:'No internet',wsv:'Save & connect',wsc:'Scan networks',uck:'Check for update',uin:'Install update',uau:'Auto install',unw:'New version available',uwk:'Updating... do not power off',ke:'Expose the sensor to a steady known gas first',nw:'Sensor is warming up, keep it in clean air',lv:['Clean air','Low','Moderate','High','Danger']}};
const t=k=>T[L][k];
function lang(){document.documentElement.lang=L;document.documentElement.dir=L=='ar'?'rtl':'ltr';document.querySelectorAll('[data-i]').forEach(e=>e.textContent=t(e.dataset.i));$('lg').textContent=L=='ar'?'EN':'ع';try{localStorage.setItem('l',L)}catch(e){}if(D)render(D)}
function tg(){L=L=='ar'?'en':'ar';lang()}
try{L=localStorage.getItem('l')||'ar'}catch(e){}
$('ft').innerHTML=[...'Made by @sars__88'].map((c,i)=>`<span style="animation-delay:${i*.08}s,0s">${c==' '?'&nbsp;':c}</span>`).join('');
async function api(p){try{await fetch('/api/'+p);poll()}catch(e){}}
async function setv(k,v){try{await fetch('/api/set?'+k+'='+v);poll()}catch(e){}}
async function kcal(){const v=$('kp').value;if(!v)return;try{const r=await fetch('/api/known?ppm='+v);const j=await r.json();alert(j.ok?'OK':t('ke'))}catch(e){}}
async function ota(){try{const r=await fetch('/api/ota');const o=await r.json();$('ow').textContent=o.wifi?t('wc')+': '+o.ssid:t('wn');$('oc').textContent='FW '+o.curv+' (build '+o.cur+')';$('om').textContent=o.has?t('unw')+': v'+o.ver:(o.msg||'');$('oi').style.display=o.has?'inline-block':'none';$('oa').checked=!!o.auto}catch(e){}}
async function otaCheck(){$('om').textContent='...';try{await fetch('/api/otacheck')}catch(e){}ota()}
async function otaInst(){if(!confirm(t('uin')+'?'))return;try{await fetch('/api/otainstall')}catch(e){}$('om').textContent=t('uwk')}
async function wscan(){try{const r=await fetch('/api/wifiscan');const j=await r.json();$('wl').innerHTML=j.n.map(s=>'<option value="'+s+'">').join('')}catch(e){}}
async function wsave(){try{await fetch('/api/wifiset?ssid='+encodeURIComponent($('ws').value)+'&pass='+encodeURIComponent($('wp').value))}catch(e){}setTimeout(ota,5000)}
let hT=null;
function key(k){fetch('/api/key?k='+k).catch(()=>{})}
function hold(k){key(k);clearInterval(hT);if(k!='a')hT=setInterval(()=>key(k),90)}
function rel(){clearInterval(hT)}
async function game(g){try{await fetch('/api/game?g='+g);$('pad').style.display=g?'block':'none'}catch(e){}}
addEventListener('keydown',e=>{const m={ArrowUp:'u',ArrowDown:'d',ArrowLeft:'l',ArrowRight:'r',' ':'a'}[e.key];if(m&&$('pad').style.display=='block'){e.preventDefault();key(m)}});
const mmss=s=>String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0');
function fmt(s){return[Math.floor(s/3600),Math.floor(s/60)%60,s%60].map(v=>String(v).padStart(2,'0')).join(':')}
function draw(){const c=$('cv'),r=devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;c.width=w*r;c.height=h*r;const x=c.getContext('2d');x.scale(r,r);
const L0=64,W0=w-L0-6,H0=h-10,bs=H0/3;
[['#2a3150','hi'],['#233a3d','mid'],['#14424b','lo']].forEach((b,i)=>{x.fillStyle=b[0];x.fillRect(L0,5+i*bs,W0,bs);x.fillStyle='#8ea4b7';x.font='13px system-ui';x.textAlign='right';x.fillText(t(b[1]),L0-8,5+i*bs+bs/2+4)});
x.setLineDash([6,5]);x.strokeStyle='#4b6275';x.lineWidth=1;[1,2].forEach(i=>{x.beginPath();x.moveTo(L0,5+i*bs);x.lineTo(L0+W0,5+i*bs);x.stroke()});x.setLineDash([]);
const n=hist.length,py=v=>5+H0-v/100*H0,px=i=>L0+W0-(n-1-i)*W0/119;
x.strokeStyle='#2fd3b0';x.lineWidth=3;x.lineJoin='round';x.beginPath();
if(n<2){x.moveTo(L0,py(0));x.lineTo(L0+W0,py(0))}else hist.forEach((v,i)=>i?x.lineTo(px(i),py(v)):x.moveTo(px(i),py(v)));x.stroke();
x.beginPath();x.arc(n?px(n-1):L0+W0,py(n?hist[n-1]:0),7,0,7);x.fillStyle='#2fd3b0';x.fill();x.strokeStyle='#2fd3b066';x.lineWidth=4;x.stroke()}
function render(d){D=d;const b=d.warm||d.cal,lv=d.pct<10?0:d.pct<30?1:d.pct<55?2:d.pct<80?3:4;
const col=b?'#2fd3b0':['#2fd3b0','#9ad94a','#f0b429','#ff8a3d','#e5484d'][lv];
$('pct').textContent=b?'--':Math.round(d.pct);$('ppm').textContent=d.ppm.toFixed(1);
$('ct').textContent=d.warm?t('warm')+' · '+mmss(d.left):d.cal?t('cal')+' · '+d.calLeft+t('s'):d.ppm>=499.9?t('ov'):t('lv')[lv];
$('chip').style.color=col;$('chip').style.background=col+'24';$('cd1').style.background=col;
$('bar').style.width=b?'0':d.pct+'%';$('bar').style.background=col;$('nv').textContent=b?'--':Math.round(d.pct)+'%';
const n=$('nt');n.style.display=b?'block':'none';n.textContent=d.warm?t('nw')+' ('+mmss(d.left)+')':t('cal')+' ('+d.calLeft+t('s')+')';
$('mn').textContent=d.min.toFixed(1);$('av').textContent=d.avg.toFixed(1);$('mx').textContent=d.max.toFixed(1);
$('rs').textContent=d.rs.toFixed(2)+' kΩ';$('r0').textContent=d.r0.toFixed(2)+' kΩ';$('up').textContent=fmt(d.up);$('cl').textContent=d.clients;$('ver').textContent='FW v'+d.ver;
document.querySelectorAll('.sc').forEach((e,i)=>e.classList.toggle('a',i==d.screen));
$('bz').checked=!d.mute;if(!drag){$('br').value=d.bright;$('at').value=d.alarm}
if(!b){hist.push(d.pct);if(hist.length>120)hist.shift()}draw()}
function online(o){$('od').className='dot'+(o?' on':'');$('ot').textContent=o?t('on'):t('off')}
async function poll(){try{const r=await fetch('/api/data');render(await r.json());online(1)}catch(e){online(0)}}
lang();poll();ota();setInterval(poll,1000);setInterval(ota,10000);addEventListener('resize',draw);
</script></body></html>
)rawliteral";


void hRoot() { server.send_P(200, "text/html", PAGE); }
void hData() {
  char b[480];
  uint32_t el = millis() - warmStart;
  uint32_t ce = millis() - calStart;
  int calLeft = calibrating ? (int)((ce >= CAL_MS ? 0 : CAL_MS - ce) / 1000) + 1 : 0;
  int left = warm ? (int)((el >= WARMUP_MS) ? 0 : (WARMUP_MS - el) / 1000) : 0;
  snprintf(b, sizeof b,
           "{\"ppm\":%.2f,\"pct\":%.1f,\"rs\":%.2f,\"r0\":%.2f,\"ratio\":%.3f,\"warm\":%s,\"left\":%d,"
           "\"ver\":\"" FW_VER "\",\"cal\":%s,\"calLeft\":%d,\"screen\":%d,\"mute\":%d,\"bright\":%d,\"alarm\":%d,\"min\":%.2f,\"max\":%.2f,\"avg\":%.2f,"
           "\"up\":%lu,\"clients\":%d,\"status\":\"%s\"}",
           ppm, pct, rs, r0, ratio, warm ? "true" : "false", left, calibrating ? "true" : "false", calLeft, screen, muted ? 1 : 0, brightness,
           alarmThr, cntPpm ? minPpm : 0.0f, maxPpm, cntPpm ? sumPpm / cntPpm : 0.0f, millis() / 1000,
           WiFi.softAPgetStationNum(), warm ? "WARMING UP" : statusText(pct));
  server.send(200, "application/json", b);
}
void hSet() {
  if (server.hasArg("screen")) {
    int s = server.arg("screen").toInt();
    if (s >= 0 && s < 4) { gameMode = 0; if (warm) screen = s; else changeScreen(s); }
  }
  if (server.hasArg("mute")) muted = server.arg("mute").toInt();
  if (server.hasArg("bright")) { brightness = constrain(server.arg("bright").toInt(), 5, 100); setBacklight(); }
  if (server.hasArg("alarm")) alarmThr = constrain(server.arg("alarm").toInt(), 10, 100);
  server.send(200, "application/json", "{\"ok\":1}");
}
void hSkip() { server.send(200, "application/json", "{\"ok\":1}"); if (warm) endWarmup(); }
void hCal() { server.send(200, "application/json", "{\"ok\":1}"); if (!warm) calibrate(); }
void hReset() { resetStats(); server.send(200, "application/json", "{\"ok\":1}"); }
void hKnown() {  // calibrate the curve to a known ppm (gas must be applied and stable)
  float c = server.arg("ppm").toFloat();
  float rho = powf(10.0f, lgS);
  bool ok = !warm && !calibrating && c >= PPM_MIN && c <= PPM_MAX && rho > 1.1f;
  if (ok) {
    sensS = constrain(rho / powf(c / PPM_REF, SLOPE_N), 1.2f, 30.0f);
    prefs.putFloat("S", sensS);
  }
  server.send(200, "application/json", ok ? "{\"ok\":1}" : "{\"ok\":0}");
}
void hGame() {  // g=1..4 start a game, g=0 exit
  int g = server.arg("g").toInt();
  if (g == 0) { if (gameMode) { gameMode = 0; if (!warm) changeScreen(screen); } }
  else if (g >= 1 && g <= 4 && !warm) { gameMode = g; gameStart(); }
  server.send(200, "application/json", "{\"ok\":1}");
}
void hKey() {  // k = u d l r a
  String k = server.arg("k");
  if (gameMode && k.length()) {
    char c = k[0];
    if (c == 'l') { kMove--; kKey = 'l'; }
    else if (c == 'r') { kMove++; kKey = 'r'; }
    else if (c == 'u' || c == 'd') kKey = c;
    else if (c == 'a') kA = true;
  }
  server.send(200, "application/json", "{\"ok\":1}");
}
String clean(String x) { x.replace("\\", "/"); x.replace("\"", "'"); return x; }
void hOta() {
  char b[360];
  snprintf(b, sizeof b, "{\"cur\":%d,\"curv\":\"%s\",\"wifi\":%s,\"ssid\":\"%s\",\"has\":%s,\"ver\":\"%s\",\"msg\":\"%s\",\"auto\":%d}",
           FW_BUILD, FW_VER, WiFi.status() == WL_CONNECTED ? "true" : "false", clean(WiFi.SSID()).c_str(),
           otaHas ? "true" : "false", clean(otaVer).c_str(), clean(otaMsg).c_str(), otaAuto ? 1 : 0);
  server.send(200, "application/json", b);
}
void hOtaCheck() { otaCheck(); hOta(); }
void hOtaInstall() { otaGo = otaHas; server.send(200, "application/json", "{\"ok\":1}"); }
void hOtaAuto() { otaAuto = server.arg("v").toInt(); prefs.putBool("oauto", otaAuto); server.send(200, "application/json", "{\"ok\":1}"); }
void hWifiSet() {
  wSsid = server.arg("ssid"); wPass = server.arg("pass");
  prefs.putString("wssid", wSsid); prefs.putString("wpass", wPass);
  server.send(200, "application/json", "{\"ok\":1}");
  if (wSsid.length()) { WiFi.disconnect(false); WiFi.begin(wSsid.c_str(), wPass.c_str()); otaLast = 0; }
}
void hWifiScan() {
  int n = WiFi.scanNetworks();
  String j = "{\"n\":[";
  for (int i = 0; i < n && i < 15; i++) { if (i) j += ","; j += "\"" + clean(WiFi.SSID(i)) + "\""; }
  WiFi.scanDelete();
  server.send(200, "application/json", j + "]}");
}
void hRedirect() {  // captive portal: any unknown URL opens the dashboard
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}


void startWeb() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  dns.start(53, "*", apIP);
  server.on("/", hRoot);
  server.on("/api/data", hData);
  server.on("/api/set", hSet);
  server.on("/api/skip", hSkip);
  server.on("/api/cal", hCal);
  server.on("/api/reset", hReset);
  server.on("/api/known", hKnown);
  server.on("/api/game", hGame);
  server.on("/api/key", hKey);
  server.on("/api/ota", hOta);
  server.on("/api/otacheck", hOtaCheck);
  server.on("/api/otainstall", hOtaInstall);
  server.on("/api/otaauto", hOtaAuto);
  server.on("/api/wifiset", hWifiSet);
  server.on("/api/wifiscan", hWifiScan);
  server.onNotFound(hRedirect);
  server.begin();
}


// ---------------- Button ----------------
void handleButton() {
  static bool last = true;
  static uint32_t t = 0;
  bool now = digitalRead(BTN_PIN);
  if (last && !now && millis() - t > 250) {
    t = millis();
    if (!warm) { gameMode = 0; changeScreen(screen + 1); beep(1800, 40); }
  }
  last = now;
}


// ---------------- Main ----------------
void setup() {
  Serial.begin(115200);
  pinMode(BTN_PIN, INPUT_PULLUP);
  prefs.begin("gas", false);
  sensS = prefs.getFloat("S", SENS_S);
  pinMode(BUZZER_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);


  initBacklight();
  SPI.begin(TFT_SCK, -1, TFT_MOSI, TFT_CS);
  // SPI mode is saved in flash. Beeps at start-up = mode number (1 = MODE2, 2 = MODE3, 3 = MODE0, 4 = MODE1).
  // If the screen stays black, press BOOT during the colour test: it switches to the next mode and restarts.
  static const uint8_t spiModes[4] = {SPI_MODE2, SPI_MODE3, SPI_MODE0, SPI_MODE1};
  int mi = prefs.getInt("spim", 0) % 4;
  delay(150);   // let the display power up
  tft.init(240, 240, spiModes[mi]);
  tft.setSPISpeed(TFT_SPI_HZ);
  tft.setRotation(0);
  tft.fillScreen(BLACK);
  Serial.printf("TFT: SPI mode index %d, %d Hz\n", mi, TFT_SPI_HZ);
#if TFT_TEST
  for (int i = 0; i <= mi; i++) { tone(BUZZER_PIN, 1500); delay(80); noTone(BUZZER_PIN); delay(150); }
  static const uint16_t testCol[3] = {0xF800, 0x07E0, 0x001F};
  for (int c = 0; c < 3; c++) {
    tft.fillScreen(testCol[c]);
    uint32_t t1 = millis();
    while (millis() - t1 < 1300) {
      if (digitalRead(BTN_PIN) == LOW) {          // BOOT pressed: try the next SPI mode
        prefs.putInt("spim", (mi + 1) % 4);
        tone(BUZZER_PIN, 800); delay(300); noTone(BUZZER_PIN);
        ESP.restart();
      }
      delay(10);
    }
  }
  tft.fillScreen(BLACK);
#endif


  startWeb();
  wifiBegin();
  introTone();
  helloFade();


  warmStart = millis();
  warmStatic();
}


void loop() {
  dns.processNextRequest();
  server.handleClient();
  handleButton();
  handleBeep();
  updateSfx();
  otaLoop();
  uint32_t now = millis();


  if (now - lastSample >= 100) { lastSample = now; sampleSensor(); }


  if (warm) {
    warmUpdate();
    if (now - warmStart >= WARMUP_MS) endWarmup();
    return;
  }


  // continuous alarm: sounds as long as the gas level stays above the threshold
  bool want = !muted && (alarmOn ? pct >= alarmThr - 3 : pct >= alarmThr);
  if (want && !alarmOn) { alarmOn = true; beepEnd = 0; tone(BUZZER_PIN, 2600); }
  else if (!want && alarmOn) { alarmOn = false; noTone(BUZZER_PIN); }


  // trend history (1 sample / second)
  if (!calibrating && now - lastHist >= 1000) { lastHist = now; histPush(); }


  if (gameMode) { gameLoop(now); return; }  // a game is running: skip the gauges


  // display refresh
  if (now - lastUi >= 33) {
    lastUi = now;
    smoothValues();
    switch (screen) {
      case 0: s0Update(); break;
      case 1: s1Update(); break;
      case 2: s2Update(); break;
      default: s3Update(); break;
    }
    histDirty = false;
  }
}