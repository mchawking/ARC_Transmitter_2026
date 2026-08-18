// RCRA Display v1.4.8 — FRAM save blink + no prefix arrows
// Changelog:
// - Removed prefix arrows from status header (now shows only ARMED/UNARMED in color)
// - Green FRAM indicator now blinks with a black circle for a few seconds after a successful save
// - Keeps v1.4.7 features: 3-option calibration picker with live Abs & Δ-from-zero (~20 Hz), SAVED splash, FRAM v3 persistence

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include "crsf.h"
#include <Adafruit_ST7789.h>
#include <Adafruit_FRAM_I2C.h>
#include <math.h>

// --- TFT pins ---
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_CS   9
#define TFT_DC   10
#define TFT_RST  15

// --- CRSF UART pins (must match hardware connections) ---
#define CRSF_RX_PIN  21
#define CRSF_TX_PIN  20

// --- Inputs ---
#define BUTTON_BACK   18  // Back — 5s zero-all (MAIN) / Exit menus
#define BUTTON_SELECT 19  // Select — 2s opens Calibration Menu / selects item / confirm
#define BUTTON_UP     13  // Up — navigate up
#define BUTTON_DOWN   14  // Down — navigate down
#define TOGGLE_PIN    16  // Armed / Unarmed

// --- I2C wiring ---
#define I2C_SDA 6
#define I2C_SCL 7
const uint8_t FRAM_ADDR = 0x50;

// --- I2C devices ---
const uint8_t MUX_ADDR  = 0x70;   // TCA9548A
const uint8_t AS5600_ADDR = 0x36; // AS5600 encoders
const uint8_t AS5600_RAW_ANGLE_H = 0x0C;

Adafruit_ST7789 tft(TFT_CS, TFT_DC, TFT_RST);
Adafruit_FRAM_I2C fram;
// CRSF transmitter instance (created in setup)
CrsfSerial* crsf = nullptr;

// --- Layout for 320x170, rotation=1 ---
const int16_t PAD_X    = 8;
const int16_t PAD_Y    = 6;
const int16_t LINE_H   = 26;

// Live readout strip (shown in CAL_PICK)
const int16_t CAL_LIVE_H = 22;     // height of the live readout box above footer
float    calLiveAbs  = NAN;        // absolute scaled angle (deg)
float    calLiveDist = NAN;        // distance-from-zero (deg)
uint32_t calLiveLastMs = 0;        // last refresh time

// Right-column/values on main screen
const int16_t IND_SIZE = 20;            // FRAM indicator square
const int16_t ICON_BOX = 20;            // arrow icon cell
const int16_t ICON_GAP = 12;            // spacing between icon cells
const int16_t VALUE_X  = 110;           // left edge of values (shifted left)
const int16_t VALUE_RIGHT_MARGIN = 4; // Reduced margin for longer bars

int16_t rightColX_LeftEdge() { return tft.width() - PAD_X - IND_SIZE; }
int16_t valueRightLimit()    { return rightColX_LeftEdge() - VALUE_RIGHT_MARGIN; }

// --- Buttons state ---
bool lastBack=HIGH, lastSelect=HIGH, lastUp=HIGH, lastDown=HIGH, lastToggle=HIGH;

// --- FRAM status + save-blink ---
bool framOK = false;
bool framProbeOK = false;
uint32_t lastFramSaveMs = 0;
const uint16_t FRAM_SAVED_FLASH_MS = 2500;  // blink indicator for ~2.5s after a save
const uint32_t BOOT_SELECT_TIMEOUT_MS = 5000; // auto-continue if Select is not pressed

// UI screens
enum UIScreen { UI_MAIN, UI_STATS };
UIScreen uiScreen = UI_MAIN;

// Stats screen state (kept but minimal)
uint8_t statsIndex = 0;
bool    needsMainRedraw  = false;
bool    needsStatsRedraw = false;

// -------------------- Angle unwrap + zero tracking --------------------
struct AngleTrack {
  float lastDeg = NAN;     // last raw 0..360 (encoder space)
  long  turns   = 0;       // wrap counter (encoder space)
  float zeroCont = 0;      // baseline continuous angle (scaled joint units)
  float minDist = 0;       // min since zero (scaled)
  float maxDist = 0;       // max since zero (scaled)
  bool  inited  = false;   // first-sample flag
  float lastContScaled = 0;// latest continuous joint angle (scaled)
};
AngleTrack ang[4];

// -------- Fixed pulley compensation and per-channel angle direction --------
// The shoulder encoder is coupled through a fixed 4:1 pulley; the other
// encoders measure their joint directly. These are not selectable gear ratios.
const float CH_SCALE_BASE[4] = { 1.0f/4.0f, 1.0f, 1.0f, 1.0f };
int8_t CH_SIGN[4] = { +1, +1, +1, +1 }; // stored in FRAM (+1 or -1)
inline float scaledWithSign(uint8_t ch, float contEncoder) {
  return contEncoder * CH_SCALE_BASE[ch] * (float)CH_SIGN[ch];
}

// Soft limits (relative to zero, scaled joint degrees)
float softMin[4] = {0,0,0,0};
float softMax[4] = {0,0,0,0};

// --- AS5600 / MUX helpers ---
bool muxSelect(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  return (Wire.endTransmission() == 0);
}
bool as5600ReadRaw(uint16_t &raw) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAW_ANGLE_H);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom(AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  raw = ((uint16_t)hi << 8) | lo;
  raw &= 0x0FFF; // 12-bit
  return true;
}
bool readAngleDeg(uint8_t channel, float &degOut) {
  if (!muxSelect(channel)) { degOut = NAN; return false; }
  uint16_t raw;
  if (!as5600ReadRaw(raw)) { degOut = NAN; return false; }
  degOut = (raw * 360.0f) / 4096.0f;
  return true;
}

// --- Unwrap (encoder space) ---
float unwrapAngle(uint8_t ch, float curDeg) {
  AngleTrack &s = ang[ch];
  if (!s.inited) {
    s.lastDeg = curDeg; s.turns = 0; s.inited = true;
    return curDeg;
  }
  float delta = curDeg - s.lastDeg;
  if (delta < -180.0f)      s.turns += 1;
  else if (delta > +180.0f) s.turns -= 1;
  s.lastDeg = curDeg;
  return s.turns * 360.0f + curDeg;
}
void seedUnwrapFromScaled(uint8_t ch, float raw0to360, float targetContScaled) {
  float xTargetEncoder = targetContScaled / (CH_SCALE_BASE[ch] * (float)CH_SIGN[ch]);
  float nFloat = (xTargetEncoder - raw0to360) / 360.0f;
  long n = lroundf(nFloat);
  ang[ch].turns = n;
  ang[ch].lastDeg = raw0to360;
  ang[ch].inited = true;
}

// -------------------- Main UI helpers --------------------
const int16_t SENSOR_BLOCK_H = (4 * LINE_H) + 8;
int16_t SENSOR_BLOCK_Y = 0;

void clearRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color=ST77XX_BLACK){ tft.fillRect(x, y, w, h, color); }
void clearLineSpan(int16_t y, int16_t x0=0, int16_t x1=-1) { if (x1 < 0) x1 = tft.width() - 1; clearRect(x0, y - 2, x1 - x0 + 1, LINE_H); }
void drawFramIndicator(bool ok) {
  int16_t x = rightColX_LeftEdge(), y = PAD_Y;
  uint16_t color = ok ? ST77XX_GREEN : ST77XX_RED;
  // Draw a circular indicator instead of a square
  int16_t cx = x + IND_SIZE/2;
  int16_t cy = y + IND_SIZE/2;
  uint16_t rOut = IND_SIZE/2;
  // Outer filled circle (green when OK, red when not)
  tft.fillCircle(cx, cy, rOut, color);
  // Thin black outline for contrast
  tft.drawCircle(cx, cy, rOut, ST77XX_BLACK);

  // Blink a small black circle overlay for a short window after save
  if (ok) {
    uint32_t dt = millis() - lastFramSaveMs;
    if (dt < FRAM_SAVED_FLASH_MS) {
      // ~2 Hz blink (250ms on / 250ms off)
      if ((dt / 250) % 2 == 0) {
        uint16_t rIn = IND_SIZE/4;
        tft.fillCircle(cx, cy, rIn, ST77XX_BLACK);
      }
    }
  }
}
// Draw a small CRSF status indicator immediately to the left of the FRAM
// indicator. Uses crsf->getLastChannelUpdate() to determine activity.
void drawCrsfIndicator() {
  int16_t framX = rightColX_LeftEdge();
  int16_t x = framX - IND_SIZE - 8; // place to the left of the FRAM indicator
  int16_t y = PAD_Y;
  int16_t cx = x + IND_SIZE/2;
  int16_t cy = y + IND_SIZE/2;
  // Default: assume OK until proven otherwise
  uint16_t outerFill = ST77XX_BLUE;
  uint16_t outerOutline = ST77XX_BLACK;
  bool blinkInnerBlack = false;

  if (crsf == nullptr || !crsf->isInitialized()) {
    // CRSF failed to initialize — show red
    outerFill = ST77XX_RED;
    outerOutline = ST77XX_BLACK;
  } else {
    uint32_t lastSent = crsf->getLastChannelUpdate();
    uint32_t dtSent = (lastSent == 0) ? 0xFFFFFFFFu : (millis() - lastSent);

    // If it's been more than 5 seconds since last send, show yellow
    if (dtSent > 5000u) {
      outerFill = ST77XX_YELLOW;
      outerOutline = ST77XX_BLACK;
    } else {
      // Within 5s -> blue. If very recent (<600ms), blink inner black circle.
      outerFill = ST77XX_BLUE;
      outerOutline = ST77XX_BLACK;
      if (lastSent != 0 && dtSent < 600u) blinkInnerBlack = true;
    }
  }

  // Draw outer filled circle then outline
  tft.fillCircle(cx, cy, IND_SIZE/2, outerFill);
  tft.drawCircle(cx, cy, IND_SIZE/2, outerOutline);

  // Blink a small black filled circle inside when data was recently sent
  if (blinkInnerBlack) {
    if ((millis() / 250) % 2 == 0) {
      int16_t rIn = IND_SIZE/4;
      tft.fillCircle(cx, cy, rIn, ST77XX_BLACK);
    }
  }
}

// Convert HSV (h:0-360, s:0-1, v:0-1) to 16-bit RGB565 color for the TFT
static inline uint16_t hsvToRGB565(float h, float s, float v) {
  // normalize
  while (h < 0) h += 360.0f;
  while (h >= 360.0f) h -= 360.0f;
  float c = v * s;
  float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
  float m = v - c;
  float r1=0, g1=0, b1=0;
  if (h < 60) { r1 = c; g1 = x; b1 = 0; }
  else if (h < 120) { r1 = x; g1 = c; b1 = 0; }
  else if (h < 180) { r1 = 0; g1 = c; b1 = x; }
  else if (h < 240) { r1 = 0; g1 = x; b1 = c; }
  else if (h < 300) { r1 = x; g1 = 0; b1 = c; }
  else { r1 = c; g1 = 0; b1 = x; }
  uint8_t r = (uint8_t)roundf((r1 + m) * 255.0f);
  uint8_t g = (uint8_t)roundf((g1 + m) * 255.0f);
  uint8_t b = (uint8_t)roundf((b1 + m) * 255.0f);
  // RGB888 -> RGB565
  uint16_t rgb565 = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
  return rgb565;
}

// Boot / splash screen shown at startup
// Shows product name, version info, hardware details, initialization status,
// and an animated progress bar before transitioning to the main UI.
void drawBootScreen(bool framOk, bool framRwOk, bool crsfOk) {
  tft.fillScreen(ST77XX_BLACK);
  
  // Main title (use smaller font to prevent edge overlap)
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  const char* title = "ARC Transmitter";
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds((char*)title, 0, 0, &bx, &by, &bw, &bh);
  int16_t titleX = (tft.width() - bw) / 2;
  int16_t titleYTop = 8;
  int16_t cursorY = titleYTop - by;
  if (cursorY < 0) cursorY = 0;
  tft.setCursor(titleX, cursorY);
  tft.setTextColor(ST77XX_CYAN);
  tft.print(title);
  
  // Version and build info on a single line just below the title
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  int16_t infoY = cursorY + bh + 8;

  // Primary/alternate strings for width fallback
  const char* verPrimary = "Firmware v2.1";
  const char* verAlt     = "FW v2.1";
  const char* bldPrimary = "Build: Aug 13, 2026";
  const char* bldAlt1    = "Build Aug 13, 2026"; // drop colon
  const char* bldAlt2    = "Aug 13, 2026";       // shortest

  // Measure candidates to decide a fit using the current font
  int16_t vbx, vby, sbx, sby, dbx, dby; uint16_t vbw, vbh, sbw, sbh, dbw, dbh;
  const char* sep = "  |  ";
  tft.getTextBounds((char*)verPrimary, 0, 0, &vbx, &vby, &vbw, &vbh);
  tft.getTextBounds((char*)sep,        0, 0, &sbx, &sby, &sbw, &sbh);
  tft.getTextBounds((char*)bldPrimary, 0, 0, &dbx, &dby, &dbw, &dbh);
  uint16_t avail = (uint16_t)(tft.width() - 2*PAD_X);

  const char* verUse = verPrimary;
  const char* bldUse = bldPrimary;
  uint16_t lineW = vbw + sbw + dbw;

  bool drewWithBitmap = false;
  if (lineW > avail) {
    // Try abbreviated firmware label
    int16_t v2bx, v2by; uint16_t v2bw, v2bh; tft.getTextBounds((char*)verAlt, 0, 0, &v2bx, &v2by, &v2bw, &v2bh);
    if ((uint16_t)(v2bw + sbw + dbw) <= avail) {
      verUse = verAlt; vbw = v2bw; vbh = v2bh; vby = v2by;
      lineW = vbw + sbw + dbw;
    } else {
      // Try shorter build text variants with abbreviated firmware
      int16_t d1bx, d1by; uint16_t d1bw, d1bh; tft.getTextBounds((char*)bldAlt1, 0, 0, &d1bx, &d1by, &d1bw, &d1bh);
      if ((uint16_t)(v2bw + sbw + d1bw) <= avail) {
        verUse = verAlt; bldUse = bldAlt1; vbw = v2bw; vbh = v2bh; vby = v2by; dbw = d1bw; dbh = d1bh; dby = d1by;
        lineW = vbw + sbw + dbw;
      } else {
        int16_t d2bx, d2by; uint16_t d2bw, d2bh; tft.getTextBounds((char*)bldAlt2, 0, 0, &d2bx, &d2by, &d2bw, &d2bh);
        if ((uint16_t)(v2bw + sbw + d2bw) <= avail) {
          verUse = verAlt; bldUse = bldAlt2; vbw = v2bw; vbh = v2bh; vby = v2by; dbw = d2bw; dbh = d2bh; dby = d2by;
          lineW = vbw + sbw + dbw;
        } else {
          // Last resort: draw with default bitmap font to guarantee fit
          tft.setFont(NULL);
          tft.setTextSize(1);
          int16_t by0; uint16_t bw0, bh0;
          // Re-measure with default font
          tft.getTextBounds((char*)verAlt, 0, 0, &vbx, &by0, &vbw, &bh0);
          tft.getTextBounds((char*)sep,    0, 0, &sbx, &by0, &sbw, &bh0);
          tft.getTextBounds((char*)bldAlt2,0, 0, &dbx, &by0, &dbw, &bh0);
          verUse = verAlt; bldUse = bldAlt2;
          // Draw in single color to keep it simple in bitmap font
          tft.setTextColor(ST77XX_YELLOW);
          tft.setCursor(PAD_X, infoY);
          tft.print(verUse); tft.print(sep); tft.print(bldUse);
          // Restore GFX font for subsequent sections
          tft.setFont(&FreeSans9pt7b);
          tft.setTextSize(1);
          // Advance infoY based on a conservative line height
          infoY += (int16_t)bh0 + 4;
          drewWithBitmap = true;
        }
      }
    }
  }

  if (!drewWithBitmap) {
    // Draw multi-colored single line using GFX font
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(PAD_X, infoY - vby);
    tft.print(verUse);
    // Separator in white
    int16_t xSep = PAD_X + (int16_t)vbw;
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(xSep, infoY - sby);
    tft.print(sep);
    // Build text in green
    int16_t xBld = xSep + (int16_t)sbw;
    tft.setTextColor(ST77XX_GREEN);
    tft.setCursor(xBld, infoY - dby);
    tft.print(bldUse);
    // Advance infoY based on tallest glyph box on the line
    int16_t maxBh = (int16_t)max((int)max(vbh, sbh), (int)dbh);
    infoY += maxBh + 6;
  }

  // Progress bar box
  int16_t pbW = tft.width() - 2*PAD_X;
  int16_t pbH = 30;
  int16_t pbX = PAD_X;
  int16_t pbY = infoY + 6; // tightened spacing under the single info line
  
  tft.drawRoundRect(pbX, pbY, pbW, pbH, 4, ST77XX_WHITE);

  // Animate progress for 5 seconds
  uint32_t start = millis();
  const uint32_t totalMs = 5000;
  
  while (millis() - start < totalMs) {
    uint32_t now = millis();
    float frac = (float)(now - start) / (float)totalMs;
    if (frac < 0) frac = 0; if (frac > 1) frac = 1;
    int16_t fillW = (int16_t)roundf((pbW - 4) * frac);
    
    // Hue from 0 (red) to 240 (blue)
    float hue = 240.0f * frac;
    uint16_t col = hsvToRGB565(hue, 1.0f, 1.0f);
    
    // Fill rounded progress bar
    if (fillW > 0) {
      tft.fillRoundRect(pbX + 2, pbY + 2, fillW, pbH - 4, 3, col);
    }
    
    delay(50);
  }

  // System status on a single centered line: "Sensors: OK  FRAM: OK  CRSF: OK"
  int16_t statusY = pbY + pbH + 12;
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);

  // Evaluate statuses
  bool allSensorsOk = true;
  for (uint8_t ch = 0; ch < 4; ch++) {
    float deg;
    if (!readAngleDeg(ch, deg)) { allSensorsOk = false; break; }
  }
  const char* sensorsLabel = "Sensors: ";
  const char* sensorsState = allSensorsOk ? "OK" : "FAIL";
  const char* framLabel    = "FRAM RW: ";
  const char* framState    = framRwOk ? "PASS" : "FAIL";
  const char* crsfLabel    = "CRSF: ";
  const char* crsfState    = crsfOk ? "OK" : "FAIL";
  const char* sepStatus    = "  "; // two spaces between groups

  // Measure each segment to compute total width
  int16_t tbx, tby; uint16_t tbw, tbh;
  int16_t slBy, ssBy, flBy, fsBy, clBy, csBy, sepBy; // store by for baseline alignment
  uint16_t slW, ssW, flW, fsW, clW, csW, sepW;

  tft.getTextBounds((char*)sensorsLabel, 0, 0, &tbx, &slBy, &slW, &tbh);
  tft.getTextBounds((char*)sensorsState, 0, 0, &tbx, &ssBy, &ssW, &tbh);
  tft.getTextBounds((char*)sepStatus,    0, 0, &tbx, &sepBy, &sepW, &tbh);
  tft.getTextBounds((char*)framLabel,    0, 0, &tbx, &flBy, &flW, &tbh);
  tft.getTextBounds((char*)framState,    0, 0, &tbx, &fsBy, &fsW, &tbh);
  tft.getTextBounds((char*)crsfLabel,    0, 0, &tbx, &clBy, &clW, &tbh);
  tft.getTextBounds((char*)crsfState,    0, 0, &tbx, &csBy, &csW, &tbh);

  uint16_t totalW = slW + ssW + sepW + flW + fsW + sepW + clW + csW;
  int16_t x = (tft.width() - (int16_t)totalW) / 2;
  if (x < PAD_X) x = PAD_X;

  // Draw the composite line with colored states
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x, statusY - slBy); tft.print(sensorsLabel); x += (int16_t)slW;
  tft.setTextColor(allSensorsOk ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(x, statusY - ssBy); tft.print(sensorsState); x += (int16_t)ssW;

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x, statusY - sepBy); tft.print(sepStatus); x += (int16_t)sepW;

  tft.setCursor(x, statusY - flBy); tft.print(framLabel); x += (int16_t)flW;
  tft.setTextColor(framRwOk ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(x, statusY - fsBy); tft.print(framState); x += (int16_t)fsW;

  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x, statusY - sepBy); tft.print(sepStatus); x += (int16_t)sepW;

  tft.setCursor(x, statusY - clBy); tft.print(crsfLabel); x += (int16_t)clW;
  tft.setTextColor(crsfOk ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(x, statusY - csBy); tft.print(crsfState);

  // Prompt to proceed
  const char* prompt = "Press Select or wait...";
  tft.setTextColor(ST77XX_YELLOW);
  int16_t pBx, pBy; uint16_t pBw, pBh; tft.getTextBounds((char*)prompt, 0, 0, &pBx, &pBy, &pBw, &pBh);
  int16_t promptY = tft.height() - (int16_t)pBh - 6;
  bool prevSel = (digitalRead(BUTTON_SELECT)==LOW);
  uint32_t lastBlink = millis();
  uint32_t waitStart = millis();
  bool showPrompt = true;
  
  while (true) {
    // Blink the prompt
    if (millis() - lastBlink > 400) {
      lastBlink = millis();
      showPrompt = !showPrompt;
      // Clear prompt area
      tft.fillRect(0, promptY + pBy - 2, tft.width(), pBh + 6, ST77XX_BLACK);
      if (showPrompt) {
        tft.setTextColor(ST77XX_YELLOW);
        tft.setCursor((tft.width() - (int16_t)pBw) / 2, promptY - pBy);
        tft.print(prompt);
      }
    }
    // Wait for a fresh Select press (falling edge)
    bool curSel = (digitalRead(BUTTON_SELECT)==LOW);
    if (curSel && !prevSel) { break; }
    if (millis() - waitStart >= BOOT_SELECT_TIMEOUT_MS) { break; }
    prevSel = curSel;
    delay(20);
  }

  // Clear the whole screen so no splash remnants remain when main UI draws
  tft.fillScreen(ST77XX_BLACK);
}
void drawStatus(bool armedLow) {
  clearLineSpan(PAD_Y, 0, rightColX_LeftEdge() - 2);
  // Use a bold FreeSans GFX font for the status word to improve legibility
  const char* status = armedLow ? "ARMED - DANGER!" : "UNARMED";
  tft.setFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
  tft.setTextColor(armedLow ? ST77XX_RED : ST77XX_GREEN);
  // Compute cursor Y so the font bounding-box top sits at PAD_Y+1
  // getTextBounds returns 'by' as the top offset relative to the cursor baseline,
  // so cursorY = desiredTop - by will position the bounding box correctly.
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds((char*)status, 0, 0, &bx, &by, &bw, &bh);
  int16_t desiredTop = PAD_Y + 1; // leave a 1px margin from the very top
  int16_t cursorY = desiredTop - by;
  if (cursorY < 0) cursorY = 0;
  tft.setCursor(PAD_X, cursorY);
  tft.print(status);
  // Restore to default (bitmap) font for the rest of the UI
  tft.setFont(NULL);
  drawFramIndicator(framOK);
  // Draw CRSF indicator next to FRAM indicator
  drawCrsfIndicator();
}
int16_t iconY(uint8_t idx) { int16_t startY = PAD_Y + IND_SIZE + ICON_GAP; return startY + idx * (ICON_BOX + ICON_GAP); }
void drawArrowTriangle(int16_t xCell, int16_t yCell, uint8_t dir, bool filled, uint16_t color) { const int16_t x = xCell, y = yCell, w = ICON_BOX, h = ICON_BOX, m = 2; int16_t x0, y0, x1, y1, x2, y2; switch (dir) { case 0: x0=x+m; y0=y+h/2; x1=x+w-m; y1=y+m; x2=x+w-m; y2=y+h-m; break; case 1: x0=x+w-m; y0=y+h/2; x1=x+m; y1=y+m; x2=x+m; y2=y+h-m; break; case 2: x0=x+w/2; y0=y+m; x1=x+m; y1=y+h-m; x2=x+w-m; y2=y+h-m; break; default:x0=x+w/2; y0=y+h-m; x1=x+m; y1=y+m; x2=x+w-m; y2=y+m; break; } if (filled) tft.fillTriangle(x0,y0,x1,y1,x2,y2,color); else tft.drawTriangle(x0,y0,x1,y1,x2,y2,color); }
void drawArrowIcon(uint8_t idx, bool pressed) { uint8_t dir; uint16_t color; switch (idx) { case 0: dir=0;color=ST77XX_RED;break; case 1: dir=1;color=ST77XX_YELLOW;break; case 2: dir=2;color=ST77XX_GREEN;break; default:dir=3;color=ST77XX_BLUE;break; } int16_t x = rightColX_LeftEdge(), y = iconY(idx); tft.fillRect(x, y, ICON_BOX, ICON_BOX, ST77XX_BLACK); drawArrowTriangle(x, y, dir, pressed, color); }
void redrawArrowIconsNow() { bool bBack=(digitalRead(BUTTON_BACK)==LOW); bool bSelect=(digitalRead(BUTTON_SELECT)==LOW); bool bUp=(digitalRead(BUTTON_UP)==LOW); bool bDown=(digitalRead(BUTTON_DOWN)==LOW); /* drawArrowIcon(0, bBack); */ drawArrowIcon(1, bSelect); drawArrowIcon(2, bUp); drawArrowIcon(3, bDown); }
void drawValueRightAligned(int16_t y, const char* text) { int16_t xRight=valueRightLimit(); if(xRight>VALUE_X) clearRect(VALUE_X,y-2,xRight-VALUE_X,LINE_H); int16_t x1,y1; uint16_t w,h; tft.getTextBounds((char*)text,0,0,&x1,&y1,&w,&h); int16_t x=xRight-(int16_t)w; if(x<VALUE_X) x=VALUE_X; tft.setCursor(x,y); tft.setTextColor(ST77XX_WHITE); tft.setTextSize(2); tft.print(text);} 
// small clamp helper
static inline float clampf(float v, float a, float b){ if(v<a) return a; if(v>b) return b; return v; }

// Draw a horizontal bar representing the joint distance relative to Min/Max and center.
// - idx: channel index 0..3
// - dist: distance-from-zero (scaled joint degrees)
void drawAngleBar(uint8_t idx, float dist){ if(idx>3) return; int16_t yBase = SENSOR_BLOCK_Y + idx * LINE_H; // baseline y for the line
  // area for the bar (aligned with previous numeric area)
  int16_t x = VALUE_X;
  int16_t xRight = valueRightLimit();
  if(xRight <= x) return;
  int16_t w = xRight - x;
  int16_t h = LINE_H - 8; if(h < 8) h = 8;
  int16_t barY = yBase + 4;

  // Determine display min/max from softMin/softMax with fallback
  float sMin = softMin[idx];
  float sMax = softMax[idx];
  if (!isfinite(sMin) || !isfinite(sMax) || fabsf(sMax - sMin) < 0.1f) {
    // fallback: symmetric range around zero if not set
    sMin = -90.0f; sMax = 90.0f;
  }
  float center = 0.5f * (sMin + sMax);

  // Draw background of bar area using rounded ends for a modern look
  tft.fillRect(x, barY, w, h, ST77XX_BLACK);
  // outer rounded outline
  uint8_t radius = (uint8_t)max(1, h/2);
  tft.drawRoundRect(x, barY, w, h, radius, ST77XX_WHITE);

  // Normalized positions (0..1)
  float denom = (sMax - sMin);
  if (fabsf(denom) < 0.0001f) denom = 1.0f;
  float normVal = (dist - sMin) / denom; normVal = clampf(normVal, 0.0f, 1.0f);
  float normMin = (sMin - sMin) / denom; // =0
  float normMax = (sMax - sMin) / denom; // =1
  float normCenter = (center - sMin) / denom; normCenter = clampf(normCenter, 0.0f, 1.0f);

  // Draw filled portion from left up to value using a rounded rect fill
  int16_t innerW = w - 2;
  int16_t fillW = (int16_t)roundf((float)innerW * normVal);
  if (fillW < 0) fillW = 0; if (fillW > innerW) fillW = innerW;
  if (fillW > 0) {
    // Use a slightly smaller radius for the inner rounded fill
    uint8_t innerR = (uint8_t)max(1, (h - 2) / 2);
    // 3-color blend: Red (min) -> Yellow (mid) -> Green (max)
    uint8_t r, g, b;
    if (normVal < 0.5f) {
      // Red to Yellow: increase green from 0 to 255, red stays 255
      r = 255;
      g = (uint8_t)(normVal * 2.0f * 255.0f);
      b = 0;
    } else {
      // Yellow to Green: decrease red from 255 to 0, green stays 255
      r = (uint8_t)((1.0f - (normVal - 0.5f) * 2.0f) * 255.0f);
      g = 255;
      b = 0;
    }
    uint16_t color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); // RGB565
    tft.fillRoundRect(x + 1, barY + 1, fillW, h - 2, innerR, color);
  }

  // Draw only center marker (small white dot)
  int16_t pxCenter = x + (int16_t)roundf((w - 2) * normCenter) + 1;
  tft.fillCircle(pxCenter, barY + (h/2), 2, ST77XX_WHITE);

  // If dist is NaN, clear and show placeholder (empty bar)
  if (isnan(dist)){
    // If no data, clear inner area and redraw outer rounded outline
    tft.fillRoundRect(x+1, barY+1, w-2, h-2, radius-1, ST77XX_BLACK);
    tft.drawRoundRect(x, barY, w, h, radius, ST77XX_WHITE);
  }
}

// Draw the current relative angle as text in the right-side column, aligned per row
void drawAngleTextRight(uint8_t idx, float dist) {
  if (idx > 3) return;
  int16_t yBase = SENSOR_BLOCK_Y + idx * LINE_H;
  // Right column region starts at the left edge of the indicators and ends at right padding
  int16_t xLeft = rightColX_LeftEdge();
  // Clear all the way to the physical right edge to avoid any residual pixels
  int16_t clearX = xLeft > 0 ? (xLeft - 1) : 0; // include a 1px buffer on the left
  int16_t clearW = tft.width() - clearX;
  clearRect(clearX, yBase, clearW, LINE_H);

  // Prepare text: absolute, rounded integer, no sign/decimals
  char txt[16];
  if (isnan(dist)) {
    snprintf(txt, sizeof(txt), "--");
  } else {
    int v = (int)roundf(fabsf(dist));
    snprintf(txt, sizeof(txt), "%d", v);
  }

  // Use a slightly larger, readable GFX font
  tft.setFont(&FreeSans9pt7b);
  tft.setTextSize(1);
  // Measure and position text right-aligned and vertically centered with current font
  int16_t bx, by; uint16_t bw, bh; tft.getTextBounds(txt, 0, 0, &bx, &by, &bw, &bh);
  // Keep a small padding from the absolute right edge to avoid visual clipping
  const int16_t RIGHT_PAD = 2;
  int16_t xRight = tft.width() - RIGHT_PAD;
  int16_t x = xRight - (int16_t)bw; if (x < xLeft) x = xLeft;
  int16_t y = yBase + (LINE_H - (int16_t)bh) / 2 - by;
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(x, y);
  tft.print(txt);
  // Restore default font
  tft.setFont(NULL);
}

void drawSensorBlockFrame() { 
  clearRect(0,SENSOR_BLOCK_Y,tft.width(),tft.height()-SENSOR_BLOCK_Y);
  for (int i=0;i<4;++i){
  int16_t yLabel = SENSOR_BLOCK_Y + i * LINE_H + 16; // Lowered further for better alignment
    int16_t yBar   = SENSOR_BLOCK_Y + i * LINE_H;      // Bar stays at original position
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(1);
    tft.setCursor(PAD_X - 2, yLabel); // Shift 2px left to avoid bar overlap
    if(i==0) { tft.print("Shoulder "); }
    else if(i==1) { tft.print("Upper "); }
    else if(i==2) { tft.print("Lower "); }
    else { tft.print("Hand "); }
    tft.setFont(NULL); // Restore font for subsequent drawing
    // draw empty bar outline for each sensor line
    int16_t x = VALUE_X; int16_t xRight = valueRightLimit();
    if(xRight>x){
      int16_t w = xRight - x;
      int16_t h = LINE_H - 8; if(h<8) h=8;
      int16_t barY = yBar + 4;
      tft.drawRect(x, barY, w, h, ST77XX_WHITE);
      // Only draw Min/Max labels for the Shoulder (first) row
      if (i == 0) {
        const char* minLabel = "Min";
        const char* maxLabel = "Max";
        // Use FreeSans9pt7b font for Min/Max labels
        tft.setFont(&FreeSans9pt7b);
        tft.setTextSize(1);

        // Compute vertical position just above the bar (leave 2px gap)
        int16_t bx,by; uint16_t bw,bh;
        // Min label (yellow) - center above left endpoint
        tft.setTextColor(ST77XX_YELLOW);
        tft.getTextBounds((char*)minLabel, 0, 0, &bx, &by, &bw, &bh);
        int16_t minMarkerX = x + 1; // left inner marker position
        int16_t minX = minMarkerX - (int16_t)(bw / 2);
        if (minX < x) minX = x;
        if (minX + (int16_t)bw > x + w) minX = x + w - (int16_t)bw;
        int16_t labelY = barY - (int16_t)bh - 2;
        if (labelY < 0) labelY = 0;
        tft.setCursor(minX, labelY);
        tft.print(minLabel);

        // Max label (green) - center above right endpoint
        tft.setTextColor(ST77XX_GREEN);
        tft.getTextBounds((char*)maxLabel, 0, 0, &bx, &by, &bw, &bh);
        int16_t maxMarkerX = x + w - 2; // right inner marker position
        int16_t maxX = maxMarkerX - (int16_t)(bw / 2);
        if (maxX < x) maxX = x;
        if (maxX + (int16_t)bw > x + w) maxX = x + w - (int16_t)bw;
        tft.setCursor(maxX, labelY);
        tft.print(maxLabel);
        // Restore font for subsequent lines
        tft.setFont(NULL);
      }

      // Restore text size for subsequent lines
      tft.setTextSize(2);
    }
  }
}

void drawSensorAngle(uint8_t idx, float degrees) { 
  if(idx>3) return; 
  // 'degrees' here is distance-from-zero (dist). Render it as a bar.
  drawAngleBar(idx, degrees);
  // Draw numeric value on the right side for quick glance
  drawAngleTextRight(idx, degrees);
} 

// -------------------- FRAM persistence (v3 baseline) --------------------
const uint16_t CAL_ADDR_BASE = 0;
const uint32_t CAL_MAGIC   = 0x41524352UL; // 'R''C''R''A' LE
const uint16_t CAL_VERSION = 0x0003;       // v3: fixed shoulder pulley compensation

bool framWriteBlock(uint16_t addr, const void* src, uint16_t len) { if (!framOK || !src || !len) return false; fram.write(addr, (uint8_t*)src, len); return true; }
bool framReadBlock(uint16_t addr, void* dst, uint16_t len) { if (!framOK || !dst || !len) return false; fram.read(addr, (uint8_t*)dst, len); return true; }

bool saveStateToFRAM(){ 
  if(!framOK) return false; 
  uint8_t rec[106];
  rec[0]=(uint8_t)(CAL_MAGIC&0xFF); rec[1]=(uint8_t)((CAL_MAGIC>>8)&0xFF); rec[2]=(uint8_t)((CAL_MAGIC>>16)&0xFF); rec[3]=(uint8_t)((CAL_MAGIC>>24)&0xFF); 
  rec[4]=(uint8_t)(CAL_VERSION&0xFF); rec[5]=(uint8_t)((CAL_VERSION>>8)&0xFF); 
  uint8_t* p=&rec[6]; 
  auto wr4=[&](float f){ uint8_t* q=(uint8_t*)&f; for(int i=0;i<4;i++) *p++=q[i]; }; 
  for(uint8_t ch=0;ch<4;++ch) wr4(ang[ch].zeroCont); 
  for(uint8_t ch=0;ch<4;++ch) wr4(ang[ch].lastContScaled); 
  for(uint8_t ch=0;ch<4;++ch) wr4(ang[ch].minDist); 
  for(uint8_t ch=0;ch<4;++ch) wr4(ang[ch].maxDist); 
  for(uint8_t ch=0;ch<4;++ch) *p++=(uint8_t)CH_SIGN[ch]; 
  for(uint8_t ch=0;ch<4;++ch) wr4(softMin[ch]); 
  for(uint8_t ch=0;ch<4;++ch) wr4(softMax[ch]); 
  bool ok = framWriteBlock(CAL_ADDR_BASE,rec,sizeof(rec));
  if (ok) lastFramSaveMs = millis();
  return ok; 
}

bool loadStateFromFRAM(){ 
  if(!framOK) return false; 
  uint8_t hdr[6]; 
  if(!framReadBlock(CAL_ADDR_BASE,hdr,sizeof(hdr))) return false; 
  uint32_t magic=(uint32_t)hdr[0]|((uint32_t)hdr[1]<<8)|((uint32_t)hdr[2]<<16)|((uint32_t)hdr[3]<<24); 
  if(magic!=CAL_MAGIC) return false; 
  uint16_t ver=(uint16_t)hdr[4]|((uint16_t)hdr[5]<<8); 
  if(ver==CAL_VERSION){
    uint8_t rec[106];
    if(!framReadBlock(CAL_ADDR_BASE,rec,sizeof(rec))) return false; 
    const uint8_t* p=&rec[6]; 
    auto rd4=[&](){ float f; uint8_t* q=(uint8_t*)&f; for(int i=0;i<4;i++) q[i]=*p++; return f; }; 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].zeroCont=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].lastContScaled=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].minDist=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].maxDist=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) CH_SIGN[ch]=(int8_t)(*p++); 
    for(uint8_t ch=0;ch<4;++ch) softMin[ch]=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) softMax[ch]=rd4(); 
    return true; 
  } 
  if(ver==0x0004){ // migrate the temporary ratio format to fixed mechanical scales
    uint8_t rec[122];
    if(!framReadBlock(CAL_ADDR_BASE,rec,sizeof(rec))) return false;
    const uint8_t* p=&rec[6];
    auto rd4=[&](){ float f; uint8_t* q=(uint8_t*)&f; for(int i=0;i<4;i++) q[i]=*p++; return f; };
    for(uint8_t ch=0;ch<4;++ch) ang[ch].zeroCont=rd4();
    for(uint8_t ch=0;ch<4;++ch) ang[ch].lastContScaled=rd4();
    for(uint8_t ch=0;ch<4;++ch) ang[ch].minDist=rd4();
    for(uint8_t ch=0;ch<4;++ch) ang[ch].maxDist=rd4();
    for(uint8_t ch=0;ch<4;++ch) CH_SIGN[ch]=(int8_t)(*p++);
    for(uint8_t ch=0;ch<4;++ch) softMin[ch]=rd4();
    for(uint8_t ch=0;ch<4;++ch) softMax[ch]=rd4();
    float oldScale[4];
    for(uint8_t ch=0;ch<4;++ch) oldScale[ch]=rd4();
    for(uint8_t ch=0;ch<4;++ch){
      if(isfinite(oldScale[ch]) && fabsf(oldScale[ch]) > 1e-6f){
        float k=CH_SCALE_BASE[ch]/oldScale[ch];
        ang[ch].zeroCont*=k; ang[ch].lastContScaled*=k;
        ang[ch].minDist*=k; ang[ch].maxDist*=k;
        softMin[ch]*=k; softMax[ch]*=k;
      }
    }
    return true;
  }
  if(ver==0x0002){ 
    uint8_t rec70[70]; 
    if(!framReadBlock(CAL_ADDR_BASE,rec70,sizeof(rec70))) return false; 
    const uint8_t* p=&rec70[6]; 
    auto rd4=[&](){ float f; uint8_t* q=(uint8_t*)&f; for(int i=0;i<4;i++) q[i]=*p++; return f; }; 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].zeroCont=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].lastContScaled=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].minDist=rd4(); 
    for(uint8_t ch=0;ch<4;++ch) ang[ch].maxDist=rd4(); 
    for(uint8_t ch=0;ch<4;++ch){ CH_SIGN[ch]=+1; softMin[ch]=0; softMax[ch]=0; } 
    return true; 
  } 
  if(ver==0x0001){ 
    uint8_t rec22[22]; 
    if(!framReadBlock(CAL_ADDR_BASE,rec22,sizeof(rec22))) return false; 
    const uint8_t* p=&rec22[6]; 
    for(uint8_t ch=0;ch<4;++ch){ 
      float z; uint8_t* zf=(uint8_t*)&z; 
      for(int b=0;b<4;++b) zf[b]=*p++; 
      ang[ch].zeroCont=z; ang[ch].minDist=0; ang[ch].maxDist=0; CH_SIGN[ch]=+1; softMin[ch]=0; softMax[ch]=0; 
    } 
    return true; 
  } 
  return false; 
}

// ==================== Calibration MENUS & FLOWS ====================

// Guards to prevent accidental 'Select' on entry
uint32_t calInputStallUntil = 0;            // swallow edges for a short time after entering a menu
bool     calBlockSelectUntilRelease = false; // require Select to be released once after entry before acting

enum CalState { CAL_OFF=0, CAL_INTRO, CAL_MENU, CAL_PICK, CAL_INFO, CAL_CONFIRM, CAL_SAVED };
CalState calState = CAL_OFF;
uint8_t  calIdx   = 0; // 0..3

// Action picker state
uint8_t  calAction = 0; // 0:Set Zero 1:Set Min 2:Set Max 3:Toggle Dir

const char* chName(uint8_t ch){ switch(ch){ case 0: return "Shoulder"; case 1: return "Upper"; case 2: return "Lower"; default: return "Hand"; } }

// Header/Footer helpers
const int16_t HDR_H=24, SUB_H=14, FTR_H=24; 
int16_t BODY_Y(){return HDR_H+SUB_H+2;} 
int16_t BODY_H(){return tft.height()-BODY_Y()-FTR_H;}

void drawHeader(const char* title, const char* sub){
  tft.fillRect(0,0,tft.width(),HDR_H,ST77XX_BLUE);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(2);
  tft.setCursor(PAD_X,6); tft.print(title);
  tft.fillRect(0,HDR_H,tft.width(),SUB_H,ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW); tft.setTextSize(2);
  tft.setCursor(PAD_X,HDR_H+2); tft.print(sub);
}
void drawFooterCentered(const char* text){
  int16_t yFooter=tft.height()-FTR_H;
  tft.fillRect(0,yFooter,tft.width(),FTR_H,ST77XX_BLACK);
  tft.drawRect(0,yFooter,tft.width(),FTR_H,ST77XX_WHITE);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  int16_t bx,by; uint16_t bw,bh; tft.getTextBounds((char*)text,0,0,&bx,&by,&bw,&bh);
  int16_t x=(tft.width()-bw)/2; if(x<PAD_X) x=PAD_X;
  tft.setCursor(x, yFooter + (FTR_H-bh)/2 - 1);
  tft.print(text);
}
void clearBody(){ tft.fillRect(0,BODY_Y(),tft.width(),BODY_H(),ST77XX_BLACK); }

// Simple word-wrapped text renderer for GFX fonts
// Renders text within maxWidth starting at (x,y), honoring '\n' as forced line breaks.
void drawWrappedText(const char* text, int16_t x, int16_t y, int16_t maxWidth, uint16_t color) {
  tft.setTextColor(color);
  tft.setTextSize(1);
  char line[192]; line[0] = '\0';
  const char* p = text;
  while (*p) {
    // Handle explicit newline
    if (*p == '\n') {
      if (line[0]) {
        int16_t bx, by; uint16_t bw, bh;
        tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
        tft.setCursor(x, y);
        tft.print(line);
        y += (int16_t)bh + 4;
        line[0] = '\0';
      }
      ++p;
      continue;
    }

    // Collect next word
    char word[96]; int wi = 0;
    while (*p && *p != ' ' && *p != '\n' && wi < (int)sizeof(word) - 1) {
      word[wi++] = *p++;
    }
    word[wi] = '\0';
    bool hadSpace = false;
    if (*p == ' ') { hadSpace = true; ++p; }

    // Candidate line if we append this word
    char trial[288];
    if (line[0]) snprintf(trial, sizeof(trial), "%s %s", line, word);
    else snprintf(trial, sizeof(trial), "%s", word);

    int16_t tbx, tby; uint16_t tbw, tbh;
    tft.getTextBounds(trial, 0, 0, &tbx, &tby, &tbw, &tbh);
    if (tbw <= (uint16_t)maxWidth) {
      // Fits on current line
      strncpy(line, trial, sizeof(line)); line[sizeof(line)-1] = '\0';
    } else {
      // Draw current line, start a new one with the word
      if (line[0]) {
        int16_t bx, by; uint16_t bw, bh;
        tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
        tft.setCursor(x, y);
        tft.print(line);
        y += (int16_t)bh + 4;
      }
      strncpy(line, word, sizeof(line)); line[sizeof(line)-1] = '\0';
    }
  }
  if (line[0]) {
    int16_t bx, by; uint16_t bw, bh;
    tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
    tft.setCursor(x, y);
    tft.print(line);
  }
}

// Word-wrapped text where the first line starts after an x-offset (used after a colored prefix)
int16_t drawWrappedTextAfterOffset(const char* text, int16_t x, int16_t y, int16_t maxWidth, uint16_t color, int16_t firstLineXOffset) {
  tft.setTextColor(color);
  tft.setTextSize(1);
  bool firstLine = true;
  char line[192]; line[0] = '\0';
  const char* p = text;
  while (*p) {
    if (*p == '\n') {
      if (line[0]) {
        int16_t bx, by; uint16_t bw, bh; tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
        int16_t cx = x + (firstLine ? firstLineXOffset : 0);
        tft.setCursor(cx, y); tft.print(line);
        y += (int16_t)bh + 4; line[0] = '\0'; firstLine = false;
      }
      ++p; continue;
    }
    char word[96]; int wi = 0;
    while (*p && *p != ' ' && *p != '\n' && wi < (int)sizeof(word) - 1) { word[wi++] = *p++; }
    word[wi] = '\0'; bool hadSpace = false; if (*p == ' ') { hadSpace = true; ++p; }
    char trial[288]; if (line[0]) snprintf(trial, sizeof(trial), "%s %s", line, word); else snprintf(trial, sizeof(trial), "%s", word);
    int16_t tbx, tby; uint16_t tbw, tbh; tft.getTextBounds(trial, 0, 0, &tbx, &tby, &tbw, &tbh);
    // Available width considers first-line offset
    uint16_t avail = maxWidth - (firstLine ? (uint16_t)firstLineXOffset : 0);
    if (tbw <= avail) {
      strncpy(line, trial, sizeof(line)); line[sizeof(line)-1] = '\0';
    } else {
      if (line[0]) {
        int16_t bx, by; uint16_t bw, bh; tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
        int16_t cx = x + (firstLine ? firstLineXOffset : 0);
        tft.setCursor(cx, y); tft.print(line);
        y += (int16_t)bh + 4; firstLine = false;
      }
      strncpy(line, word, sizeof(line)); line[sizeof(line)-1] = '\0';
    }
  }
  if (line[0]) {
    int16_t bx, by; uint16_t bw, bh; tft.getTextBounds(line, 0, 0, &bx, &by, &bw, &bh);
    int16_t cx = x + (firstLine ? firstLineXOffset : 0);
    tft.setCursor(cx, y); tft.print(line);
    y += (int16_t)bh + 4; // advance after last line for convenient chaining
  }
  return y;
}

// Draw an instruction line like "Set Zero:" in prefixColor and the wrapped remainder in restColor.
// Returns the next y position after drawing, including an extra gap for separation.
int16_t drawInstruction(const char* prefix, const char* rest, int16_t x, int16_t y, int16_t maxWidth, uint16_t prefixColor, uint16_t restColor) {
  // Measure prefix width including a following space
  char prefBuf[64]; snprintf(prefBuf, sizeof(prefBuf), "%s ", prefix);
  int16_t pbx, pby; uint16_t pbw, pbh; tft.getTextBounds(prefBuf, 0, 0, &pbx, &pby, &pbw, &pbh);
  // Draw prefix in its color
  tft.setTextColor(prefixColor); tft.setTextSize(1); tft.setCursor(x, y); tft.print(prefBuf);
  // Draw the rest wrapped, starting after prefix width
  int16_t nextY = drawWrappedTextAfterOffset(rest, x, y, maxWidth, restColor, (int16_t)pbw);
  // Extra spacing between sentences (one more line of spacing as requested)
  nextY += 6;
  return nextY;
}

// MENU screen
static inline uint8_t wrapIndex(int i){ return (uint8_t)((i+4)&0x03); }
void calDrawMenu(uint8_t cursor){
  // Clear bottom area where footer was
  int16_t yFooter = tft.height() - FTR_H;
  tft.fillRect(0, yFooter, tft.width(), FTR_H, ST77XX_BLACK);
  const uint8_t N=4; 
  const char* names[N] = {"Shoulder","Upper","Lower","Hand"};
  // Header
  tft.fillRect(0,0,tft.width(),28,ST77XX_BLUE); // Restore/expand banner height for full coverage
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds((char*)"Calibration", 0, 0, &bx, &by, &bw, &bh);
  int16_t bannerY = (28 - bh) / 2 - by; // Center text in 28px banner
  tft.setCursor(PAD_X, bannerY); tft.print("Calibration");
  // Body
  int16_t footerH=FTR_H; int16_t yTop=HDR_H+2; int16_t yBottom=tft.height()-footerH; tft.fillRect(0,yTop,tft.width(),yBottom-yTop,ST77XX_BLACK);
  tft.setFont(&FreeSans9pt7b); tft.setTextSize(1);
  tft.getTextBounds((char*)"1. Shoulder", 0, 0, &bx, &by, &bw, &bh);
  int16_t rowH = bh + 6; // Add padding for visual comfort
  int16_t totalH = N * rowH;
  int16_t startY = yTop + ((yBottom - yTop - totalH) / 2);
  for (uint8_t i = 0; i < N; i++) {
    int16_t y = startY + i * rowH;
    if (i == cursor) {
      tft.fillRect(0, y, tft.width(), rowH, ST77XX_BLUE);
      tft.setTextColor(ST77XX_YELLOW);
    } else {
      tft.fillRect(0, y, tft.width(), rowH, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
    }
    tft.setFont(&FreeSans9pt7b);
    tft.setTextSize(1);
    int16_t textY = y + rowH / 2 - bh / 2 - by; // Center text vertically in box
    tft.setCursor(PAD_X, textY);
    tft.print(i + 1); tft.print(". "); tft.print(names[i]);
    tft.setFont(NULL);
  }
  // Footer removed
}

// 4-option picker: Set Zero, Set Min, Set Max, Toggle Dir
void calDrawPicker(uint8_t cursor){
  int16_t yFooter = tft.height() - FTR_H;
  tft.fillRect(0, yFooter, tft.width(), FTR_H, ST77XX_BLACK);
  const uint8_t N=4;
  const char* names[N] = {"Set ABS Zero","Set REL Min","Set REL Max","Toggle Dir"};
  int16_t bx, by; uint16_t bw, bh;
  tft.fillRect(0,0,tft.width(),28,ST77XX_BLUE);
  tft.setFont(&FreeSans9pt7b); tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  char hdr[40];
  snprintf(hdr, sizeof(hdr), "%s Select Option", chName(calIdx));
  tft.getTextBounds(hdr, 0, 0, &bx, &by, &bw, &bh);
  tft.setCursor(PAD_X, (28 - bh) / 2 - by); tft.print(hdr);
  int16_t yTop = HDR_H + 2;
  int16_t yBottom = tft.height() - FTR_H;
  tft.fillRect(0, yTop, tft.width(), yBottom - yTop, ST77XX_BLACK);
  tft.setFont(&FreeSans9pt7b); tft.setTextSize(1);
  tft.getTextBounds((char*)"1. Set ABS Zero", 0, 0, &bx, &by, &bw, &bh);
  int16_t rowH = bh + 4;
  int16_t startY = yTop + ((yBottom - yTop - N * rowH) / 2);
  for (uint8_t i = 0; i < N; i++) {
    int16_t y = startY + i * rowH;
    if (i == cursor) {
      tft.fillRect(0, y, tft.width(), rowH, ST77XX_BLUE);
      tft.setTextColor(ST77XX_YELLOW);
    } else {
      tft.fillRect(0, y, tft.width(), rowH, ST77XX_BLACK);
      tft.setTextColor(ST77XX_WHITE);
    }
    tft.setFont(&FreeSans9pt7b); tft.setTextSize(1);
    int16_t textY = y + rowH / 2 - bh / 2 - by;
    tft.setCursor(PAD_X, textY);
    tft.print(i + 1); tft.print(". "); tft.print(names[i]);
    char savedVal[20];
    if (i == 0) {
      float v = ang[calIdx].zeroCont;
      if (isnan(v)) snprintf(savedVal, sizeof(savedVal), "SAVED: --");
      else snprintf(savedVal, sizeof(savedVal), "SAVED: %.1f", v);
    } else if (i == 1) {
      float v = softMin[calIdx];
      if (isnan(v)) snprintf(savedVal, sizeof(savedVal), "SAVED: --");
      else snprintf(savedVal, sizeof(savedVal), "SAVED: %.1f", v);
    } else if (i == 2) {
      float v = softMax[calIdx];
      if (isnan(v)) snprintf(savedVal, sizeof(savedVal), "SAVED: --");
      else snprintf(savedVal, sizeof(savedVal), "SAVED: %.1f", v);
    } else if (i == 3) {
      snprintf(savedVal, sizeof(savedVal), "SAVED: %+d", (int)CH_SIGN[calIdx]);
    } else {
      savedVal[0] = '\0';
    }
    int16_t sv_bx, sv_by; uint16_t sv_bw, sv_bh;
    tft.getTextBounds(savedVal, 0, 0, &sv_bx, &sv_by, &sv_bw, &sv_bh);
    tft.setCursor(tft.width() - sv_bw - PAD_X, textY);
    tft.print(savedVal);
    tft.setFont(NULL);
  }
  float minVal = softMin[calIdx], maxVal = softMax[calIdx];
  float range = isfinite(minVal) && isfinite(maxVal) ? (maxVal - minVal) : NAN;
  char rangeText[32];
  if (isnan(range)) snprintf(rangeText, sizeof(rangeText), "Range: -- deg");
  else snprintf(rangeText, sizeof(rangeText), "Range: %.1f deg", range);
  tft.setFont(&FreeSans9pt7b); tft.setTextSize(1);
  tft.setCursor(PAD_X, tft.height() - FTR_H + 4);
  tft.setTextColor(ST77XX_CYAN); tft.print(rangeText);
  tft.setFont(NULL);
}

// INFO screen (existing)
// Calibration info screen
void calDrawIntroInfo() {
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRect(0,0,tft.width(),28,ST77XX_BLUE);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  int16_t bx, by; uint16_t bw, bh;
  tft.getTextBounds((char*)"Calibration Info", 0, 0, &bx, &by, &bw, &bh);
  int16_t x = (tft.width() - bw) / 2;
  int16_t y = (28 - bh) / 2 - by;
  tft.setCursor(x, y);
  tft.print("Calibration Info");
  // Body text (word wrapped)
  int16_t bodyY = 38; // below 28px banner
  int16_t maxW = tft.width() - 2*PAD_X;
  // Use the built-in bitmap font for a slightly smaller body text
  tft.setFont(NULL);
  int16_t yy = bodyY;
  yy = drawInstruction("Set Zero:", "Saves the current position as the neutral (reference) for this joint.", PAD_X, yy, maxW, ST77XX_RED, ST77XX_WHITE);
  yy = drawInstruction("Set Min:",  "Saves the lowest allowed angle relative to Zero.", PAD_X, yy, maxW, ST77XX_RED, ST77XX_WHITE);
  yy = drawInstruction("Set Max:",  "Saves the highest allowed angle relative to Zero.", PAD_X, yy, maxW, ST77XX_RED, ST77XX_WHITE);

  yy = drawWrappedTextAfterOffset("Use these to calibrate each joint's range.", PAD_X, yy, maxW, ST77XX_WHITE, 0);
  yy = drawWrappedTextAfterOffset("Press Select to continue.", PAD_X, yy, maxW, ST77XX_WHITE, 0);
  tft.setFont(NULL);
}
void calDrawInfo(uint8_t idx){
  char sub[40]; snprintf(sub,sizeof(sub),"%s", chName(idx));
  drawHeader("Calibrate", sub);
  clearBody();
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  int16_t y=BODY_Y();
  tft.setCursor(PAD_X,y); tft.print("Last Saved:");
  tft.setFont(NULL);
  y += 14;
  char b[64];
  snprintf(b,sizeof(b),"Zero: %.1f   Min: %.1f   Max: %.1f", ang[idx].zeroCont, softMin[idx], softMax[idx]);
  tft.setCursor(PAD_X,y); tft.print(b);
  y += 18;
  tft.setTextColor(ST77XX_YELLOW); 
  tft.setCursor(PAD_X,y); tft.print("Enter: Set ZERO = current");
  y += 14;
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(PAD_X,y); tft.print("(This clears MIN/MAX and overwrites)");
  drawFooterCentered(" Back / Enter ");
}

// CONFIRM (existing)
void calDrawConfirm(uint8_t idx){
  char sub[40]; snprintf(sub,sizeof(sub),"%s", chName(idx));
  drawHeader("Confirm", sub);
  clearBody();
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_WHITE); tft.setTextSize(1);
  int16_t y=BODY_Y();
  tft.setCursor(PAD_X,y); tft.print("Overwrite saved values?");
  tft.setFont(NULL);
  y += 16;
  tft.setCursor(PAD_X,y); tft.print("• ZERO = current position");
  y += 14;
  tft.setCursor(PAD_X,y); tft.print("• MIN/MAX = 0");
  y += 14;
  tft.setCursor(PAD_X,y); tft.print("• Save to FRAM");
  drawFooterCentered(" Back / Enter ");
}

// SAVED splash (existing)
uint32_t calSavedUntil=0;
void calDrawSaved(){
  tft.fillScreen(ST77XX_BLACK);
  tft.setFont(&FreeSans9pt7b);
  tft.setTextColor(ST77XX_GREEN); tft.setTextSize(3);
  int16_t bx,by; uint16_t bw,bh; tft.getTextBounds((char*)"SAVED",0,0,&bx,&by,&bw,&bh);
  int16_t x=(tft.width()-bw)/2; int16_t y=(tft.height()-bh)/2;
  tft.setCursor(x,y); tft.print("SAVED");
  calSavedUntil = millis() + 2000; // 2 seconds
}

// Entry helpers
void calEnterIntro(){
  calState = CAL_INTRO;
  calInputStallUntil = millis() + 200;
  calBlockSelectUntilRelease = (digitalRead(BUTTON_SELECT)==LOW);
  calDrawIntroInfo();
}
void calEnterMenu(uint8_t preselect=0){
  calState=CAL_MENU; calIdx=preselect; 
  calInputStallUntil = millis() + 200; 
  calBlockSelectUntilRelease = (digitalRead(BUTTON_SELECT)==LOW);
  calDrawMenu(calIdx);
}
void calEnterPicker(uint8_t idx){
  calState=CAL_PICK; calIdx=idx; calAction=0;
  calInputStallUntil = millis() + 200;
  calBlockSelectUntilRelease = (digitalRead(BUTTON_SELECT)==LOW);
  // Reset live cache on entry
  calLiveAbs = NAN; calLiveDist = NAN; calLiveLastMs = 0;
  calDrawPicker(calAction);
}
void calEnterInfo(uint8_t idx){
  calState=CAL_INFO; calIdx=idx; 
  calInputStallUntil = millis() + 200; 
  calBlockSelectUntilRelease = (digitalRead(BUTTON_SELECT)==LOW);
  calDrawInfo(calIdx);
}
void calEnterConfirm(){
  calState=CAL_CONFIRM; 
  calInputStallUntil = millis() + 200; 
  calBlockSelectUntilRelease = (digitalRead(BUTTON_SELECT)==LOW);
  calDrawConfirm(calIdx);
}
void calEnterSaved(){ calState=CAL_SAVED; calDrawSaved(); }
void calExitToMain(){
  calState=CAL_OFF; uiScreen=UI_MAIN; 
  tft.fillScreen(ST77XX_BLACK);
  SENSOR_BLOCK_Y=tft.height()-SENSOR_BLOCK_H; 
  drawStatus(digitalRead(TOGGLE_PIN)==LOW); /* redrawArrowIconsNow(); */ drawSensorBlockFrame();
  needsMainRedraw=false; 
}

// -------------------- Setup / Loop --------------------
const uint32_t LONG_PRESS_MS_CAL =2000; // Select→Open Menu
const uint32_t LONG_PRESS_MS_STATS=5000; // Up→Stats (kept)

uint32_t selPressStart=0, upPressStart=0; bool selLongDone=false, upLongDone=false;

void drawMainUIFresh(){ 
  tft.fillScreen(ST77XX_BLACK);
  SENSOR_BLOCK_Y=tft.height()-SENSOR_BLOCK_H;
  drawStatus(digitalRead(TOGGLE_PIN)==LOW);
  // redrawArrowIconsNow(); // Removed: no arrow icons on main screen
  drawSensorBlockFrame();
  for(uint8_t ch=0; ch<4; ++ch){ 
    float deg0to360; 
    if(readAngleDeg(ch,deg0to360)){ 
      float contEnc=unwrapAngle(ch,deg0to360); 
      float contScaled=scaledWithSign(ch,contEnc); 
      ang[ch].lastContScaled=contScaled; 
      float dist=contScaled-ang[ch].zeroCont; 
      drawSensorAngle(ch,dist);
    } else { 
      drawSensorAngle(ch,NAN);
    } 
  }
  
  Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission(); 
}

bool probeFramReadWrite() {
  if (!framOK) return false;

  // Probe FRAM at a reserved address away from the calibration record.
  const uint16_t kProbeAddr = 0x03F0;
  uint8_t oldByte = 0;
  uint8_t testByte = 0;

  if (!framReadBlock(kProbeAddr, &oldByte, 1)) return false;

  testByte = (uint8_t)(oldByte ^ 0xA5);
  if (!framWriteBlock(kProbeAddr, &testByte, 1)) return false;

  uint8_t verify = 0;
  if (!framReadBlock(kProbeAddr, &verify, 1)) return false;

  // Restore original byte to keep FRAM contents stable.
  framWriteBlock(kProbeAddr, &oldByte, 1);
  return verify == testByte;
}

void printFramStartupDiagnostic(bool stateLoaded, bool crsfReady) {
  Serial.println("=== FRAM startup diagnostic ===");
  Serial.printf("FRAM address: 0x%02X\n", FRAM_ADDR);
  Serial.printf("FRAM init: %s\n", framOK ? "OK" : "FAIL");
  Serial.printf("FRAM read/write probe: %s\n", framProbeOK ? "PASS" : "FAIL");
  Serial.printf("FRAM state load: %s\n", stateLoaded ? "OK" : "FAIL");
  Serial.printf("FRAM verification: %s\n", (framOK && framProbeOK && stateLoaded) ? "PASS" : "FAIL");
  Serial.printf("CRSF init: %s\n", crsfReady ? "OK" : "FAIL");
  Serial.println("===============================");
}

void setup(){
  Serial.begin(115200);
  delay(1000); // Allow Serial to initialize
  Serial.println("\n=== ARC V2.1 Display with CRSF TX ===");
  
  SPI.begin(TFT_SCLK,-1,TFT_MOSI,TFT_CS);
  tft.init(170,320); tft.setRotation(1); tft.setTextWrap(false); tft.fillScreen(ST77XX_BLACK);
  SENSOR_BLOCK_Y=tft.height()-SENSOR_BLOCK_H;
  pinMode(BUTTON_BACK,INPUT_PULLUP); pinMode(BUTTON_SELECT,INPUT_PULLUP); pinMode(BUTTON_UP,INPUT_PULLUP); pinMode(BUTTON_DOWN,INPUT_PULLUP); pinMode(TOGGLE_PIN,INPUT_PULLUP);
  lastBack=digitalRead(BUTTON_BACK); lastSelect=digitalRead(BUTTON_SELECT); lastUp=digitalRead(BUTTON_UP); lastDown=digitalRead(BUTTON_DOWN); lastToggle=digitalRead(TOGGLE_PIN);
  Wire.begin(I2C_SDA,I2C_SCL); framOK=fram.begin(FRAM_ADDR,&Wire);

  // Initialize CRSF early so the boot screen can report its status
  crsf = new CrsfSerial(Serial1, CRSF_RX_PIN, CRSF_TX_PIN, true);
  crsf->begin(420000);

  framProbeOK = probeFramReadWrite();

  // Show boot/splash screen (5s) that reports FRAM and CRSF status
  drawBootScreen(framOK, framProbeOK, crsf->isInitialized());

  bool loaded=framOK && loadStateFromFRAM();
  printFramStartupDiagnostic(loaded, crsf->isInitialized());
  for(uint8_t ch=0; ch<4; ++ch){ 
    float deg0to360; 
    if(readAngleDeg(ch,deg0to360)){ 
      if(loaded) seedUnwrapFromScaled(ch,deg0to360,ang[ch].lastContScaled); 
      else { unwrapAngle(ch,deg0to360); ang[ch].lastContScaled=scaledWithSign(ch,deg0to360);} 
    } 
  }
  Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission();
  drawStatus(lastToggle==LOW); redrawArrowIconsNow(); drawSensorBlockFrame();
}

void loop(){
  bool bBack=(digitalRead(BUTTON_BACK)==LOW);
  bool bSelect=(digitalRead(BUTTON_SELECT)==LOW);
  bool bUp=(digitalRead(BUTTON_UP)==LOW);
  bool bDown=(digitalRead(BUTTON_DOWN)==LOW);
  bool bToggle=(digitalRead(TOGGLE_PIN)==LOW);

  

  // --- Calibration menus & flows ---
  if (calState!=CAL_OFF){
    static bool lastB=false,lastS=false,lastU=false,lastD=false; static uint32_t deb=0;
    auto edge=[&](bool now,bool &last){ if(now!=last && (millis()-deb)>80){ deb=millis(); last=now; return true;} return false; };
    bool eBack=edge(bBack,lastB), eSel=edge(bSelect,lastS), eUp=edge(bUp,lastU), eDown=edge(bDown,lastD);

    // Swallow edges immediately after entering screens
    if (millis() < calInputStallUntil) { eBack=eSel=eUp=eDown=false; }

    switch(calState){
      case CAL_INTRO:
        if (calBlockSelectUntilRelease) { if(!bSelect) calBlockSelectUntilRelease=false; }
        else { if(eSel && bSelect){ calEnterMenu(0); } }
        if(eBack && bBack){ calExitToMain(); }
        break;

      case CAL_MENU:
        if(eUp && bUp){ calIdx=wrapIndex(calIdx-1); calDrawMenu(calIdx);} 
        if(eDown && bDown){ calIdx=wrapIndex(calIdx+1); calDrawMenu(calIdx);} 
        if (calBlockSelectUntilRelease) { if(!bSelect) calBlockSelectUntilRelease=false; }
        else { if(eSel && bSelect){ calEnterPicker(calIdx);} }
        if(eBack && bBack){ calExitToMain(); }
        break;

      case CAL_PICK:
        if(eUp && bUp){ calAction = (uint8_t)((int)calAction + 3) % 4; calDrawPicker(calAction);} // wrap -1
        if(eDown && bDown){ calAction = (uint8_t)((int)calAction + 1) % 4; calDrawPicker(calAction);} // wrap +1
        if (calBlockSelectUntilRelease) { if(!bSelect) calBlockSelectUntilRelease=false; }
        else { if(eSel && bSelect){
            if (calAction == 3) {
              // Toggle direction and preserve calibration by mirroring values.
              CH_SIGN[calIdx] = (int8_t)(-CH_SIGN[calIdx]);
              ang[calIdx].zeroCont = -ang[calIdx].zeroCont;
              ang[calIdx].lastContScaled = -ang[calIdx].lastContScaled;
              float oldMinDist = ang[calIdx].minDist;
              float oldMaxDist = ang[calIdx].maxDist;
              ang[calIdx].minDist = -oldMaxDist;
              ang[calIdx].maxDist = -oldMinDist;
              float oldSoftMin = softMin[calIdx];
              float oldSoftMax = softMax[calIdx];
              softMin[calIdx] = -oldSoftMax;
              softMax[calIdx] = -oldSoftMin;
              saveStateToFRAM();
              Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission();
              calEnterSaved();
            } else {
              float deg0to360;
              if(readAngleDeg(calIdx,deg0to360)){
                float contEnc=unwrapAngle(calIdx,deg0to360);
                float contScaled=scaledWithSign(calIdx,contEnc);
                float dist = contScaled - ang[calIdx].zeroCont;
                if(calAction==0){
                  ang[calIdx].zeroCont=contScaled;
                  ang[calIdx].minDist=0; ang[calIdx].maxDist=0;
                  softMin[calIdx]=0; softMax[calIdx]=0;
                  ang[calIdx].lastContScaled=contScaled;
                } else if(calAction==1){
                  softMin[calIdx]=dist;
                } else {
                  softMax[calIdx]=dist;
                }
                saveStateToFRAM();
              }
              Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission();
              calEnterSaved();
            }
          } }
        if(eBack && bBack){ calEnterMenu(calIdx); }

        // --- Live readout update (~20 Hz) ---
        if (millis() - calLiveLastMs >= 50) {
          calLiveLastMs = millis();
          float deg0to360;
          if (readAngleDeg(calIdx, deg0to360)) {
            float contEnc    = unwrapAngle(calIdx, deg0to360);
            float contScaled = scaledWithSign(calIdx, contEnc);
            float dist       = contScaled - ang[calIdx].zeroCont;

            // Only redraw if meaningfully changed
            if (isnan(calLiveAbs) || fabsf(contScaled - calLiveAbs) > 0.1f || fabsf(dist - calLiveDist) > 0.1f) {
              calLiveAbs  = contScaled;
              calLiveDist = dist;
              // Always redraw picker to update live/saved box
              calDrawPicker(calAction);
            }
          }
          // Park MUX lines when idle
          Wire.beginTransmission(MUX_ADDR); 
          Wire.write(0x00); 
          Wire.endTransmission();
        }
        break;

      case CAL_INFO:
        if (calBlockSelectUntilRelease) { if(!bSelect) calBlockSelectUntilRelease=false; }
        else { if(eSel && bSelect){ calEnterConfirm(); } }
        if(eBack && bBack){ calEnterMenu(calIdx); }
        break;

      case CAL_CONFIRM:
        if (calBlockSelectUntilRelease) { if(!bSelect) calBlockSelectUntilRelease=false; }
        else if (eSel && bSelect){
          // Legacy: ZERO = current pos (scaled); MIN/MAX = 0
          float deg0to360; if(readAngleDeg(calIdx,deg0to360)){
            float contEnc=unwrapAngle(calIdx,deg0to360);
            float contScaled=scaledWithSign(calIdx,contEnc);
            ang[calIdx].zeroCont = contScaled;
            softMin[calIdx] = 0; softMax[calIdx] = 0;
            ang[calIdx].lastContScaled = contScaled;
            saveStateToFRAM();
          }
          Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission();
          calEnterSaved();
        }
        if(eBack && bBack){ calEnterInfo(calIdx); }
        break;

      case CAL_SAVED:
        if (millis() >= calSavedUntil){ calEnterPicker(calIdx); }
        break;

      default: break;
    }
    return; // block other UI while in menus
  }

  // --- STATS screen (optional) ---
  if (uiScreen==UI_STATS){ 
    static bool lastUpStats=false,lastDownStats=false,lastBackStats=false; 
    static uint32_t lastDebStats=0; 
    auto edgeS=[&](bool now,bool &last){ if(now!=last && (millis()-lastDebStats)>120){ lastDebStats=millis(); last=now; return true;} return false; }; 
    bool eUp=edgeS(bUp,lastUpStats), eDown=edgeS(bDown,lastDownStats), eBack=edgeS(bBack,lastBackStats); 
    if(eUp && bUp){ statsIndex=wrapIndex(statsIndex+1); needsStatsRedraw=true;} 
    if(eDown && bDown){ statsIndex=wrapIndex(statsIndex-1); needsStatsRedraw=true;} 
    if(eBack && bBack){ uiScreen=UI_MAIN; drawMainUIFresh(); needsMainRedraw=false; } 
    if(needsStatsRedraw){ 
      tft.fillScreen(ST77XX_BLACK); 
      tft.fillRect(0,0,tft.width(),28,ST77XX_GREEN); 
      tft.setTextColor(ST77XX_BLACK); tft.setTextSize(2); tft.setCursor(PAD_X,6); tft.print("Saved Ranges"); 
      tft.setTextColor(ST77XX_YELLOW); tft.setCursor(PAD_X,36); tft.print(chName(statsIndex)); 
      tft.setTextColor(ST77XX_WHITE); 
      int16_t y=36+LINE_H+6; 
      char buf[32]; 
      snprintf(buf,sizeof(buf),"Zero: %.1f deg", ang[statsIndex].zeroCont); tft.setCursor(PAD_X,y); tft.print(buf); 
      y+=LINE_H; snprintf(buf,sizeof(buf),"Min:  %.1f deg", softMin[statsIndex]); tft.setCursor(PAD_X,y); tft.print(buf); 
      y+=LINE_H; snprintf(buf,sizeof(buf),"Max:  %.1f deg", softMax[statsIndex]); tft.setCursor(PAD_X,y); tft.print(buf); 
      tft.setTextSize(1); tft.setCursor(PAD_X,tft.height()-18); tft.print("UP/DOWN: Scroll   Back: Exit"); 
      needsStatsRedraw=false; 
    } 
    return; 
  }

  // --- MAIN screen logic ---
  static uint32_t lastMs=0; auto changed=[&](bool now,bool &last){ if(now!=last && (millis()-lastMs)>30){ lastMs=millis(); last=now; return true;} return false; };

  // Select long-press → open calibration intro
  if(bSelect){ if(selPressStart==0){ selPressStart=millis(); selLongDone=false; } else if(!selLongDone && millis()-selPressStart>=LONG_PRESS_MS_CAL){ selLongDone=true; calEnterIntro(); return; } } else { selPressStart=0; selLongDone=false; }

  // Up long-press → Stats (optional)
  if(bUp){ if(upPressStart==0){ upPressStart=millis(); upLongDone=false; } else if(!upLongDone && millis()-upPressStart>=LONG_PRESS_MS_STATS){ upLongDone=true; uiScreen=UI_STATS; statsIndex=0; needsStatsRedraw=true; return; } } else { upPressStart=0; upLongDone=false; }

  // Icons + status
  if(changed(bToggle,lastToggle)) drawStatus(bToggle);
  // if(changed(bBack,lastBack))     drawArrowIcon(0,bBack);
  // if(changed(bSelect,lastSelect)) drawArrowIcon(1,bSelect);
  // if(changed(bUp,lastUp))         drawArrowIcon(2,bUp);
  // if(changed(bDown,lastDown))     drawArrowIcon(3,bDown);

  // Redraw main if needed
  if(needsMainRedraw){ drawMainUIFresh(); needsMainRedraw=false; }

  // Sensor polling (~20 Hz)
  static uint32_t lastSense=0; 
  if(millis()-lastSense>=50){ 
    lastSense=millis(); bool anyChanged=false; 
    // Sample all sensors first, then draw them in a tight batch to avoid
    // a visible diagonal refresh caused by per-channel read/draw delays.
    float sampledDist[4]; bool sampledOk[4];
    for(uint8_t ch=0; ch<4; ++ch){ 
      float deg0to360; bool ok = readAngleDeg(ch,deg0to360);
      sampledOk[ch] = ok;
      if(!ok){ sampledDist[ch] = NAN; continue; }
      float contEnc = unwrapAngle(ch,deg0to360);
      float contScaled = scaledWithSign(ch,contEnc);
      float dist = contScaled - ang[ch].zeroCont;

      // update min/max/last samples as before
      if(dist < ang[ch].minDist){ ang[ch].minDist = dist; anyChanged = true; }
      if(dist > ang[ch].maxDist){ ang[ch].maxDist = dist; anyChanged = true; }
      if(fabsf(contScaled - ang[ch].lastContScaled) > 0.1f){ ang[ch].lastContScaled = contScaled; anyChanged = true; }

      sampledDist[ch] = dist;
    }

    // Only redraw bars if any joint value changed
    if (anyChanged) {
      for(uint8_t ch=0; ch<4; ++ch){ 
        if(!sampledOk[ch]) drawSensorAngle(ch, NAN);
        else drawSensorAngle(ch, sampledDist[ch]);
      }
    }
    Wire.beginTransmission(MUX_ADDR); Wire.write(0x00); Wire.endTransmission(); 
    static uint32_t lastPersist=0; 
    if(framOK && (anyChanged || (millis()-lastPersist>=2000))){ 
      if (saveStateToFRAM()) { /* lastFramSaveMs updated inside */ }
      lastPersist=millis(); 
    }
  // Refresh FRAM indicator each cycle so the blink decays naturally
  drawFramIndicator(framOK);
  // Update CRSF indicator so UI reflects the latest send timing
  drawCrsfIndicator();
  }

  // Update CRSF channels and drive transmissions at ~250 Hz regardless of sensor polling cadence
  if (crsf != nullptr) {
    // Set channels 1-4 mapped explicitly to microseconds [1000..2000]
    // Using linear mapping: softMin -> 1000us, softMax -> 2000us, midpoint -> 1500us
    for (uint8_t ch = 0; ch < 4; ++ch) {
      uint16_t us = 1500;
      if (ang[ch].inited && isfinite(ang[ch].lastContScaled)) {
        float dist = ang[ch].lastContScaled - ang[ch].zeroCont;
        if (!isnan(dist)) {
          float sMin = softMin[ch];
          float sMax = softMax[ch];
          if (!isfinite(sMin) || !isfinite(sMax) || fabsf(sMax - sMin) < 0.1f) {
            // Fallback symmetric range if not configured
            sMin = -90.0f; sMax = +90.0f;
          }
          float usf = 1000.0f + (dist - sMin) * (2000.0f - 1000.0f) / (sMax - sMin);
          if (!isfinite(usf)) usf = 1500.0f;
          if (usf < 1000.0f) usf = 1000.0f;
          if (usf > 2000.0f) usf = 2000.0f;
          us = (uint16_t)lroundf(usf);
        }
      }
      crsf->setChannelUs((uint8_t)(ch + 1), us);
    }
    // Channel 5: Power/Arm switch (explicit high/low in microseconds)
    // Arming toggle is INPUT_PULLUP, active-low when ARMED
    crsf->setChannelUs(5, bToggle ? 2000 : 1000);

    // Channel 6 fixed low at 1000us at all times
    crsf->setChannelUs(6, 1000);
    // Keep channels 7-16 neutral (center)
    for (uint8_t i = 7; i <= 16; i++) {
      crsf->setChannelFloat(i, 0.0f);
    }
    
    // Debug: print exact values being sent on CRSF as floats [-1.000 .. +1.000]
    // These come from the same internal ticks used to transmit, so they reflect the TX state.
    static uint32_t lastDebug = 0;
    if (millis() - lastDebug >= 200) { // ~5 Hz; adjust if you want more/less
      lastDebug = millis();
      Serial.print("CH1: "); Serial.println(crsf->getChannelFloat(1), 3);
      Serial.print("CH2: "); Serial.println(crsf->getChannelFloat(2), 3);
      Serial.print("CH3: "); Serial.println(crsf->getChannelFloat(3), 3);
      Serial.print("CH4: "); Serial.println(crsf->getChannelFloat(4), 3);
      Serial.print("CH5: "); Serial.println(crsf->getChannelFloat(5), 3);
      Serial.print("CH6: "); Serial.println(crsf->getChannelFloat(6), 3);
      Serial.println();
    }
    
    crsf->update();
  }
}
