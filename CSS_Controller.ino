/* =============================================================================
 * CSS Controller  v0.8
 * Stuart Robins - https://github.com/DreamMachine/constant-surface-speed
 * Constant Surface Speed controller for a lathe  —  ESP32 WROOM-32D
 *
 * Reads cross-slide position (magnetic scale, x4 quadrature via PCNT),
 * measures spindle RPM (Hall, 1 pulse/rev), computes the target RPM to hold a
 * constant surface speed as the tool feeds toward centre, and commands an
 * SU-800/900 VFD over Modbus RTU to track it.
 *
 * v0.7 changes vs v0.5/v0.6:
 *   - Non-touch ILI9341 on HSPI @ 40 MHz, landscape (setRotation(1))
 *   - VINKA capacitive 4x4 keypad over I2C (replaces on-screen touch buttons)
 *   - Landscape 320x240 layout with a SET REF numeric-entry area
 *   - All XPT2046 / touch code removed
 *
 * Libraries (Arduino / ESP32 core):
 *   Adafruit_GFX, Adafruit_ILI9341, Wire, WiFi, WebServer, driver/pcnt.h
 *
 * NOTE on driver/pcnt.h: this is the legacy PCNT driver. It compiles on core
 * 2.x cleanly and on core 3.x with deprecation warnings (still functional).
 * This matches the "hardware PCNT + overflow ISR" carried from v1.5/v1.6.
 * ============================================================================= */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "driver/pcnt.h"

// ----------------------------------------------------------------------------
// PIN MAP  (final for v0.8 — see brief)
// ----------------------------------------------------------------------------
// Quadrature (cross-slide), from 74HC245 buffer via 1k/2k divider, 3.33V
#define PIN_QUAD_A     32
#define PIN_QUAD_B     33
// Hall spindle sensor, 22k + 1N4148 clamp + 100nF
#define PIN_HALL       14
// VINKA keypad, I2C
#define PIN_KP_SDA     25
#define PIN_KP_SCL     26
#define PIN_KP_INT     27      // active LOW on key press
// ILI9341 display, HSPI
#define PIN_TFT_SCK    22
#define PIN_TFT_MOSI   23
#define PIN_TFT_CS      5
#define PIN_TFT_DC      4
#define PIN_TFT_RST    16      // board silk: RX2
#define PIN_TFT_BL     17      // board silk: TX2
// RS485 / MAX485 -> VFD
#define PIN_RS485_TX   21      // -> DI
#define PIN_RS485_RX   19      // <- RO (through 1k/2k divider)
#define PIN_RS485_DE   18      // DE + RE tied: HIGH = TX, LOW = RX
// Front-panel momentary RESET pushbutton, to GND (active low). GPIO13 is free,
// has an internal pull-up, and is not a strapping pin — no external parts needed.
#define PIN_RESET_BTN  13

// ----------------------------------------------------------------------------
// CSS ENGINE PARAMETERS   *** CALIBRATE THESE ON THE MACHINE ***
// ----------------------------------------------------------------------------
long   SCALE_COUNTS_PER_MM = 800;     // 5um scale x4. Measure & set exactly.
float  VFD_BASE_RPM        = 1450.0;  // measured spindle RPM at VFD_BASE_FREQ_HZ
float  VFD_BASE_FREQ_HZ    = 50.0;    // frequency at which VFD_BASE_RPM is reached
float  VFD_MAX_FREQ_HZ     = 50.0;    // == VFD param P0-10. See note below.
float  DIAMETER_MULTIPLIER = 0.8;     // radius -> diameter
int    DIAMETER_DIR        = 1;       // sign of travel: feeding to centre must
                                      //   REDUCE diameter. Flipped from -1 after
                                      //   bench test showed speed change reversed.
float  MIN_DIAMETER_MM     = 5.0;     // clamp: prevents RPM runaway near centre
float  MAX_RPM             = 2000.0;
float  MAX_FREQ_SLEW_HZ    = 5.0;     // max commanded-freq change per 20 Hz cycle
float  SURFACE_SPEED_DEF   = 100.0;   // m/min at boot
float  VC_MIN              = 1.0;
float  VC_MAX              = 500.0;

// ---- RPM divergence guard (stall / belt-slip protection) ----
// Compares Hall RPM against the RPM the commanded frequency should produce.
// OFF by default: only turn on once the Hall reading is trusted, or it will
// nuisance-trip whenever the sensor is disconnected/noisy. Live-adjustable.
bool     divEnabled  = false;   // 'div on' / 'div off'
int      divTolPct   = 40;      // allowed deviation, % of expected RPM ('divtol N')
uint32_t divDelayMs  = 1500;    // must stay out of tolerance this long to trip ('divdelay N')
#define  DIV_GRACE_MS        5000    // ignore for this long after a start (spin-up ramp)
#define  DIV_MIN_EXPECTED_RPM 100    // don't check below this commanded speed

// NOTE on VFD_MAX_FREQ_HZ / P0-10:
//   Register 0x1000 takes a PERCENTAGE of P0-10 (0.01% units, 10000 = 100%).
//   commanded reg = freqHz / VFD_MAX_FREQ_HZ * 10000, clamped 0..10000.
//   To actually reach MAX_RPM you need P0-10 (and VFD_MAX_FREQ_HZ here) high
//   enough: freq_at_MAX_RPM = MAX_RPM * VFD_BASE_FREQ_HZ / VFD_BASE_RPM.
//   e.g. 2000 rpm -> 2000*50/1450 = 69.0 Hz. With P0-10 = 50 Hz the drive
//   saturates at 100% (1450 rpm). Set P0-10 to ~70 Hz (and match here) if you
//   want the full 2000 rpm range.

// ----------------------------------------------------------------------------
// VFD / MODBUS
// ----------------------------------------------------------------------------
// ---- runtime link settings (changeable from web/serial, no recompile) ----
uint32_t mbBaud   = 9600;               // Modbus baud rate
uint8_t  vfdSlave = 1;                  // Modbus slave address (VFD Pd-02)
struct SerFmt { const char* name; uint32_t cfg; };
const SerFmt SER_FMTS[] = {
  {"8N1", SERIAL_8N1}, {"8N2", SERIAL_8N2}, {"8E1", SERIAL_8E1}, {"8O1", SERIAL_8O1}
};
const int SER_FMT_COUNT = 4;
int mbFmtIdx = 1;                        // 0..3 index into SER_FMTS; default 8N2 (per brief)
#define REG_FREQ_SET       0x1000   // % of P0-10, 0.01% units
#define REG_RUN_CMD        0x2000   // 1 fwd, 2 rev, 5 coast, 6 decel, 7 fault-reset
#define REG_FREQ_RB        0x1001   // operating freq read-back (0.01 Hz? per manual)
#define REG_STATUS         0x3000   // status word
// Command/frequency source parameters, written at their RAM address (P0-group
// high byte 0x00 = RAM-only, not EEPROM) so toggling CSS never wears EEPROM.
//   P0-02 command source:   0=panel, 1=terminal, 2=communication
//   P0-03 frequency source: 4=keyboard potentiometer, 9=communication
// NOTE: only the COMMAND source is switched by firmware (it applies live from
// RAM). The frequency source is left at P0-03 and follows the command source via
// the drive's P0-27 "command-source bound frequency source" (set hundreds digit
// = 9). REG_FREQ_SRC_RAM / SRC_FREQ_COMM / CSS_OFF_FREQ_SRC are retained for
// reference but no longer written.
#define REG_CMD_SRC_RAM    0x0002   // P0-02 in RAM
#define REG_FREQ_SRC_RAM   0x0003   // P0-03 in RAM (unused now; see note above)
#define SRC_CMD_COMM       2
#define SRC_FREQ_COMM      9
// What the drive returns to when CSS is switched OFF. Change CSS_OFF_CMD_SRC to
// 1 (terminal) once the carriage lever is wired to a digital input (X1).
uint16_t CSS_OFF_CMD_SRC  = 0;      // 0 = VFD panel run/stop
uint16_t CSS_OFF_FREQ_SRC = 4;      // (unused now) panel pot — set via P0-03 on the drive
#define VFD_RUN_FWD        1
#define VFD_RUN_REV        2
#define VFD_STOP_COAST     5
#define VFD_STOP_DECEL     6
#define VFD_FAULT_RESET    7
#define MODBUS_TIMEOUT_MS  60      // a valid reply arrives in <20 ms even at 4800;
                                   // shorter = the loop recovers faster on no-reply
#define MODBUS_MAX_FAILS   5
#define RS485_TX_HOLD_US   1200     // hold DE high after flush() so the last char
                                    // + stop bits fully clear the wire before RX
                                    // (1 char @ 9600 8N2 ~ 1.15 ms)
// Debug verbosity is now a RUNTIME level (set from web or serial: 'debug N'):
//   0 = off, 1 = errors only, 2 = all TX/RX frames.  See debugLevel below.

// ----------------------------------------------------------------------------
// TIMING
// ----------------------------------------------------------------------------
#define CSS_PERIOD_MS      50       // 20 Hz control loop
#define DISP_PERIOD_MS     200      // 5 Hz display refresh
#define KEYPAD_PERIOD_MS   20       // keypad poll
#define KEY_REPEAT_MS      180      // hold-repeat for 0/5

// ----------------------------------------------------------------------------
// WiFi
// ----------------------------------------------------------------------------
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

// ----------------------------------------------------------------------------
// DISPLAY
// ----------------------------------------------------------------------------
SPIClass hspi(HSPI);
Adafruit_ILI9341 tft = Adafruit_ILI9341(&hspi, PIN_TFT_DC, PIN_TFT_CS, PIN_TFT_RST);

#define C_BG      ILI9341_BLACK
#define C_LABEL   ILI9341_DARKGREY
#define C_VALUE   ILI9341_WHITE
#define C_OK      ILI9341_GREEN
#define C_WARN    ILI9341_YELLOW
#define C_BAD     ILI9341_RED
#define C_ACCENT  ILI9341_CYAN

// ----------------------------------------------------------------------------
// WEB
// ----------------------------------------------------------------------------
WebServer server(80);

// ============================================================================
// GLOBAL STATE
// ============================================================================
float   surfaceSpeed = SURFACE_SPEED_DEF;   // m/min (target)
float   refDiameter  = 50.0;                // mm, at refPosition
int64_t refPosition  = 0;                   // scale counts at reference
float   curDiameter  = 50.0;                // mm, live
float   targetRPM    = 0.0;
float   commandedFreq = 0.0;                // Hz, after slew

bool    cssEnabled   = false;
bool    vfdRunning   = false;
bool    vfdFault     = false;
volatile int modbusFails = 0;

enum Mode { MODE_NORMAL, MODE_SETREF, MODE_SETVC };
Mode    mode = MODE_NORMAL;
String  setRefBuf = "";

// A screen slot we redraw only when its value changes. Defined here (not in the
// DISPLAY section) so the Arduino IDE's auto-generated function prototypes,
// which it injects near the top of the file, can see the type before use.
struct Field { int x, y; String last; };

// ============================================================================
// QUADRATURE  (PCNT unit 0, x4, with overflow accumulation)
// ============================================================================
#define PCNT_UNIT_USE  PCNT_UNIT_0
#define PCNT_H_LIM     30000
#define PCNT_L_LIM    -30000

volatile int64_t pcntAccum = 0;
portMUX_TYPE pcntMux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR pcntOverflowISR(void* arg) {
  uint32_t st = 0;
  pcnt_get_event_status(PCNT_UNIT_USE, &st);
  portENTER_CRITICAL_ISR(&pcntMux);
  if (st & PCNT_EVT_H_LIM) pcntAccum += PCNT_H_LIM;
  else if (st & PCNT_EVT_L_LIM) pcntAccum += PCNT_L_LIM;
  portEXIT_CRITICAL_ISR(&pcntMux);
}

void setupPCNT() {
  // Channel 0: pulse = A, ctrl = B
  pcnt_config_t c0 = {};
  c0.pulse_gpio_num = PIN_QUAD_A;
  c0.ctrl_gpio_num  = PIN_QUAD_B;
  c0.channel        = PCNT_CHANNEL_0;
  c0.unit           = PCNT_UNIT_USE;
  c0.pos_mode       = PCNT_COUNT_INC;
  c0.neg_mode       = PCNT_COUNT_DEC;
  c0.lctrl_mode     = PCNT_MODE_REVERSE;
  c0.hctrl_mode     = PCNT_MODE_KEEP;
  c0.counter_h_lim  = PCNT_H_LIM;
  c0.counter_l_lim  = PCNT_L_LIM;
  pcnt_unit_config(&c0);

  // Channel 1: pulse = B, ctrl = A  (gives full x4)
  pcnt_config_t c1 = {};
  c1.pulse_gpio_num = PIN_QUAD_B;
  c1.ctrl_gpio_num  = PIN_QUAD_A;
  c1.channel        = PCNT_CHANNEL_1;
  c1.unit           = PCNT_UNIT_USE;
  c1.pos_mode       = PCNT_COUNT_INC;
  c1.neg_mode       = PCNT_COUNT_DEC;
  c1.lctrl_mode     = PCNT_MODE_KEEP;
  c1.hctrl_mode     = PCNT_MODE_REVERSE;
  c1.counter_h_lim  = PCNT_H_LIM;
  c1.counter_l_lim  = PCNT_L_LIM;
  pcnt_unit_config(&c1);

  // light glitch filter (APB cycles); scales are clean TTL but this is cheap
  pcnt_set_filter_value(PCNT_UNIT_USE, 1023);
  pcnt_filter_enable(PCNT_UNIT_USE);

  pcnt_event_enable(PCNT_UNIT_USE, PCNT_EVT_H_LIM);
  pcnt_event_enable(PCNT_UNIT_USE, PCNT_EVT_L_LIM);

  pcnt_counter_pause(PCNT_UNIT_USE);
  pcnt_counter_clear(PCNT_UNIT_USE);
  pcnt_isr_service_install(0);
  pcnt_isr_handler_add(PCNT_UNIT_USE, pcntOverflowISR, NULL);
  pcnt_counter_resume(PCNT_UNIT_USE);
}

int64_t readPosition() {
  int16_t c = 0;
  portENTER_CRITICAL(&pcntMux);
  pcnt_get_counter_value(PCNT_UNIT_USE, &c);
  int64_t p = pcntAccum + (int64_t)c;
  portEXIT_CRITICAL(&pcntMux);
  return p;
}

void zeroPosition() {
  portENTER_CRITICAL(&pcntMux);
  pcnt_counter_clear(PCNT_UNIT_USE);
  pcntAccum = 0;
  portEXIT_CRITICAL(&pcntMux);
  refPosition = 0;
}

// ============================================================================
// HALL  (spindle RPM, 1 pulse/rev, FALLING edge)
// ============================================================================
volatile uint32_t hallLastUs = 0;
volatile uint32_t hallPeriodUs = 0;
volatile uint32_t hallMinPeriodUs = 20000;   // ignore pulses closer than this (de-glitch /
                                             // reject VFD noise). 20 ms => rejects >3000 rpm.
                                             // Live-adjustable: 'hallms N'.
#define HALL_TIMEOUT_US     2000000  // 2 s with no pulse -> 0 rpm

void IRAM_ATTR hallISR() {
  uint32_t now = micros();
  uint32_t dt = now - hallLastUs;
  if (dt < hallMinPeriodUs) return;   // de-glitch
  hallLastUs = now;
  hallPeriodUs = dt;
}

float getRPM() {
  uint32_t period, last;
  noInterrupts();
  period = hallPeriodUs;
  last   = hallLastUs;
  interrupts();
  if (period == 0) return 0.0;
  if ((micros() - last) > HALL_TIMEOUT_US) return 0.0;
  return 60000000.0 / (float)period;
}

// ============================================================================
// MODBUS RTU MASTER  (FC 03 read, FC 06 write, inline CRC)
// ============================================================================
uint16_t modbusCRC(const uint8_t* buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
      else              { crc >>= 1; }
    }
  }
  return crc;
}

int modbusReadBytes(uint8_t* buf, int want) {
  uint32_t start = millis();
  int i = 0;
  while (i < want && (millis() - start) < MODBUS_TIMEOUT_MS) {
    if (Serial2.available()) { buf[i++] = Serial2.read(); start = millis(); }
  }
  return i;
}

// Read one reply frame of exactly frameLen bytes, tolerating leading junk.
// The RS485 transmit->receive turnaround (no line biasing) injects a phantom
// byte (typically 0x00) before the real reply, so we discard bytes until the
// slave address appears, then read the rest aligned. *skipped returns how many
// leading bytes were dropped. Returns bytes stored in out, or -1 on timeout.
int modbusReadFrame(uint8_t* out, int frameLen, int* skipped) {
  *skipped = 0;
  bool got = false;
  uint32_t t = millis();
  while ((millis() - t) < MODBUS_TIMEOUT_MS) {
    if (Serial2.available()) {
      uint8_t b = Serial2.read();
      t = millis();
      if (b == vfdSlave) { out[0] = b; got = true; break; }   // frame start
      if (++(*skipped) > 8) return -1;                        // too much junk
    }
  }
  if (!got) return -1;
  int i = 1; t = millis();
  while (i < frameLen && (millis() - t) < MODBUS_TIMEOUT_MS) {
    if (Serial2.available()) { out[i++] = Serial2.read(); t = millis(); }
  }
  return i;
}

// ---- runtime debug + log ring (viewable on the web console) ----
volatile int debugLevel = 0;            // 0=off, 1=errors, 2=all frames
#define LOG_CAP 80
String   logBuf[LOG_CAP];
uint32_t logSeq   = 0;                  // total lines ever logged (monotonic)
int      logCount = 0;                  // valid entries currently held in the ring

void logLine(const String& s) {
  Serial.println(s);                    // still goes to serial when a laptop is attached
  logBuf[logSeq % LOG_CAP] = s;
  logSeq++;
  if (logCount < LOG_CAP) logCount++;
}

String jsonEscape(const String& in) {
  String o; o.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if ((uint8_t)c >= 0x20) o += c;      // drop control chars
  }
  return o;
}

// Turn a Modbus TX frame into a short plain-English label, so the log reads
// e.g. "MB TX06: 01 06 20 00 00 06 02 08  (decel stop)". Kept terse to stay on
// one line. Decodes from the frame's function code, register and value.
String mbLabel(const uint8_t* b, int n) {
  if (n < 6) return "";
  uint16_t reg = ((uint16_t)b[2] << 8) | b[3];
  uint16_t val = ((uint16_t)b[4] << 8) | b[5];
  if (b[1] == 0x06) {                                 // write
    switch (reg) {
      case REG_RUN_CMD:
        if (val == VFD_RUN_FWD)     return "run fwd";
        if (val == VFD_RUN_REV)     return "run rev";
        if (val == VFD_STOP_COAST)  return "coast stop";
        if (val == VFD_STOP_DECEL)  return "decel stop";
        if (val == VFD_FAULT_RESET) return "fault reset";
        return "run cmd";
      case REG_FREQ_SET:     return "set freq " + String(val / 100) + "%";
      case REG_CMD_SRC_RAM:
        if (val == SRC_CMD_COMM) return "cmd src -> comms";
        if (val == 0)            return "cmd src -> panel";
        if (val == 1)            return "cmd src -> terminal";
        return "cmd src";
      case REG_FREQ_SRC_RAM: return "freq src";
      default:               return "write reg";
    }
  } else if (b[1] == 0x03) {                           // read
    switch (reg) {
      case REG_FREQ_RB: return "read op-freq";
      case REG_STATUS:  return "read status";
      default:          return "read reg";
    }
  }
  return "";
}

void mbDump(const char* tag, const uint8_t* b, int n) {
  if (debugLevel < 2) return;
  String s = tag;
  char h[5];
  for (int i = 0; i < n; i++) { snprintf(h, sizeof(h), " %02X", b[i]); s += h; }
  if (n == 0) s += " (none)";
  if (strstr(tag, "TX")) {                             // annotate the command frames
    String lbl = mbLabel(b, n);
    if (lbl.length()) s += "  (" + lbl + ")";
  }
  logLine(s);
}

void mbErr(const char* why, int n) {
  if (debugLevel < 1) return;
  logLine(String("MB ERR : ") + why + " (" + n + " bytes)");
}

void modbusOK()   { modbusFails = 0; }

void modbusFail() {
  if (modbusFails < 100000) modbusFails++;
  if (modbusFails >= MODBUS_MAX_FAILS && !vfdFault) {
    vfdFault   = true;
    cssEnabled = false;
    // NOTE: we do NOT try to drive a stop from here (the link is down, and it
    // would recurse). The VFD's own Pd-04 comms timeout (~1.0 s) is the primary
    // safety and will decel-stop/fault the drive on its own. The control loop
    // also keeps sending 0 Hz best-effort while faulted.
  }
}

bool modbusWriteReg(uint16_t reg, uint16_t val) {
  uint8_t f[8];
  f[0] = vfdSlave; f[1] = 0x06;
  f[2] = reg >> 8;  f[3] = reg & 0xFF;
  f[4] = val >> 8;  f[5] = val & 0xFF;
  uint16_t crc = modbusCRC(f, 6);
  f[6] = crc & 0xFF; f[7] = crc >> 8;

  while (Serial2.available()) Serial2.read();          // clear stale rx
  digitalWrite(PIN_RS485_DE, HIGH); delayMicroseconds(20);
  Serial2.write(f, 8); Serial2.flush();                // drain TX FIFO
  delayMicroseconds(RS485_TX_HOLD_US);                 // let last char fully clear
  digitalWrite(PIN_RS485_DE, LOW);                     // release driver -> receive
  mbDump("MB TX06:", f, 8);

  uint8_t r[8];
  int skipped = 0;
  int n = modbusReadFrame(r, 8, &skipped);          // aligns on slave addr, skips phantom
  if (n > 0) mbDump(skipped ? "MB RX+ :" : "MB RX  :", r, n);
  if (n < 8)                                    { mbErr("write: short/no reply", n); modbusFail(); return false; }
  uint16_t rc = modbusCRC(r, 6);
  if (r[6] != (rc & 0xFF) || r[7] != (rc >> 8)) { mbErr("write: bad CRC", n);        modbusFail(); return false; }
  if (r[1] != 0x06)                             { mbErr("write: func mismatch", n);  modbusFail(); return false; }
  modbusOK();
  return true;
}

bool modbusReadReg(uint16_t reg, uint16_t& out) {
  uint8_t f[8];
  f[0] = vfdSlave; f[1] = 0x03;
  f[2] = reg >> 8;  f[3] = reg & 0xFF;
  f[4] = 0x00;      f[5] = 0x01;
  uint16_t crc = modbusCRC(f, 6);
  f[6] = crc & 0xFF; f[7] = crc >> 8;

  while (Serial2.available()) Serial2.read();
  digitalWrite(PIN_RS485_DE, HIGH); delayMicroseconds(20);
  Serial2.write(f, 8); Serial2.flush();
  delayMicroseconds(RS485_TX_HOLD_US);
  digitalWrite(PIN_RS485_DE, LOW);
  mbDump("MB TX03:", f, 8);

  uint8_t r[7];                                       // slave,fc,bc,hi,lo,crcL,crcH
  int skipped = 0;
  int n = modbusReadFrame(r, 7, &skipped);
  if (n > 0) mbDump(skipped ? "MB RX+ :" : "MB RX  :", r, n);
  if (n < 7)                                    { mbErr("read: short/no reply", n); modbusFail(); return false; }
  uint16_t rc = modbusCRC(r, 5);
  if (r[5] != (rc & 0xFF) || r[6] != (rc >> 8)) { mbErr("read: bad CRC", n);        modbusFail(); return false; }
  if (r[1] != 0x03)                             { mbErr("read: func mismatch", n);  modbusFail(); return false; }
  out = ((uint16_t)r[3] << 8) | r[4];
  modbusOK();
  return true;
}

// (Re)open Serial2 with the current baud/format. Called at boot and whenever a
// link setting changes.
void applySerial() {
  Serial2.end();
  Serial2.begin(mbBaud, SER_FMTS[mbFmtIdx].cfg, PIN_RS485_RX, PIN_RS485_TX);
}

// Sweep every baud x format at the current address, one FC03 read each, and
// stop on the first that replies. Blocks a few seconds — bench bring-up only.
String mbScan() {
  logLine("--- MB SCAN start (addr " + String(vfdSlave) + ") ---");
  uint32_t savedBaud = mbBaud; int savedFmt = mbFmtIdx;
  const uint32_t bl[] = {9600, 19200, 38400, 4800, 57600, 115200};
  bool found = false; uint32_t fBaud = 0; int fFmt = 0;
  for (int bi = 0; bi < 6 && !found; bi++) {
    for (int fi = 0; fi < SER_FMT_COUNT && !found; fi++) {
      mbBaud = bl[bi]; mbFmtIdx = fi; applySerial();
      delay(25);
      uint16_t v = 0;
      bool ok = modbusReadReg(REG_FREQ_RB, v);
      logLine(String("try ") + bl[bi] + " " + SER_FMTS[fi].name + (ok ? "  -> REPLY" : "  -> none"));
      if (ok) { found = true; fBaud = bl[bi]; fFmt = fi; }
      delay(0);                                    // feed the task watchdog
    }
  }
  modbusFails = 0; vfdFault = false;               // scanning racks up failures; clear
  if (found) {
    mbBaud = fBaud; mbFmtIdx = fFmt; applySerial();
    logLine("MB SCAN: match " + String(fBaud) + " " + SER_FMTS[fFmt].name + " — link set");
    return "MB SCAN found: " + String(fBaud) + " " + String(SER_FMTS[fFmt].name);
  }
  mbBaud = savedBaud; mbFmtIdx = savedFmt; applySerial();
  logLine("MB SCAN: no reply on any baud/format");
  return "MB SCAN: no match (restored " + String(mbBaud) + " " + String(SER_FMTS[mbFmtIdx].name) + ")";
}

// ============================================================================
// VFD CONTROL
// ============================================================================
uint32_t runSince = 0;   // millis() when the spindle last started (for divergence grace)

void vfdStart()      { if (modbusWriteReg(REG_RUN_CMD, VFD_RUN_FWD)) { vfdRunning = true; runSince = millis(); } }
void vfdStop()       { if (modbusWriteReg(REG_RUN_CMD, VFD_STOP_DECEL)) vfdRunning = false; }
void vfdFaultReset() { modbusWriteReg(REG_RUN_CMD, VFD_FAULT_RESET); }

// Hand the drive to Modbus control (CSS on) / back to the panel (CSS off).
// We switch ONLY the command source (P0-02) — confirmed to apply live from RAM.
// The FREQUENCY source is NOT touched: set the drive's "command-source bound
// frequency source" (P0-27) so that communication-command binds to the comms
// frequency setpoint, while panel/terminal command fall back to P0-03 (the pot).
// That sidesteps P0-03 not switching from RAM, and never writes EEPROM.
void vfdTakeover() {
  modbusWriteReg(REG_CMD_SRC_RAM, SRC_CMD_COMM);   // command source -> communication
}
void vfdHandback() {
  modbusWriteReg(REG_CMD_SRC_RAM, CSS_OFF_CMD_SRC); // command back to panel (or terminal)
}

// Deferred handback: the drive rejects a command-source change while it's still
// decelerating, so on OFF/STOP we command the stop, flag a pending handback, and
// complete it only once the status word reports stopped (or after a timeout).
// This runs in the loop so it never blocks the web server.
#define STATUS_STOPPED       3      // 0x3000: 1=fwd, 2=rev, 3=stopped (per manual)
#define HANDBACK_TIMEOUT_MS  8000
bool     handbackPending = false;
uint32_t handbackSince   = 0;

void serviceHandback() {
  if (!handbackPending) return;
  bool stopped = false;
  uint16_t st = 0;
  if (modbusReadReg(REG_STATUS, st) && st == STATUS_STOPPED) stopped = true;
  if (stopped || (millis() - handbackSince > HANDBACK_TIMEOUT_MS)) {
    vfdHandback();                                // now the drive will accept it
    handbackPending = false;
    logLine(stopped ? "handback: drive stopped, command source -> panel"
                    : "handback: timeout, command source -> panel (best effort)");
  }
}

void vfdSetFreqHz(float hz) {
  if (hz < 0) hz = 0;
  if (hz > VFD_MAX_FREQ_HZ) hz = VFD_MAX_FREQ_HZ;
  long reg = lroundf(hz / VFD_MAX_FREQ_HZ * 10000.0);
  if (reg < 0) reg = 0; if (reg > 10000) reg = 10000;
  modbusWriteReg(REG_FREQ_SET, (uint16_t)reg);
}

// ============================================================================
// CSS ENGINE  (20 Hz)
// ============================================================================
float computeDiameter() {
  int64_t pos = readPosition();
  float dmm = (float)(pos - refPosition) / (float)SCALE_COUNTS_PER_MM;  // mm travel
  return refDiameter + (float)DIAMETER_DIR * dmm * DIAMETER_MULTIPLIER;
}

void setReference(float dia) {
  refDiameter = dia;
  refPosition = readPosition();
}

void cssLoop() {
  curDiameter = computeDiameter();

  // ---- target RPM from surface speed: N = Vc*1000 / (pi*D) ----
  float d = curDiameter;
  if (d < MIN_DIAMETER_MM) d = MIN_DIAMETER_MM;
  targetRPM = (surfaceSpeed * 1000.0) / (PI * d);
  if (targetRPM > MAX_RPM) targetRPM = MAX_RPM;
  if (targetRPM < 0)       targetRPM = 0;

  float targetFreq = targetRPM * (VFD_BASE_FREQ_HZ / VFD_BASE_RPM);

  // ---- faulted: stay silent on the bus. The link is presumed bad, so we do
  // NOT block the loop hammering dead writes — the VFD's Pd-04 timeout is the
  // real backstop, and staying quiet keeps the web UI responsive for recovery.
  if (vfdFault) { commandedFreq = 0; return; }

  if (cssEnabled) {
    if (!vfdRunning) vfdStart();          // ensure running (retries until ok)
    // slew-limit toward target
    if (targetFreq > commandedFreq)
      commandedFreq = min(targetFreq, commandedFreq + MAX_FREQ_SLEW_HZ);
    else
      commandedFreq = max(targetFreq, commandedFreq - MAX_FREQ_SLEW_HZ);
  } else {
    // ramp down, then stop the drive once at 0
    commandedFreq = max(0.0f, commandedFreq - MAX_FREQ_SLEW_HZ);
    if (commandedFreq <= 0.0f && vfdRunning) vfdStop();
  }

  // Only touch the bus when there is something to command (running, or ramping
  // down). When idle+stopped we stay silent — that way a failing link (each
  // write blocks up to MODBUS_TIMEOUT_MS on no reply) can't starve the web
  // server / serial while you're bench-debugging with CSS off.
  if (cssEnabled || vfdRunning || commandedFreq > 0.0f) {
    vfdSetFreqHz(commandedFreq);          // heartbeat keeps Pd-04 alive while running
  }

  checkDivergence();
}

// Trip a fault if the measured (Hall) RPM strays too far from what the commanded
// frequency should be producing — the signature of a stalled spindle or slipping
// belt. Rides through spin-up (grace period) and only checks steady operation.
uint32_t divBadSince = 0;
void checkDivergence() {
  if (!divEnabled || !cssEnabled || !vfdRunning || vfdFault) { divBadSince = 0; return; }
  if (millis() - runSince < DIV_GRACE_MS)                    { divBadSince = 0; return; }
  float expected = commandedFreq * (VFD_BASE_RPM / VFD_BASE_FREQ_HZ);
  if (expected < DIV_MIN_EXPECTED_RPM)                       { divBadSince = 0; return; }
  float actual = getRPM();
  float tol = expected * (divTolPct / 100.0f);
  if (fabsf(actual - expected) > tol) {
    if (divBadSince == 0) divBadSince = millis();
    else if (millis() - divBadSince > divDelayMs) {
      logLine("DIV FAULT: expected " + String((int)expected) + " rpm, Hall " + String((int)actual) + " rpm");
      vfdStop(); vfdHandback();                 // stop and return control to the panel
      cssEnabled = false; commandedFreq = 0; vfdFault = true;
      divBadSince = 0;
    }
  } else {
    divBadSince = 0;
  }
}

// ============================================================================
// COMMAND INTERFACE  (shared by serial + web + keypad -> executeCommand)
// ============================================================================
String statusString() {
  String s = "CSS ";  s += cssEnabled ? "ON" : "OFF";
  if (vfdFault) s += " [FAULT]";
  s += "  Vc=" + String(surfaceSpeed, 1) + " m/min";
  s += "  refD=" + String(refDiameter, 1);
  s += "  curD=" + String(curDiameter, 1);
  s += "  tRPM=" + String((int)targetRPM);
  s += "  aRPM=" + String((int)getRPM());
  s += "  f=" + String(commandedFreq, 1) + "Hz";
  s += "  run=" + String(vfdRunning ? 1 : 0);
  s += "  mbFail=" + String(modbusFails);
  return s;
}

String helpString() {
  return "Commands: ref XX.X | css XX.X | on | off | stop | reset | zero | mbtest | mbscan | baud N | fmt 8N1 | addr N | link | hallms N | div on/off | divtol N | divdelay N | debug N | logclear | status | help";
}

String executeCommand(String cmd) {
  cmd.trim();
  String lc = cmd; lc.toLowerCase();

  if (lc.startsWith("ref ")) {
    setReference(cmd.substring(4).toFloat());
    return "REF set: " + String(refDiameter, 1) + " mm @ current position";
  }
  if (lc.startsWith("css ")) {
    surfaceSpeed = constrain(cmd.substring(4).toFloat(), VC_MIN, VC_MAX);
    return "Vc = " + String(surfaceSpeed, 1) + " m/min";
  }
  if (lc == "on") {
    if (vfdFault) return "FAULT active — 'reset' first";
    handbackPending = false;             // cancel any pending handback
    vfdTakeover();                       // drive -> comms control (RAM); cssLoop then runs it
    cssEnabled = true;  return "CSS ON";
  }
  if (lc == "off")   { cssEnabled = false; commandedFreq = 0; vfdStop(); handbackPending = true; handbackSince = millis(); return "CSS OFF"; }
  if (lc == "stop")  { cssEnabled = false; commandedFreq = 0; vfdStop(); handbackPending = true; handbackSince = millis(); return "STOP"; }
  if (lc == "reset") { modbusFails = 0; vfdFault = false; vfdFaultReset(); return "RESET (fault cleared)"; }
  if (lc == "mbtest") {
    uint16_t v = 0;
    bool ok = modbusReadReg(REG_FREQ_RB, v);      // FC03 read of 0x1001 (needs no P0 setup)
    return ok ? ("MB OK: reg 0x1001 = " + String(v))
              : "MB FAIL: no valid reply — check A/B, ground, addr, baud/format (set debug 2 for raw)";
  }
  if (lc.startsWith("debug")) {
    String a = cmd.substring(5); a.trim();
    if (a.length()) debugLevel = constrain(a.toInt(), 0, 2);
    return "debug level = " + String(debugLevel) + " (0 off / 1 errors / 2 frames)";
  }
  if (lc == "logclear") { logCount = 0; return "log cleared"; }
  if (lc.startsWith("hallms")) {
    int ms = cmd.substring(6).toInt();
    if (ms >= 1 && ms <= 100) hallMinPeriodUs = (uint32_t)ms * 1000UL;
    return "hall debounce = " + String(hallMinPeriodUs / 1000) + " ms";
  }
  if (lc == "div on")  { divEnabled = true;  return "divergence guard ON"; }
  if (lc == "div off") { divEnabled = false; return "divergence guard OFF"; }
  if (lc.startsWith("divtol")) {
    int p = cmd.substring(6).toInt();
    if (p >= 5 && p <= 90) divTolPct = p;
    return "divergence tolerance = " + String(divTolPct) + "%";
  }
  if (lc.startsWith("divdelay")) {
    long d = cmd.substring(8).toInt();
    if (d >= 200 && d <= 10000) divDelayMs = (uint32_t)d;
    return "divergence delay = " + String(divDelayMs) + " ms";
  }
  if (lc.startsWith("baud")) {
    long b = cmd.substring(4).toInt();
    if (b >= 1200 && b <= 250000) { mbBaud = (uint32_t)b; applySerial(); }
    return "baud = " + String(mbBaud);
  }
  if (lc.startsWith("fmt")) {
    String a = cmd.substring(3); a.trim(); a.toUpperCase();
    for (int i = 0; i < SER_FMT_COUNT; i++)
      if (a == SER_FMTS[i].name) { mbFmtIdx = i; applySerial(); break; }
    return "fmt = " + String(SER_FMTS[mbFmtIdx].name);
  }
  if (lc.startsWith("addr")) {
    int a = cmd.substring(4).toInt();
    if (a >= 1 && a <= 247) vfdSlave = (uint8_t)a;
    return "addr = " + String(vfdSlave);
  }
  if (lc == "link")   { return "link: " + String(mbBaud) + " " + SER_FMTS[mbFmtIdx].name + " addr " + String(vfdSlave); }
  if (lc == "mbscan") { return mbScan(); }
  if (lc == "zero")  { zeroPosition(); return "Position zeroed"; }
  if (lc == "status"){ return statusString(); }
  if (lc == "help")  { return helpString(); }
  return "? unknown — try 'help'";
}

// ============================================================================
// KEYPAD  (VINKA capacitive 4x4 over I2C)
// ============================================================================
// The module presents a 16-bit key state over I2C (one bit per key). The exact
// I2C address and the bit->key order vary between VINKA modules, so we scan for
// the address at boot and use a mapping table that you can verify with
// KEYPAD_DEBUG. Physical layout is assumed:
//        1 2 3 A
//        4 5 6 B
//        7 8 9 C
//        * 0 # D
// If a pressed key prints the wrong char, reorder KEYMAP to match the raw bit.
#define KEYPAD_DEBUG 0            // set 1 to print raw 16-bit value on each press

uint8_t kpAddr = 0x00;           // 0 = not found
// This module reports keys COLUMN-major: bit = column*4 + row (top -> bottom),
// so KEYMAP is indexed by bit number, one column at a time:
//   bits 0-3   = column 0  (1,4,7,*)
//   bits 4-7   = column 1  (2,5,8,0)
//   bits 8-11  = column 2  (3,6,9,#)
//   bits 12-15 = column 3  (A,B,C,D)
// If a key still prints wrong, set KEYPAD_DEBUG 1 and reorder to match the raw bit.
const char KEYMAP[16] = {
  '1','4','7','*',
  '2','5','8','0',
  '3','6','9','#',
  'A','B','C','D'
};

void keypadScan() {
  for (uint8_t a = 0x08; a <= 0x77; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      kpAddr = a;
      Serial.printf("[keypad] found device at 0x%02X\n", a);
      return;
    }
  }
  Serial.println("[keypad] NOT FOUND — check wiring / address");
}

uint16_t keypadReadRaw() {
  if (!kpAddr) return 0;
  uint16_t v = 0;
  int n = Wire.requestFrom((int)kpAddr, 2);
  if (n >= 2) {
    uint8_t lo = Wire.read();
    uint8_t hi = Wire.read();
    v = ((uint16_t)hi << 8) | lo;      // if your module is MSB-first, swap here
  } else {
    while (Wire.available()) Wire.read();
  }
  return v;
}

char keypadDecode(uint16_t raw) {
  if (raw == 0) return 0;
  for (int i = 0; i < 16; i++) if (raw & (1 << i)) return KEYMAP[i];  // first bit set
  return 0;
}

// ---- key actions ----
void handleKey(char k) {
  if (mode == MODE_NORMAL) {
    switch (k) {
      case 'A': mode = MODE_SETVC;  setRefBuf = ""; break;  // type exact Vc
      case 'B': mode = MODE_SETREF; setRefBuf = ""; break;  // type exact ref diameter
      case 'C': executeCommand("stop"); break;
      case 'D': executeCommand("zero"); break;              // zero cross-slide position
      case '#': executeCommand("on");  break;               // CSS ON
      case '*': executeCommand("off"); break;               // CSS OFF
      case '5': surfaceSpeed = constrain(surfaceSpeed + 5, VC_MIN, VC_MAX); break; // +5 (hold)
      case '0': surfaceSpeed = constrain(surfaceSpeed - 5, VC_MIN, VC_MAX); break; // -5 (hold)
      default: break;
    }
  } else { // numeric entry — MODE_SETVC or MODE_SETREF (same keys, confirm differs)
    if (k >= '0' && k <= '9') {
      if (setRefBuf.length() < 7) setRefBuf += k;
    } else if (k == '*') {                       // decimal point
      if (setRefBuf.indexOf('.') < 0) setRefBuf += (setRefBuf.length() ? "." : "0.");
    } else if (k == '#') {                        // confirm
      if (setRefBuf.length()) {
        if (mode == MODE_SETVC) surfaceSpeed = constrain(setRefBuf.toFloat(), VC_MIN, VC_MAX);
        else                    setReference(setRefBuf.toFloat());
      }
      mode = MODE_NORMAL; setRefBuf = "";
    } else if (k == 'D') {                         // cancel
      mode = MODE_NORMAL; setRefBuf = "";
    } else if (k == 'C') {                         // backspace
      if (setRefBuf.length()) setRefBuf.remove(setRefBuf.length() - 1);
    }
  }
}

char     kpLast = 0;
uint32_t kpLastRepeat = 0;

void keypadTask() {
  uint16_t raw = keypadReadRaw();
  char k = keypadDecode(raw);
  uint32_t now = millis();

  if (k && k != kpLast) {                 // new press
#if KEYPAD_DEBUG
    Serial.printf("[keypad] raw=0x%04X key=%c\n", raw, k);
#endif
    kpLast = k; kpLastRepeat = now;
    handleKey(k);
  } else if (k && k == kpLast) {          // held
    if (mode == MODE_NORMAL && (k == '0' || k == '5') && (now - kpLastRepeat > KEY_REPEAT_MS)) {
      kpLastRepeat = now; handleKey(k);
    }
  } else if (!k) {                        // released
    kpLast = 0;
  }
}

// ============================================================================
// DISPLAY  (landscape 320x240, partial updates to avoid flicker)
// ============================================================================
Field fState, fVc, fRefD, fCurD, fTgtRpm, fActRpm, fFreq, fMb, fIp, fEntry;

void dLabel(int x, int y, const char* t) {
  tft.setTextSize(1);
  tft.setTextColor(C_LABEL, C_BG);
  tft.setCursor(x, y);
  tft.print(t);
}

void drawStaticUI() {
  tft.fillScreen(C_BG);

  // header
  tft.fillRect(0, 0, 320, 24, ILI9341_NAVY);
  tft.setTextColor(C_ACCENT, ILI9341_NAVY);
  tft.setTextSize(2);
  tft.setCursor(6, 4); tft.print("CSS v0.8");

  // labels
  dLabel(6,   34, "STATE");        fState  = {6,   44, ""};
  dLabel(6,   74, "Vc  m/min");    fVc     = {6,   84, ""};
  dLabel(6,  114, "REF D  mm");    fRefD   = {6,  124, ""};
  dLabel(6,  154, "CUR D  mm");    fCurD   = {6,  164, ""};

  dLabel(170, 34, "TARGET RPM");   fTgtRpm = {170, 44, ""};
  dLabel(170, 74, "ACTUAL RPM");   fActRpm = {170, 84, ""};
  dLabel(170,114, "FREQ  Hz");     fFreq   = {170,124, ""};
  dLabel(170,154, "MODBUS");       fMb     = {170,164, ""};

  dLabel(6,  204, "IP");           fIp     = {6,  214, ""};
  fEntry = {150, 210, ""};         // SET REF entry area (right/bottom)
}

void dValue(Field& f, const String& s, uint16_t color, int size, int clearW) {
  if (s == f.last) return;
  tft.fillRect(f.x, f.y, clearW, size * 8 + 2, C_BG);
  tft.setTextSize(size);
  tft.setTextColor(color, C_BG);
  tft.setCursor(f.x, f.y);
  tft.print(s);
  f.last = s;
}

void updateDisplay() {
  // state
  String st = cssEnabled ? "CSS ON" : "CSS OFF";
  uint16_t sc = cssEnabled ? C_OK : C_LABEL;
  if (vfdFault) { st = "FAULT"; sc = C_BAD; }
  dValue(fState, st, sc, 2, 150);

  dValue(fVc,    String(surfaceSpeed, 1),      C_VALUE, 3, 150);
  dValue(fRefD,  String(refDiameter, 1),       C_VALUE, 2, 150);
  dValue(fCurD,  String(curDiameter, 1),       C_ACCENT,3, 150);

  dValue(fTgtRpm,String((int)targetRPM),       C_VALUE, 3, 150);
  dValue(fActRpm,String((int)getRPM()),        C_ACCENT,3, 150);
  dValue(fFreq,  String(commandedFreq, 1),     C_VALUE, 2, 150);

  // Binary link status only — no running count, so the field is stable and
  // only redraws on an actual OK<->FAIL transition. (The changing count was
  // redrawing every failed cycle and flickering.) The consecutive-failure
  // counter is still kept internally to trip the fault after MODBUS_MAX_FAILS.
  String mb = (modbusFails == 0) ? "OK" : "FAIL";
  dValue(fMb, mb, (modbusFails == 0) ? C_OK : C_BAD, 2, 150);

  dValue(fIp, WiFi.isConnected() ? WiFi.localIP().toString() : String("no wifi"),
         C_LABEL, 1, 140);

  // numeric entry line
  if (mode == MODE_SETVC) {
    dValue(fEntry, "SET Vc: " + setRefBuf + "_", C_WARN, 2, 170);
  } else if (mode == MODE_SETREF) {
    dValue(fEntry, "SET D: " + setRefBuf + "_", C_WARN, 2, 170);
  } else {
    dValue(fEntry, "", C_BG, 2, 170);
  }
}

// ============================================================================
// WEB DASHBOARD
// ============================================================================
String jsonStatus() {
  String s = "{";
  s += "\"vc\":"    + String(surfaceSpeed, 1) + ",";
  s += "\"refD\":"  + String(refDiameter, 1) + ",";
  s += "\"curD\":"  + String(curDiameter, 1) + ",";
  s += "\"tRPM\":"  + String((int)targetRPM) + ",";
  s += "\"aRPM\":"  + String((int)getRPM()) + ",";
  s += "\"freq\":"  + String(commandedFreq, 1) + ",";
  s += "\"css\":"   + String(cssEnabled ? 1 : 0) + ",";
  s += "\"run\":"   + String(vfdRunning ? 1 : 0) + ",";
  s += "\"fault\":" + String(vfdFault ? 1 : 0) + ",";
  s += "\"mbFail\":"+ String(modbusFails) + ",";
  s += "\"pos\":"   + String((long)readPosition()) + ",";
  s += "\"dbg\":"   + String(debugLevel) + ",";
  s += "\"rssi\":"  + String(WiFi.isConnected() ? WiFi.RSSI() : 0) + ",";
  s += "\"heap\":"  + String((unsigned long)ESP.getFreeHeap()) + ",";
  s += "\"up\":"    + String(millis() / 1000) + ",";
  s += "\"baud\":"  + String(mbBaud) + ",";
  s += "\"fmt\":\"" + String(SER_FMTS[mbFmtIdx].name) + "\",";
  s += "\"addr\":"  + String(vfdSlave) + ",";
  s += "\"hallms\":"+ String(hallMinPeriodUs / 1000) + ",";
  s += "\"div\":"   + String(divEnabled ? 1 : 0) + ",";
  s += "\"divtol\":"+ String(divTolPct) + ",";
  s += "\"mode\":\""+ String(mode == MODE_SETVC ? "SETVC" : (mode == MODE_SETREF ? "SETREF" : "NORMAL")) + "\"";
  s += "}";
  return s;
}

const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>CSS v0.8</title>
<style>
 :root{--bg:#111;--card:#1c1c1c;--mut:#888;--cy:#0ff;--gr:#0f0;--rd:#f55;--yl:#ff0}
 *{box-sizing:border-box}
 body{font-family:system-ui,Arial;margin:0;background:var(--bg);color:#eee}
 h1{font-size:16px;background:#123;margin:0;padding:10px}
 h2{font-size:12px;color:var(--mut);margin:10px 12px 2px;letter-spacing:.5px}
 .meta{font-size:11px;color:var(--mut);padding:4px 12px;word-break:break-all}
 .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px;padding:8px 12px}
 .card{background:var(--card);border-radius:8px;padding:8px}
 .k{color:var(--mut);font-size:11px}.v{font-size:19px}
 .cy{color:var(--cy)}.gr{color:var(--gr)}.rd{color:var(--rd)}.yl{color:var(--yl)}
 .row{display:flex;flex-wrap:wrap;gap:6px;padding:4px 12px}
 button{flex:1;min-width:60px;padding:12px;font-size:14px;border:0;border-radius:8px;background:#2a2a2a;color:#eee}
 button:active{background:#444}
 .on{background:#063}.off{background:#630}.stop{background:#900}
 .sel{outline:2px solid var(--cy)}
 input{width:78px;padding:9px;font-size:15px;border-radius:8px;border:1px solid #444;background:#222;color:#eee}
 #log{margin:4px 12px 16px;background:#000;border:1px solid #333;border-radius:8px;height:240px;
      overflow:auto;font-family:ui-monospace,Menlo,Consolas,monospace;font-size:11px;
      line-height:1.35;padding:6px;white-space:pre;color:#bfe}
 #log .err{color:var(--rd)}
</style></head><body>
<h1>CSS Controller v0.8</h1>
<div class=meta id=meta>connecting…</div>
<div class=grid>
 <div class=card><div class=k>STATE</div><div class=v id=state>-</div></div>
 <div class=card><div class=k>MODBUS</div><div class=v id=mb>-</div></div>
 <div class=card><div class=k>FAILS</div><div class=v id=mbf>-</div></div>
 <div class=card><div class=k>Vc m/min</div><div class="v cy" id=vc>-</div></div>
 <div class=card><div class=k>FREQ Hz</div><div class=v id=freq>-</div></div>
 <div class=card><div class=k>POS cnt</div><div class="v cy" id=pos>-</div></div>
 <div class=card><div class=k>REF D</div><div class=v id=refD>-</div></div>
 <div class=card><div class=k>CUR D</div><div class="v cy" id=curD>-</div></div>
 <div class=card><div class=k>MODE</div><div class=v id=mode>-</div></div>
 <div class=card><div class=k>TGT RPM</div><div class=v id=tRPM>-</div></div>
 <div class=card><div class=k>ACT RPM</div><div class="v cy" id=aRPM>-</div></div>
 <div class=card><div class=k>RUN</div><div class=v id=run>-</div></div>
</div>

<h2>CONTROL</h2>
<div class=row>
 <button class=on onclick="cmd('on')">CSS ON</button>
 <button class=off onclick="cmd('off')">CSS OFF</button>
 <button class=stop onclick="cmd('stop')">STOP</button>
</div>
<div class=row>
 <button onclick="cmd('css '+(S.vc-1))">Vc-1</button>
 <button onclick="cmd('css '+(S.vc+1))">Vc+1</button>
 <button onclick="cmd('css '+(S.vc-5))">Vc-5</button>
 <button onclick="cmd('css '+(S.vc+5))">Vc+5</button>
</div>
<div class=row>
 <input id=refin placeholder="dia">
 <button onclick="cmd('ref '+$('refin').value)">SET REF</button>
 <button onclick="cmd('zero')">ZERO</button>
 <button onclick="cmd('reset')">RESET</button>
</div>

<h2>LINK</h2>
<div class=row>
 <button id=b4800 onclick="cmd('baud 4800')">4800</button>
 <button id=b9600 onclick="cmd('baud 9600')">9600</button>
 <button id=b19200 onclick="cmd('baud 19200')">19200</button>
 <button id=b38400 onclick="cmd('baud 38400')">38400</button>
 <button id=b57600 onclick="cmd('baud 57600')">57600</button>
 <button id=b115200 onclick="cmd('baud 115200')">115200</button>
</div>
<div class=row>
 <button id=f8N1 onclick="cmd('fmt 8N1')">8N1</button>
 <button id=f8N2 onclick="cmd('fmt 8N2')">8N2</button>
 <button id=f8E1 onclick="cmd('fmt 8E1')">8E1</button>
 <button id=f8O1 onclick="cmd('fmt 8O1')">8O1</button>
 <input id=addrin placeholder="addr">
 <button onclick="cmd('addr '+$('addrin').value)">SET</button>
</div>
<div class=row>
 <button onclick="cmd('mbscan')">MB SCAN (auto)</button>
 <button onclick="cmd('mbtest')">MB TEST</button>
</div>

<h2>TUNE</h2>
<div class=row>
 <input id=hallin placeholder="hall ms">
 <button onclick="cmd('hallms '+$('hallin').value)">SET DEBOUNCE</button>
 <input id=divin placeholder="div %">
 <button onclick="cmd('divtol '+$('divin').value)">SET TOL</button>
</div>
<div class=row>
 <button id=dvon onclick="cmd('div on')">DIV GUARD ON</button>
 <button id=dvoff onclick="cmd('div off')">DIV GUARD OFF</button>
</div>

<h2>485 DEBUG</h2>
<div class=row>
 <button id=d0 onclick="cmd('debug 0')">OFF</button>
 <button id=d1 onclick="cmd('debug 1')">ERRORS</button>
 <button id=d2 onclick="cmd('debug 2')">FRAMES</button>
 <button id=pz onclick="togglePause()">Pause</button>
 <button onclick="clr()">Clear</button>
</div>
<div id=log></div>

<script>
let S={vc:0}, lastSeq=0, paused=false;
let dupText=null, dupCount=1, dupEl=null;
const $=id=>document.getElementById(id);
function cmd(c){fetch('/cmd?c='+encodeURIComponent(c)).then(status).catch(()=>{});}
function togglePause(){paused=!paused;$('pz').textContent=paused?'Resume':'Pause';$('pz').className=paused?'sel':'';}
function clr(){$('log').textContent='';dupText=null;dupEl=null;dupCount=1;fetch('/cmd?c=logclear').catch(()=>{});}
function status(){fetch('/status').then(r=>r.json()).then(d=>{
 S=d;
 $('state').textContent=d.fault?'FAULT':(d.css?'CSS ON':'CSS OFF');
 $('state').className='v '+(d.fault?'rd':(d.css?'gr':''));
 $('mb').textContent=d.mbFail?'FAIL':'OK';$('mb').className='v '+(d.mbFail?'rd':'gr');
 $('mbf').textContent=d.mbFail;
 $('vc').textContent=(+d.vc).toFixed(1);
 $('freq').textContent=(+d.freq).toFixed(1);
 $('pos').textContent=d.pos;
 $('refD').textContent=(+d.refD).toFixed(1);
 $('curD').textContent=(+d.curD).toFixed(1);
 $('mode').textContent=d.mode;
 $('tRPM').textContent=d.tRPM;
 $('aRPM').textContent=d.aRPM;
 $('run').textContent=d.run?'YES':'no';
 $('meta').textContent='IP '+location.hostname+'  '+d.baud+' '+d.fmt+' addr'+d.addr+'  hall '+d.hallms+'ms  div '+(d.div?d.divtol+'%':'off')+'  RSSI '+d.rssi+'dBm  up '+d.up+'s';
 for(const i of [0,1,2])$('d'+i).className=(d.dbg==i?'sel':'');
 $('dvon').className=(d.div?'sel':'');
 $('dvoff').className=(d.div?'':'sel');
 for(const b of [4800,9600,19200,38400,57600,115200])$('b'+b).className=(d.baud==b?'sel':'');
 for(const f of ['8N1','8N2','8E1','8O1'])$('f'+f).className=(d.fmt==f?'sel':'');
}).catch(()=>{});}
function pollLog(){
 if(paused)return;
 fetch('/log?since='+lastSeq).then(r=>r.json()).then(d=>{
  lastSeq=d.seq;
  if(!d.lines||!d.lines.length)return;
  const L=$('log');
  for(const ln of d.lines){
   if(ln===dupText && dupEl){                 // same as previous line: bump counter
    dupCount++;
    dupEl.textContent=ln+'   (x'+dupCount+')';
   } else {                                    // new distinct line
    const e=document.createElement('div');
    if(ln.indexOf('ERR')>=0)e.className='err';
    e.textContent=ln;
    L.appendChild(e);
    dupEl=e; dupText=ln; dupCount=1;
   }
  }
  while(L.childElementCount>400)L.removeChild(L.firstChild);
  if(!L.contains(dupEl)){dupEl=null;dupText=null;dupCount=1;}  // anchor scrolled off
  L.scrollTop=L.scrollHeight;
 }).catch(()=>{});
}
setInterval(status,300);
setInterval(pollLog,300);
status();
</script></body></html>
)HTML";

void handleRoot()   { server.send_P(200, "text/html", PAGE); }
void handleStatus() { server.send(200, "application/json", jsonStatus()); }
void handleCmd()    {
  String c = server.hasArg("c") ? server.arg("c") : "";
  String r = executeCommand(c);
  server.send(200, "text/plain", r);
}
void handleLog() {
  // Return log lines with absolute index > 'since'. The ring only holds the
  // last LOG_CAP, so if the client is further behind it resyncs to the oldest.
  long since  = server.hasArg("since") ? server.arg("since").toInt() : 0;
  long oldest = (long)logSeq - logCount;
  long from   = since < oldest ? oldest : since;
  if (from < 0) from = 0;
  String out = "{\"seq\":" + String((unsigned long)logSeq) + ",\"lines\":[";
  for (long i = from; i < (long)logSeq; i++) {
    if (i > from) out += ",";
    out += "\"" + jsonEscape(logBuf[i % LOG_CAP]) + "\"";
  }
  out += "]}";
  server.send(200, "application/json", out);
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== CSS Controller v0.8 ===");

  // RS485 direction
  pinMode(PIN_RS485_DE, OUTPUT);
  digitalWrite(PIN_RS485_DE, LOW);          // default: receive
  applySerial();                            // opens Serial2 at current mbBaud / format

  // Backlight on
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  // Quadrature + Hall
  setupPCNT();
  pinMode(PIN_HALL, INPUT);                 // existing display's pull-up holds it high
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);

  // Keypad I2C
  pinMode(PIN_KP_INT, INPUT_PULLUP);
  Wire.begin(PIN_KP_SDA, PIN_KP_SCL);
  Wire.setClock(100000);
  keypadScan();

  // Front-panel momentary RESET pushbutton (to GND, internal pull-up)
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);

  // Display (HSPI @ 40 MHz, landscape)
  hspi.begin(PIN_TFT_SCK, -1, PIN_TFT_MOSI);   // SCK, MISO(unused), MOSI
  tft.begin(40000000);
  tft.setRotation(1);                          // 320x240 landscape
  drawStaticUI();

  // WiFi (STA, non-blocking-ish with timeout)
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 8000) {
    delay(200); Serial.print(".");
  }
  if (WiFi.isConnected()) {
    Serial.printf("\n[wifi] connected  http://%s/\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\n[wifi] not connected (running without dashboard)");
  }

  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/cmd", handleCmd);
  server.on("/log", handleLog);
  server.begin();

  Serial.println(helpString());
  Serial.println("Ready.");
}

// ============================================================================
// LOOP
// ============================================================================
uint32_t tCss = 0, tDisp = 0, tKp = 0, tHb = 0;
String serialBuf = "";

void serialTask() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBuf.length()) {
        Serial.println(executeCommand(serialBuf));
        serialBuf = "";
      }
    } else if (serialBuf.length() < 40) {
      serialBuf += c;
    }
  }
}

// Front-panel momentary RESET button — debounced, active low. Fires the
// "reset" command: clears the latched Modbus fault + failure counter and sends
// the VFD a fault-reset.
bool     rbStable   = HIGH;
bool     rbReading  = HIGH;
uint32_t rbLastEdge = 0;
#define  RB_DEBOUNCE_MS 30

void resetButtonTask() {
  bool r = digitalRead(PIN_RESET_BTN);
  uint32_t t = millis();
  if (r != rbReading) { rbReading = r; rbLastEdge = t; }        // input moved: restart timer
  if ((t - rbLastEdge) > RB_DEBOUNCE_MS && r != rbStable) {     // settled to a new state
    rbStable = r;
    if (rbStable == LOW) executeCommand("reset");               // act on the press edge
  }
}

void loop() {
  uint32_t now = millis();

  server.handleClient();
  serialTask();
  resetButtonTask();

  if (now - tKp >= KEYPAD_PERIOD_MS)   { tKp = now;   keypadTask(); }
  if (now - tCss >= CSS_PERIOD_MS)     { tCss = now;  cssLoop(); }
  if (now - tHb >= 150)                { tHb = now;   serviceHandback(); }
  server.handleClient();                 // again after cssLoop, so a slow/failed
                                         // Modbus cycle can't starve the browser
  if (now - tDisp >= DISP_PERIOD_MS)   { tDisp = now; updateDisplay(); }
}
