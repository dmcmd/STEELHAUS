/**
 * Heat Treat Oven Controller - ESP32 Firmware
 * 
 * Handles:
 * - MAX31856 thermocouple reading via SPI
 * - SSR PWM output (both hot legs of 240V switched in lockstep)
 * - Contactor relay control (master AC disconnect before SSRs)
 * - Serial communication with RPi (JSON over UART2, GPIO16/17)
 * - Hardware watchdog: cuts ALL outputs if Pi goes silent for 5 seconds
 * 
 * SAFETY ARCHITECTURE:
 *   Power flow: 240V panel -> CONTACTOR -> SSRs (MOSFET on GPIO27) -> heating element
 * 
 *   The contactor is a master disconnect. When de-energized it physically
 *   breaks both hot legs upstream of the SSRs, regardless of SSR state.
 *   The SSRs do the PID duty-cycle switching within that safe envelope.
 * 
 *   Contactor energizes ONLY when the Pi explicitly commands it ON.
 *   It de-energizes immediately on: E-stop, watchdog timeout, thermocouple
 *   fault, over-temperature, or any serial communication loss.
 * 
 *   On shutdown the SSRs are opened first, then the contactor drops after
 *   a 20ms delay so the contactor is never switching under full load.
 * 
 * PIN ASSIGNMENTS:
 *   MAX31856 (SPI):
 *     CS   -> GPIO 5
 *     SCK  -> GPIO 18
 *     MOSI -> GPIO 23
 *     MISO -> GPIO 19
 *     DRDY -> GPIO 4
 *
 *   SSR output (MOSFET gate)  -> GPIO 27
 *   Contactor relay signal    -> GPIO 33  (HIGH = relay energized = contactor closed)
 *   IRLZ44N gate (fan/DC aux) -> GPIO 27
 *   Status LED (built-in)     -> GPIO 2
 *   Emergency Stop button     -> GPIO 34  (wire NC button between pin and GND)
 */

#include <SPI.h>
#include <Adafruit_MAX31856.h>
#include <ArduinoJson.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
#define MAX31856_CS_PIN    5
#define MAX31856_DRDY_PIN  4
#define SSR_PIN            27
#define CONTACTOR_PIN      33   // HIGH = relay energized = contactor closed = AC live
#define MOSFET_PIN         27
#define STATUS_LED_PIN     2
#define ESTOP_PIN          34

// UART2 pins for RPi communication (avoids USB DTR reset issue)
#define UART_RX_PIN        16
#define UART_TX_PIN        17

#define DOOR_PIN           35   // Door switch NC contact — LOW = door closed, HIGH = door open

// ── Safety limits ─────────────────────────────────────────────────────────────
#define WATCHDOG_TIMEOUT_MS  5000
#define MAX_SAFE_TEMP_C      1350.0f
#define MIN_TEMP_C           -50.0f

// ── Timing ────────────────────────────────────────────────────────────────────
#define TEMP_READ_INTERVAL_MS  250
#define SERIAL_REPORT_MS       250
#define SSR_CYCLE_MS           1000   // Soft-PWM cycle: 1 second

// ── Hardware ──────────────────────────────────────────────────────────────────
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(MAX31856_CS_PIN);

// ── State ─────────────────────────────────────────────────────────────────────
float    currentTemp      = 0.0f;
float    coldJunctionTemp = 0.0f;
uint8_t  ssrDuty          = 0;      // 0-100% PWM duty cycle
uint8_t  mosfetDuty       = 0;
bool     contactorOn      = false;  // true = relay energized, AC power live at SSRs
bool     emergencyStop    = false;
bool     watchdogTripped  = false;
bool     doorOpen         = false;
uint32_t lastCommandTime  = 0;
uint32_t lastTempRead     = 0;
uint32_t lastReport       = 0;
uint32_t cycleStartTime   = 0;
uint8_t  faultCode        = 0;

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);   // USB — for Arduino IDE flashing only
  Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  pinMode(SSR_PIN,        OUTPUT);
  pinMode(CONTACTOR_PIN,  OUTPUT);
  pinMode(MOSFET_PIN,     OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ESTOP_PIN,      INPUT);  // External 10k pull-down to GND; 3V3→button→pin
  pinMode(DOOR_PIN,       INPUT);  // External 10k pull-down to GND; 3V3→NC contact→pin

  cutAllOutputs();  // Safe state on boot

  if (!maxthermo.begin()) {
    // Rapid flash = MAX31856 not found — check wiring
    while (1) {
      digitalWrite(STATUS_LED_PIN, HIGH); delay(100);
      digitalWrite(STATUS_LED_PIN, LOW);  delay(100);
    }
  }

  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_60HZ);

  // Wait for MAX31856 to complete first conversion cycle before reading
  delay(200);

  lastCommandTime = millis();
  cycleStartTime  = millis();
  lastTempRead    = millis(); // Delay first read by TEMP_READ_INTERVAL_MS

  // Single slow blink = ready
  digitalWrite(STATUS_LED_PIN, HIGH); delay(500);
  digitalWrite(STATUS_LED_PIN, LOW);
}

// ── Main loop ─────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // Physical E-stop button check (normally-closed, press = opens = LOW)
  if (!digitalRead(ESTOP_PIN)) {
    emergencyStop = true;
    cutAllOutputs();
  }

  // Door switch check (NC contact — door open = closes = HIGH)
  doorOpen = digitalRead(DOOR_PIN);
  if (doorOpen) {
    cutAllOutputs();
  }

  // Watchdog: Pi must send a command at least every 5 seconds
  if ((now - lastCommandTime) > WATCHDOG_TIMEOUT_MS) {
    watchdogTripped = true;
    cutAllOutputs();
  } else {
    watchdogTripped = false;
  }

  // Temperature reading
  if ((now - lastTempRead) >= TEMP_READ_INTERVAL_MS) {
    lastTempRead = now;
    readTemperature();
  }

  // SSR output — only when contactor is on and system is healthy
  if (!emergencyStop && !watchdogTripped && contactorOn) {
    applySSROutput(now);
  } else {
    // SSRs must be off whenever contactor is off or any fault is active
    digitalWrite(SSR_PIN,  LOW);
  }

  // Receive commands from Pi
  if (Serial2.available()) {
    String line = Serial2.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) parseCommand(line);
  }

  // Send status report to Pi
  if ((now - lastReport) >= SERIAL_REPORT_MS) {
    lastReport = now;
    sendReport();
  }

  updateStatusLED(now);
}

// ── Temperature reading ───────────────────────────────────────────────────────
void readTemperature() {
  faultCode = maxthermo.readFault();
  if (faultCode == 0) {
    currentTemp      = maxthermo.readThermocoupleTemperature();
    coldJunctionTemp = maxthermo.readCJTemperature();
    if (currentTemp > MAX_SAFE_TEMP_C || currentTemp < MIN_TEMP_C) {
      emergencyStop = true;
      cutAllOutputs();
    }
  } else {
    // Any thermocouple fault = kill everything immediately
    cutAllOutputs();
  }
}

// ── SSR soft-PWM ──────────────────────────────────────────────────────────────
void applySSROutput(uint32_t now) {
  uint32_t cyclePos = (now - cycleStartTime) % SSR_CYCLE_MS;
  bool ssrOn;

  if (ssrDuty == 0) {
    ssrOn = false;
  } else if (ssrDuty >= 100) {
    ssrOn = true;
  } else {
    uint32_t onTime = (SSR_CYCLE_MS * (uint32_t)ssrDuty) / 100;
    ssrOn = (cyclePos < onTime);
  }

  // Back-to-back writes — both legs switch together
  digitalWrite(SSR_PIN,  ssrOn ? HIGH : LOW);
}

// ── Parse command from Pi ─────────────────────────────────────────────────────
void parseCommand(const String& line) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, line) != DeserializationError::Ok) return;

  lastCommandTime = millis();  // Any valid JSON resets the watchdog

  const char* cmd = doc["cmd"];
  if (!cmd) return;

  if (strcmp(cmd, "set") == 0) {
    if (!emergencyStop) {
      ssrDuty    = constrain((int)doc["duty"],   0, 100);
      // MOSFET_PIN shares GPIO27 with SSR_PIN — do not call analogWrite here
      // as it would conflict with the digitalWrite in applySSROutput
    }

  } else if (strcmp(cmd, "contactor") == 0) {
    bool requested = (bool)doc["on"];
    if (requested && (emergencyStop || watchdogTripped || faultCode != 0 || doorOpen)) {
      // Refuse to energize contactor while any fault is active or door is open
      contactorOn = false;
      digitalWrite(CONTACTOR_PIN, LOW);
    } else {
      contactorOn = requested;
      if (!contactorOn) {
        // Turning off: SSRs first, then contactor
        ssrDuty = 0;
        digitalWrite(SSR_PIN,  LOW);
        delay(20);  // Let SSRs open before dropping contactor
      }
      digitalWrite(CONTACTOR_PIN, contactorOn ? HIGH : LOW);
    }

  } else if (strcmp(cmd, "estop") == 0) {
    emergencyStop = true;
    cutAllOutputs();

  } else if (strcmp(cmd, "reset") == 0) {
    // Clears software fault flags. Contactor stays off — Pi must explicitly
    // re-enable it so the operator takes a deliberate action to restore power.
    emergencyStop   = false;
    watchdogTripped = false;
    lastCommandTime = millis();
    cutAllOutputs();

  } else if (strcmp(cmd, "ping") == 0) {
    // Watchdog keepalive only — no output changes
  }
}

// ── Send status report to Pi ──────────────────────────────────────────────────
void sendReport() {
  StaticJsonDocument<256> doc;
  doc["temp"]      = round(currentTemp * 10.0f) / 10.0f;
  doc["cj"]        = round(coldJunctionTemp * 10.0f) / 10.0f;
  doc["ssr_duty"]  = ssrDuty;
  doc["contactor"] = contactorOn;
  doc["mosfet"]    = mosfetDuty;
  doc["fault"]     = faultCode;
  doc["estop"]     = emergencyStop;
  doc["wdog"]      = watchdogTripped;
  doc["door"]      = doorOpen;
  serializeJson(doc, Serial2);
  Serial2.println();
}

// ── Cut all outputs ───────────────────────────────────────────────────────────
// Called from every fault path. SSRs open first, contactor drops after a
// short delay so the contactor contacts never interrupt current directly.
void cutAllOutputs() {
  ssrDuty    = 0;
  mosfetDuty = 0;
  digitalWrite(SSR_PIN,       LOW);
  digitalWrite(MOSFET_PIN,    LOW);
  delay(20);                          // SSRs open, now safe to drop contactor
  contactorOn = false;
  digitalWrite(CONTACTOR_PIN, LOW);
}

// ── Status LED ────────────────────────────────────────────────────────────────
void updateStatusLED(uint32_t now) {
  static uint32_t lastBlink = 0;
  static bool     ledState  = false;

  uint32_t blinkRate;
  if (emergencyStop || watchdogTripped || faultCode != 0 || doorOpen) {
    blinkRate = 100;   // Very fast = fault / e-stop
  } else if (contactorOn && ssrDuty > 0) {
    blinkRate = 250;   // Fast = actively heating
  } else if (contactorOn) {
    blinkRate = 500;   // Medium = contactor on, SSRs idle (soak hold, etc.)
  } else {
    blinkRate = 1000;  // Slow = standby
  }

  if ((now - lastBlink) >= blinkRate) {
    lastBlink = now;
    ledState  = !ledState;
    digitalWrite(STATUS_LED_PIN, ledState);
  }
}
