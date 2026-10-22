// =====================================================================
//  Smart Motor Monitor - FINAL FIRMWARE
//  ESP32 + MPU6050 + INA219 + LM75A + DS18B20 + A3144 + I2C OLED
//  + relay + buzzer, linked to Firebase Realtime Database.
//
//  Core 1 (loop):  sensors, fault engine, relay, buzzer, OLED
//                  -> never waits for the network
//  Core 0 (task):  Wi-Fi + Firebase (telemetry, commands, config, fault log)
//  Only core 1 touches the I2C bus.
// =====================================================================

#include <Arduino.h>
#include <time.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

// ---------------------------------------------------------------------
//  1. SETTINGS
// ---------------------------------------------------------------------
#define SIMULATE 0     // 0 = real sensors (hardware build), 1 = fake sensors

#define WIFI_SSID    ""
#define WIFI_PASS    ""
#define API_KEY      ""
#define DATABASE_URL "https://motor-monitor-d7663-default-rtdb.asia-southeast1.firebasedatabase.app"
#define DEVICE_EMAIL "device@motor.test"
#define DEVICE_PASS  ""

// Telegram alerts (see setup guide). Leave TG_TOKEN empty to disable alerts.
#define TG_TOKEN       ""
#define TG_CHAT_ID     ""
#define DASHBOARD_URL  "https://motor-monitor-d7663.web.app"

const String BASE = "/devices/motor1";
String dbPath(const char* suffix) { String path = BASE; path.concat(suffix); return path; }

// Pins
const int SDA_PIN = 21, SCL_PIN = 22;                 // I2C: OLED, MPU6050, INA219, LM75A
const int DS_PIN = 4, HALL_PIN = 18, RELAY_PIN = 17, BUZZER_PIN = 16;
const bool RELAY_ACTIVE_LOW = true;                   // check your relay module

const uint8_t ADDR_OLED = 0x3C, ADDR_INA = 0x40, ADDR_LM75 = 0x48;
const uint8_t ADDR_MPU = 0x68, ADDR_MPU_ALT = 0x69;

// Protection settings (not editable from the app)
const unsigned long BLANK_MS = 1000;                  // ignore current right after start
const int   CONFIRM_SAMPLES = 3;                      // samples (200 ms each) to confirm a trip
const float V_LOW = 8.0;                              // under-voltage trip
const float STALL_RPM = 100;                          // below this with current up = stall
const unsigned long STALL_MS = 2000;
const bool  STALL_CHECK = true;                       // needs the Hall sensor + magnet working
const float MAX_RPM = 800;                            // a bit ABOVE your motor's top speed (300-400 RPM); faster pulses = noise
const bool  STOP_ON_CLOUD_LOSS = false;               // true = trip if cloud silent too long
const unsigned long CLOUD_TIMEOUT_MS = 60000;
const unsigned long SENSOR_TIMEOUT_MS = 3000;         // a sensor may drop out this long before it counts as failed
const int HISTORY_INTERVAL_S = 15;                    // one history sample every 15 s
const int HISTORY_SLOTS = 480;                        // ring buffer: 480 x 15 s = 2 hours

// ---------------------------------------------------------------------
//  2. GLOBAL STATE
// ---------------------------------------------------------------------
Adafruit_SSD1306 display(128, 64, &Wire, -1);          // I2C OLED, no reset pin
Adafruit_INA219 ina;
Adafruit_MPU6050 mpu;
OneWire oneWire(DS_PIN);
DallasTemperature ds(&oneWire);
bool hasOLED = false, hasINA = false, hasMPU = false;

// Readings. NAN means "sensor failure".
#if SIMULATE
float current = 0.2, voltage = 12.0, motorTemp = 30, ambientTemp = 29, vibration = 0.1;
#else
float current = NAN, voltage = NAN, motorTemp = NAN, ambientTemp = NAN, vibration = NAN;
#endif
float rpm = 0;
unsigned long okINA = 0, okAmb = 0, okDS = 0, okVib = 0;   // last time each sensor read OK

enum State { STOPPED, STARTING, RUNNING, WARNING, TRIPPED };
const char* STATE_NAMES[] = {"STOPPED", "STARTING", "RUNNING", "WARNING", "TRIPPED"};
volatile State state = STOPPED;

// Thresholds: defaults, replaced by Firebase /config
float currWarn = 0.50, currTrip = 1.00, currInstant = 3.00;
float tempWarn = 40,   tempTrip = 50;
float vibWarn  = 1.0,  vibTrip  = 3.0;

int tripCount = 0, nanCount = 0;
unsigned long startTime = 0, stallSince = 0;
char faultReason[40] = "";
char warnNote[40] = "";

// Mailboxes between the two cores
volatile int  pendingCmd = 0;            // cloud -> loop: 1=start 2=stop 3=reset
volatile bool faultPending = false;      // loop -> cloud: log a fault
volatile bool tripAlertPending = false;  // loop -> cloud: send a trip alert
char alertReason[40] = "";
volatile unsigned long lastCloudOk = 0;  // cloud -> loop: last successful upload
float fCurrent, fTemp, fVib;             // snapshot at the moment of a trip

// ---------------------------------------------------------------------
//  3. SENSORS
// ---------------------------------------------------------------------
const int RPM_BUF = 6;                   // remember the last 6 pulse times (5 gaps)
volatile uint32_t pulseTimes[RPM_BUF] = {0};
volatile uint8_t pulseHead = 0;          // next slot to write
volatile uint32_t lastPulseUs = 0;
const uint32_t MIN_PULSE_US = (uint32_t)(60000000.0f / MAX_RPM);   // shortest believable gap

void IRAM_ATTR onPulse() {               // Hall interrupt: keep it tiny
  uint32_t now = micros();
  if (now - lastPulseUs > MIN_PULSE_US) {          // ignore noise faster than MAX_RPM
    pulseTimes[pulseHead] = now;
    pulseHead = (pulseHead + 1) % RPM_BUF;
    lastPulseUs = now;
  }
}

// RPM from the MEDIAN of the last gaps between pulses: a single noise pulse cannot
// make the value jump, and it still reacts within a few revolutions.
float computeRPM() {
  static float lastRpm = 0;
  uint32_t t[RPM_BUF], last;
  uint8_t head;
  noInterrupts();
  for (int i = 0; i < RPM_BUF; i++) t[i] = pulseTimes[i];
  head = pulseHead; last = lastPulseUs;
  interrupts();

  uint32_t now = micros();
  if (last == 0 || now - last > 1500000UL) { lastRpm = 0; return 0; }   // no pulse: stopped

  uint32_t iv[RPM_BUF];
  int n = 0;
  uint32_t prev = 0;
  for (int k = 0; k < RPM_BUF; k++) {              // oldest to newest
    uint32_t ts = t[(head + k) % RPM_BUF];
    if (ts == 0 || now - ts > 2000000UL) { prev = 0; continue; }   // empty or too old
    if (prev) iv[n++] = ts - prev;
    prev = ts;
  }
  if (n == 0) return lastRpm;                      // only one pulse so far: keep the last value

  for (int i = 1; i < n; i++) {                    // tiny insertion sort
    uint32_t v = iv[i]; int j = i - 1;
    while (j >= 0 && iv[j] > v) { iv[j + 1] = iv[j]; j--; }
    iv[j + 1] = v;
  }
  uint32_t med = iv[n / 2];
  uint32_t timeout = med * 3;                      // stopped if no pulse for 3 gaps
  if (timeout < 400000UL) timeout = 400000UL;
  if (timeout > 1500000UL) timeout = 1500000UL;
  if (now - last > timeout) { lastRpm = 0; return 0; }

  lastRpm = 60000000.0f / med;
  return lastRpm;
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

uint8_t mpuAddress = 0;
bool mpu6500Compatible = false;

bool readMPUId(uint8_t addr, uint8_t& id) {
  Wire.beginTransmission(addr);
  Wire.write(0x75);                      // WHO_AM_I register
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  id = Wire.read();
  return true;
}

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMPU6500Magnitude(float& magnitude) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(0x3B);                      // accelerometer X high byte
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(mpuAddress, (uint8_t)6) != 6) return false;
  int16_t rawX = (int16_t)(((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read());
  int16_t rawY = (int16_t)(((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read());
  int16_t rawZ = (int16_t)(((uint16_t)Wire.read() << 8) | (uint8_t)Wire.read());
  const float scale = 9.80665f / 4096.0f; // +/-8 g range
  float x = rawX * scale, y = rawY * scale, z = rawZ * scale;
  magnitude = sqrt(x * x + y * y + z * z);
  return true;
}

bool beginMPU() {
  const uint8_t addresses[] = {ADDR_MPU, ADDR_MPU_ALT};
  for (uint8_t addr : addresses) {
    if (!i2cPresent(addr)) continue;
    uint8_t id;
    if (!readMPUId(addr, id)) {
      Serial.printf("MPU at 0x%02X responds to scan but WHO_AM_I read failed\n", addr);
      continue;
    }
    if (id == 0x70) {                    // MPU6500 shares the MPU6050 accel register map
      mpuAddress = addr;
      if (writeMPURegister(0x6B, 0x01) && // wake and select gyro X clock
          writeMPURegister(0x1A, 0x00) && // enable accelerometer bandwidth
          writeMPURegister(0x19, 0x00) && // 1 kHz sample rate
          writeMPURegister(0x1C, 0x10)) { // accelerometer range: +/-8 g
        delay(100);
        mpu6500Compatible = true;
        hasMPU = true;
        Serial.printf("MPU6500-compatible sensor initialized at 0x%02X\n", addr);
        return true;
      }
      Serial.printf("MPU6500-compatible sensor at 0x%02X failed register setup\n", addr);
      continue;
    }
    if (id != 0x68) {
      Serial.printf("MPU at 0x%02X reports WHO_AM_I=0x%02X (expected 0x68)\n", addr, id);
      continue;
    }
    if (mpu.begin(addr, &Wire)) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setFilterBandwidth(MPU6050_BAND_260_HZ);
      mpuAddress = addr;
      mpu6500Compatible = false;
      hasMPU = true;
      Serial.printf("MPU6050 initialized at 0x%02X\n", addr);
      return true;
    }
    Serial.printf("MPU at 0x%02X reports MPU6050 ID but driver initialization failed\n", addr);
  }
  hasMPU = false;
  mpuAddress = 0;
  mpu6500Compatible = false;
  return false;
}

float readLM75A() {
  Wire.beginTransmission(ADDR_LM75);
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return NAN;
  if (Wire.requestFrom((int)ADDR_LM75, 2) != 2) return NAN;
  int16_t raw = (Wire.read() << 8) | Wire.read();
  return (raw >> 5) * 0.125;
}

void readDS18B20() {                     // non-blocking: request, collect 800 ms later
  static unsigned long t0 = 0;
  static bool waiting = false;
  if (!waiting) { ds.requestTemperatures(); t0 = millis(); waiting = true; }
  else if (millis() - t0 >= 800) {
    float t = ds.getTempCByIndex(0);
    if (t != DEVICE_DISCONNECTED_C && t != 85.0) { motorTemp = t; okDS = millis(); }
    else if (millis() - okDS > SENSOR_TIMEOUT_MS) motorTemp = NAN;   // only after a LONG dropout
    waiting = false;
  }
}

void readFastSensors() {                 // every 200 ms
#if !SIMULATE
  unsigned long nowMs = millis();
  bool inaOk = false;
  if (hasINA && i2cPresent(ADDR_INA)) {
    float busV = ina.getBusVoltage_V();
    float shuntmV = ina.getShuntVoltage_mV();
    float c = fabs(ina.getCurrent_mA()) / 1000.0;
    if (!isnan(c) && !isnan(busV)) {
      current = c; voltage = busV + shuntmV / 1000.0; okINA = nowMs; inaOk = true;
    }
  }
  if (!inaOk && nowMs - okINA > SENSOR_TIMEOUT_MS) { current = NAN; voltage = NAN; }

  float amb = readLM75A();               // not critical: only displayed
  if (!isnan(amb)) { ambientTemp = amb; okAmb = nowMs; }
  else if (nowMs - okAmb > SENSOR_TIMEOUT_MS) ambientTemp = NAN;
  readDS18B20();

  rpm = computeRPM();
#endif
}

void readVibration() {                   // every 1 s, takes about 0.2-0.4 s
#if !SIMULATE
  const int N = 200;
  static float mag[N];
  auto lose = [&]() { if (millis() - okVib > SENSOR_TIMEOUT_MS) vibration = NAN; };
  if (!hasMPU || !i2cPresent(mpuAddress)) { lose(); return; }
  sensors_event_t a, g, t;
  float sum = 0;
  int n = 0, fails = 0;
  for (int i = 0; i < N; i++) {
    float m;
    if (mpu6500Compatible) {
      if (!readMPU6500Magnitude(m)) {    // a loose wire: skip this sample, give up after 5 in a row
        if (++fails >= 5) break;
        continue;
      }
      fails = 0;
    } else {
      mpu.getEvent(&a, &g, &t);
      m = sqrt(a.acceleration.x * a.acceleration.x + a.acceleration.y * a.acceleration.y +
               a.acceleration.z * a.acceleration.z);
    }
    mag[n] = m;
    sum += m;
    n++;
  }
  if (n < N / 2) { lose(); return; }     // too few good samples this second
  float mean = sum / n, sq = 0;
  for (int i = 0; i < n; i++) sq += (mag[i] - mean) * (mag[i] - mean);
  vibration = sqrt(sq / n);              // RMS of the fluctuation = vibration
  okVib = millis();
#endif
}

void scanI2C() {                         // prints every device found on the I2C bus
  Serial.print("I2C scan:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) if (i2cPresent(a)) { Serial.printf(" 0x%02X", a); n++; }
  Serial.println(n ? "" : " NOTHING FOUND (check SDA/SCL, 3.3V and GND)");
}

void retrySensors() {                    // late start: pick up devices that were missing at boot
  static unsigned long t = 0;
  if (millis() - t < 5000) return;
  t = millis();
  if (!hasOLED && i2cPresent(ADDR_OLED) && display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED)) {
    hasOLED = true;
    display.setTextColor(SSD1306_WHITE);
    Serial.println("OLED found");
  }
#if !SIMULATE
  if (!hasINA && i2cPresent(ADDR_INA) && ina.begin()) { hasINA = true; Serial.println("INA219 found"); }
  if (!hasMPU) beginMPU();
#endif
}

// Computed values (NAN if an input is missing)
float powerW()   { return (isnan(voltage) || isnan(current)) ? NAN : voltage * current; }
float tempRise() { return (isnan(motorTemp) || isnan(ambientTemp)) ? NAN : motorTemp - ambientTemp; }

bool anySensorBad() { return isnan(current) || isnan(motorTemp) || isnan(vibration); }

// ---------------------------------------------------------------------
//  4. FAULT ENGINE
// ---------------------------------------------------------------------
void relayOn(bool on) { digitalWrite(RELAY_PIN, (on ^ RELAY_ACTIVE_LOW) ? HIGH : LOW); }
int level(float v, float warn, float trip) { return v >= trip ? 2 : (v >= warn ? 1 : 0); }

void trip(const char* reason) {
  relayOn(false);                        // cut the motor FIRST
  strncpy(faultReason, reason, sizeof(faultReason) - 1);
  fCurrent = current; fTemp = motorTemp; fVib = vibration;
  strncpy(alertReason, reason, sizeof(alertReason) - 1);
  state = TRIPPED;
  faultPending = true;                   // cloud task will log it
  tripAlertPending = true;               // ...and send the phone alert
  Serial.printf("TRIPPED: %s\n", reason);
}

void startMotor() {
  if (state != STOPPED) return;          // never from TRIPPED
  if (anySensorBad()) { Serial.println("Start refused: a sensor is not reading"); return; }
  relayOn(true);
  startTime = millis();
  tripCount = 0; nanCount = 0; stallSince = 0;
  warnNote[0] = 0;
  state = STARTING;
}
void stopMotor()  { relayOn(false); warnNote[0] = 0; state = STOPPED; }
void resetFault() { if (state == TRIPPED) { state = STOPPED; faultReason[0] = 0; } }

void evaluate() {                        // called every 200 ms
  if (state == STOPPED || state == TRIPPED) return;

  if (anySensorBad()) {                  // a dead sensor is itself a fault
    const char* which = isnan(current) ? "sensor failure: current"
                      : isnan(motorTemp) ? "sensor failure: motor temp"
                      : "sensor failure: vibration";
    if (++nanCount >= CONFIRM_SAMPLES) trip(which);
    return;
  }
  nanCount = 0;

  if (current >= currInstant) { trip("short circuit / stall"); return; }

  bool blanking = (state == STARTING) && (millis() - startTime < BLANK_MS);

  int worst = 0;
  const char* why = "";
  auto consider = [&](int lv, const char* r) { if (lv > worst) { worst = lv; why = r; } };
  if (!blanking) {
    consider(level(current, currWarn, currTrip), "over-current");
    if (voltage < V_LOW) consider(2, "under-voltage");
  }
  consider(level(motorTemp, tempWarn, tempTrip), "over-temperature");
  consider(level(vibration, vibWarn, vibTrip), "high vibration");

  // stall: shaft not turning while the motor draws current
  if (STALL_CHECK && !blanking && rpm < STALL_RPM && current > currWarn) {
    if (!stallSince) stallSince = millis();
    else if (millis() - stallSince > STALL_MS) { trip("stall (no rotation)"); return; }
  } else stallSince = 0;

  if (worst == 2) {
    if (++tripCount >= CONFIRM_SAMPLES) { trip(why); return; }
  } else tripCount = 0;

  if (!blanking && state == STARTING) state = RUNNING;
  if (state != STARTING) {
    state = (worst >= 1) ? WARNING : RUNNING;
    strncpy(warnNote, worst >= 1 ? why : "", sizeof(warnNote) - 1);
  }
}

void updateBuzzer() {
  bool on = false;
  if (state == TRIPPED) on = true;                          // continuous
  else if (state == WARNING) on = (millis() % 500) < 100;   // short beeps
  digitalWrite(BUZZER_PIN, on);
}

// ---------------------------------------------------------------------
//  5. OLED
// ---------------------------------------------------------------------
String f(float v, int d) { return isnan(v) ? String("--") : String(v, d); }

void drawOLED() {
  if (!hasOLED) return;
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println(STATE_NAMES[state]);
  String line = "V:";
  line.concat(f(voltage, 2)); line.concat("  I:"); line.concat(f(current, 2)); line.concat("A");
  display.println(line);
  line = "RPM:";
  line.concat(f(rpm, 0)); line.concat(" P:"); line.concat(f(powerW(), 1)); line.concat("W");
  display.println(line);
  line = "Vib:";
  line.concat(f(vibration, 2)); line.concat(" m/s2");
  display.println(line);
  line = "Mot:";
  line.concat(f(motorTemp, 1)); line.concat(" Amb:"); line.concat(f(ambientTemp, 1)); line.concat("C");
  display.println(line);
  display.println(WiFi.status() == WL_CONNECTED ? "WiFi: OK" : "WiFi: --");
  if (state == TRIPPED) display.println(faultReason);
  else if (state == WARNING) display.println(warnNote);
  display.display();
}

// ---------------------------------------------------------------------
//  6. CLOUD TASK (core 0)
// ---------------------------------------------------------------------
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig fbconfig;
int lastSeq = 0;
bool runFlagInDb = false;

void pollCommand() {
  if (!Firebase.RTDB.getInt(&fbdo, dbPath("/command/seq"))) return;
  int seq = fbdo.intData();
  if (seq == lastSeq) return;            // nothing new
  lastSeq = seq;
  bool run = false, reset = false;
  if (Firebase.RTDB.getBool(&fbdo, dbPath("/command/run")))        run = fbdo.boolData();
  if (Firebase.RTDB.getBool(&fbdo, dbPath("/command/resetFault"))) reset = fbdo.boolData();
  if (reset) { pendingCmd = 3; Firebase.RTDB.setBool(&fbdo, dbPath("/command/resetFault"), false); }
  else if (run) { pendingCmd = 1; runFlagInDb = true; }
  else pendingCmd = 2;                   // OFF always honoured
  Serial.printf("[cloud] command seq=%d run=%d reset=%d\n", seq, run, reset);
}

void uploadTelemetry() {
  FirebaseJson j;
  // A dead sensor must not leave an OLD value in the database. Numbers are sent;
  // NAN values are DELETED from the database (once), so the dashboard shows "--".
  static uint8_t clearedMask = 0;        // bit set = this field is already cleared in the DB
  const char* clearKey[8]; int clearBit[8]; int nClear = 0, idx = 0;
  auto put = [&](const char* k, float v) {
    uint8_t bit = 1 << idx++;
    if (!isnan(v)) { j.set(k, v); clearedMask &= ~bit; }
    else if (!(clearedMask & bit)) { clearKey[nClear] = k; clearBit[nClear++] = bit; }
  };
  put("voltage", voltage);   put("current", current);   put("rpm", rpm);
  put("vibration", vibration); put("motorTemp", motorTemp); put("ambientTemp", ambientTemp);
  put("power", powerW());  put("tempRise", tempRise());
  j.set("state", STATE_NAMES[state]);
  j.set("fault", (const char*)(state == TRIPPED ? faultReason : (state == WARNING ? warnNote : "")));
  j.set("rssi", WiFi.RSSI());
  j.set("heartbeat/.sv", "timestamp");   // the SERVER writes the time
  if (Firebase.RTDB.updateNode(&fbdo, dbPath("/telemetry"), &j)) lastCloudOk = millis();
  else Serial.printf("[cloud] telemetry failed: %s\n", fbdo.errorReason().c_str());

  for (int i = 0; i < nClear; i++)       // remove stale values of dead sensors
    {
      String p = dbPath("/telemetry/");
      p.concat(clearKey[i]);
      if (Firebase.RTDB.deleteNode(&fbdo, p)) clearedMask |= clearBit[i];
    }

  if (runFlagInDb && (state == STOPPED || state == TRIPPED)) {   // keep stored 'run' honest
    Firebase.RTDB.setBool(&fbdo, dbPath("/command/run"), false);
    runFlagInDb = false;
  }
}

void logFault() {
  FirebaseJson j;
  j.set("ts/.sv", "timestamp");
  j.set("reason", (const char*)faultReason);
  j.set("current", isnan(fCurrent) ? 0 : fCurrent);
  j.set("motorTemp", isnan(fTemp) ? 0 : fTemp);
  j.set("vibration", isnan(fVib) ? 0 : fVib);
  if (Firebase.RTDB.pushJSON(&fbdo, dbPath("/faults"), &j)) faultPending = false;
}

void logHistory() {                      // one slot of the ring buffer in /history
  time_t now = time(nullptr);
  if (now < 1700000000) return;          // clock not synced yet
  int slot = (now / HISTORY_INTERVAL_S) % HISTORY_SLOTS;
  FirebaseJson j;
  j.set("t", (int)now);                  // seconds since 1970
  auto put = [&](const char* k, float v) { if (!isnan(v)) j.set(k, v); };
  put("current", current); put("voltage", voltage); put("rpm", rpm);
  put("vibration", vibration); put("motorTemp", motorTemp);
  put("ambientTemp", ambientTemp); put("power", powerW());
  String path = dbPath("/history/");
  path.concat(String(slot));
  Firebase.RTDB.setJSON(&fbdo, path, &j);
}

// Sends one Telegram message. Returns true on success.
bool sendAlert(const String& text) {
  if (strlen(TG_TOKEN) < 10 || strlen(TG_CHAT_ID) < 3) return false;   // alerts disabled
  WiFiClientSecure client;
  client.setInsecure();                  // skips certificate check (simple, see notes)
  HTTPClient http;
  http.setTimeout(8000);
  if (!http.begin(client, String("https://api.telegram.org/bot") + TG_TOKEN + "/sendMessage")) return false;
  http.addHeader("Content-Type", "application/json");
  String body = String("{\"chat_id\":\"") + TG_CHAT_ID + "\",\"text\":\"" + text + "\"}";
  int code = http.POST(body);
  http.end();
  Serial.printf("[alert] Telegram HTTP %d\n", code);
  return code == 200;
}

void handleAlerts(unsigned long now) {
  static unsigned long tTry = 0, lastWarn = 0;
  static State prev = STOPPED;

  if (tripAlertPending && (tTry == 0 || now - tTry >= 10000)) {   // retry every 10 s
    tTry = now;
    String m = String("\xF0\x9F\x9A\xA8 MOTOR 1 TRIPPED\\nReason: ") + alertReason +
               "\\nCurrent: " + f(fCurrent, 2) + " A\\nMotor temp: " + f(fTemp, 1) +
               " C\\nVibration: " + f(fVib, 2) + "\\nThe supply relay is open.\\n" + DASHBOARD_URL;
    if (sendAlert(m) || strlen(TG_TOKEN) < 10) { tripAlertPending = false; tTry = 0; }
  }

  State cur = state;                     // warning alert: once per 5 minutes at most
  if (cur == WARNING && prev != WARNING && (lastWarn == 0 || now - lastWarn > 300000)) {
    lastWarn = now;
    sendAlert(String("\xE2\x9A\xA0 Motor 1 WARNING\\n") + warnNote + "\\n" + DASHBOARD_URL);
  }
  prev = cur;
}

void pullConfig() {
  if (!Firebase.RTDB.getJSON(&fbdo, dbPath("/config"))) return;
  FirebaseJson &j = fbdo.jsonObject();
  FirebaseJsonData d;
  float cw = currWarn, ct = currTrip, ci = currInstant, tw = tempWarn, tt = tempTrip,
        vw = vibWarn, vt = vibTrip;
  if (j.get(d, "currWarn"))    cw = d.to<float>();
  if (j.get(d, "currTrip"))    ct = d.to<float>();
  if (j.get(d, "currInstant")) ci = d.to<float>();
  if (j.get(d, "tempWarn"))    tw = d.to<float>();
  if (j.get(d, "tempTrip"))    tt = d.to<float>();
  if (j.get(d, "vibWarn"))     vw = d.to<float>();
  if (j.get(d, "vibTrip"))     vt = d.to<float>();
  if (cw > 0 && cw < ct && ct < ci && tw < tt && vw < vt) {     // sanity check
    currWarn = cw; currTrip = ct; currInstant = ci;
    tempWarn = tw; tempTrip = tt; vibWarn = vw; vibTrip = vt;
    Serial.println("[cloud] config loaded");
  } else Serial.println("[cloud] config REJECTED (warn must be below trip)");
}

void cloudTask(void*) {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long lastWifiStatus = 0, lastWifiRetry = millis();
  while (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiStatus >= 5000) {
      lastWifiStatus = now;
      Serial.printf("[wifi] waiting for connection, status=%d\n", (int)WiFi.status());
    }
    if (now - lastWifiRetry >= 15000) {
      lastWifiRetry = now;
      Serial.println("[wifi] retrying configured network");
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  Serial.printf("[cloud] Wi-Fi connected, IP %s\n", WiFi.localIP().toString().c_str());

  fbconfig.api_key = API_KEY;
  fbconfig.database_url = DATABASE_URL;
  auth.user.email = DEVICE_EMAIL;
  auth.user.password = DEVICE_PASS;
  fbconfig.token_status_callback = tokenStatusCallback;
  Firebase.reconnectNetwork(true);       // older library: Firebase.reconnectWiFi(true)
  Firebase.begin(&fbconfig, &auth);
  while (!Firebase.ready()) vTaskDelay(200 / portTICK_PERIOD_MS);
  Serial.println("[cloud] Firebase ready");
  configTime(0, 0, "pool.ntp.org", "time.google.com");   // real clock for history
  sendAlert(String("\xE2\x9C\x85 Motor 1 monitor online\\nIP: ") + WiFi.localIP().toString());

  // Boot safety: ignore any old command and clear stale flags
  if (Firebase.RTDB.getInt(&fbdo, dbPath("/command/seq"))) lastSeq = fbdo.intData();
  Firebase.RTDB.setBool(&fbdo, dbPath("/command/run"), false);
  Firebase.RTDB.setBool(&fbdo, dbPath("/command/resetFault"), false);
  pullConfig();

  unsigned long tCmd = 0, tTel = 0, tCfg = millis(), tHist = 0;
  for (;;) {
    if (Firebase.ready()) {              // also refreshes the login token
      unsigned long now = millis();
      if (faultPending) logFault();
      handleAlerts(now);
      if (now - tCmd >= 1000)  { tCmd = now; pollCommand(); }
      if (now - tTel >= 1000)  { tTel = now; uploadTelemetry(); }
      if (now - tCfg >= 30000) { tCfg = now; pullConfig(); }
      if (now - tHist >= HISTORY_INTERVAL_S * 1000UL) { tHist = now; logHistory(); }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// ---------------------------------------------------------------------
//  7. SETUP / LOOP (core 1)
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(RELAY_PIN, OUTPUT);
  relayOn(false);                        // motor OFF before anything else
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(20);                   // ms: a loose wire must not stall the loop
  delay(300);                            // let the sensors power up
  scanI2C();

  hasOLED = i2cPresent(ADDR_OLED) && display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  if (hasOLED) {
    display.setTextColor(SSD1306_WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Motor Monitor");
    display.println("starting...");
    display.display();
  }

#if !SIMULATE
  hasINA = i2cPresent(ADDR_INA) && ina.begin();
  beginMPU();
  ds.begin();
  ds.setWaitForConversion(false);
  pinMode(HALL_PIN, INPUT);              // external 10k pull-up to 3.3V
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), onPulse, FALLING);
#endif

  Serial.printf("Boot: OLED=%d INA219=%d MPU6050=%d LM75A=%d DS18B20=%d SIMULATE=%d\n",
                hasOLED, hasINA, hasMPU, i2cPresent(ADDR_LM75), ds.getDeviceCount(), SIMULATE);
  lastCloudOk = millis();
  // Priority 0 (same as the idle task) + bigger stack. At priority 1 the Firebase library's
  // waiting loops starve IDLE0 and trigger the task watchdog (reboot every ~12 s).
  xTaskCreatePinnedToCore(cloudTask, "cloud", 24576, NULL, 0, NULL, 0);
#if SIMULATE
  Serial.println("Serial: 1/2/3 current normal/warn/trip  4/5 temp hot/normal  6/7 vib high/low  8/9 voltage low/normal");
#endif
}

unsigned long lastFast = 0, lastVib = 0, lastDraw = 0, lastPrint = 0;

void loop() {
#if SIMULATE
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') current = 0.2;   if (c == '2') current = 0.7;  if (c == '3') current = 1.5;
    if (c == '4') motorTemp = 55;  if (c == '5') motorTemp = 30;
    if (c == '6') vibration = 4.0; if (c == '7') vibration = 0.1;
    if (c == '8') voltage = 9.0;   if (c == '9') voltage = 12.0;
  }
  rpm = (state == STOPPED || state == TRIPPED) ? 0 : 1450;
#endif

  int cmd = pendingCmd;                  // commands from the app
  if (cmd) {
    pendingCmd = 0;
    if (cmd == 1) startMotor();
    else if (cmd == 2) stopMotor();
    else if (cmd == 3) resetFault();
  }

  if (state == STOPPED) retrySensors();

  unsigned long now = millis();
  if (now - lastFast >= 200) { lastFast = now; readFastSensors(); evaluate(); }
  if (now - lastVib >= 1000) { lastVib = now; readVibration(); }
  if (now - lastDraw >= 500) { lastDraw = now; drawOLED(); }

  if (STOP_ON_CLOUD_LOSS && (state == STARTING || state == RUNNING || state == WARNING) &&
      now - lastCloudOk > CLOUD_TIMEOUT_MS) trip("cloud link lost");

  if (now - lastPrint >= 1000) {
    lastPrint = now;
    Serial.printf("%s V=%s I=%s RPM=%s Vib=%s Mot=%s Amb=%s\n", STATE_NAMES[state],
                  f(voltage, 2).c_str(), f(current, 2).c_str(), f(rpm, 0).c_str(),
                  f(vibration, 2).c_str(), f(motorTemp, 1).c_str(), f(ambientTemp, 1).c_str());
  }
  updateBuzzer();
}
