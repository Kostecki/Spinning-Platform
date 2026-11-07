// Spinning Platform V2
// Spins only while something is on the platform (load cell), speed from the pot.
//
// Serial (115200, any time a host is connected):
//   prints CSV: ms,raw,grams,state,rpm,vbat
//   t          tare now (platform must be empty)
//   c <grams>  calibrate with a known weight on the platform
//   m          toggle "force run" (spin regardless of weight, for noise tests)
//   s          noise stats over the last 50 samples

#include "config.h"
#include "hx711.h"

#if defined(ARDUINO_ARCH_RP2040)
#include <EEPROM.h>
#define HAS_STORAGE 1
#else
#define HAS_STORAGE 0
#endif

// ---------------------------------------------------------------- storage

struct Settings {
  uint32_t magic;
  int32_t tareRaw;
  float countsPerGram;
};
const uint32_t SETTINGS_MAGIC = 0x53504E32;  // "SPN2"

Settings settings = {SETTINGS_MAGIC, 0, DEFAULT_COUNTS_PER_GRAM};
bool haveStoredTare = false;
int32_t savedTareRaw = 0;
uint32_t lastSaveMs = 0;
bool savedThisBoot = false;

void loadSettings() {
#if HAS_STORAGE
  EEPROM.begin(64);
  Settings s;
  EEPROM.get(0, s);
  if (s.magic == SETTINGS_MAGIC && s.countsPerGram != 0 && !isnan(s.countsPerGram)) {
    settings = s;
    haveStoredTare = true;
    savedTareRaw = s.tareRaw;
  }
#endif
}

void saveSettings() {
#if HAS_STORAGE
  EEPROM.put(0, settings);
  EEPROM.commit();
  lastSaveMs = millis();
  savedThisBoot = true;
  haveStoredTare = true;
  savedTareRaw = settings.tareRaw;
#endif
}

// ---------------------------------------------------------------- motor

const uint8_t FULL_SEQ[4][4] = {{1, 1, 0, 0}, {0, 1, 1, 0}, {0, 0, 1, 1}, {1, 0, 0, 1}};
const uint8_t HALF_SEQ[8][4] = {{1, 0, 0, 0}, {1, 1, 0, 0}, {0, 1, 0, 0}, {0, 1, 1, 0},
                                {0, 0, 1, 0}, {0, 0, 1, 1}, {0, 0, 0, 1}, {1, 0, 0, 1}};
const uint8_t MOTOR_PINS[4] = {PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4};

float currentRpm = 0;   // 0 = coils released
float targetRpm = 0;
uint8_t phase = 0;
uint32_t lastStepUs = 0;
uint32_t lastRampUs = 0;

void writeCoils(const uint8_t *c) {
  for (uint8_t i = 0; i < 4; i++) digitalWrite(MOTOR_PINS[i], c[i]);
}

void releaseCoils() {
  static const uint8_t off[4] = {0, 0, 0, 0};
  writeCoils(off);
}

void stepOnce() {
  uint8_t n = HALF_STEP ? 8 : 4;
  phase = REVERSE ? (phase + n - 1) % n : (phase + 1) % n;
  writeCoils(HALF_STEP ? HALF_SEQ[phase] : FULL_SEQ[phase]);
}

void updateMotor() {
  uint32_t now = micros();
  float dt = (now - lastRampUs) / 1e6f;
  lastRampUs = now;

  // Ramp toward target; start from / stop at START_RPM.
  if (targetRpm > 0 && currentRpm == 0) {
    currentRpm = START_RPM;
    lastStepUs = now;
    writeCoils(HALF_STEP ? HALF_SEQ[phase] : FULL_SEQ[phase]);
  }
  if (currentRpm == 0) return;
  float maxDelta = ACCEL_RPM_PER_S * dt;
  float goal = targetRpm > 0 ? targetRpm : START_RPM;
  if (currentRpm < goal) currentRpm = min(goal, currentRpm + maxDelta);
  else if (currentRpm > goal) currentRpm = max(goal, currentRpm - maxDelta);
  if (targetRpm == 0 && currentRpm <= START_RPM) {
    currentRpm = 0;
    releaseCoils();
    return;
  }

  float stepsPerRev = STEPS_PER_REV_FULL * (HALF_STEP ? 2 : 1);
  uint32_t intervalUs = (uint32_t)(60e6f / (currentRpm * stepsPerRev));
  if (now - lastStepUs >= intervalUs) {
    lastStepUs += intervalUs;
    if (now - lastStepUs > intervalUs) lastStepUs = now;  // don't burst after a stall
    stepOnce();
  }
}

// ---------------------------------------------------------------- scale

HX711 scale;
const uint8_t MEDIAN_N = 5;
int32_t rawBuf[MEDIAN_N];
uint8_t rawCount = 0, rawIdx = 0;
int32_t rawMedian = 0;
bool newSample = false;

const uint8_t HIST_N = 50;
float gramsHist[HIST_N];
uint8_t histCount = 0, histIdx = 0;

float grams() { return (rawMedian - settings.tareRaw) / settings.countsPerGram; }

void updateScale() {
  if (!scale.ready()) return;
  rawBuf[rawIdx] = scale.read();
  rawIdx = (rawIdx + 1) % MEDIAN_N;
  if (rawCount < MEDIAN_N) rawCount++;

  int32_t s[MEDIAN_N];
  memcpy(s, rawBuf, sizeof(s));
  for (uint8_t i = 1; i < rawCount; i++)
    for (uint8_t j = i; j > 0 && s[j - 1] > s[j]; j--) {
      int32_t t = s[j]; s[j] = s[j - 1]; s[j - 1] = t;
    }
  rawMedian = s[rawCount / 2];
  newSample = rawCount == MEDIAN_N;

  if (newSample) {
    gramsHist[histIdx] = grams();
    histIdx = (histIdx + 1) % HIST_N;
    if (histCount < HIST_N) histCount++;
  }
}

// Spread (max - min) of the last n samples, in grams.
float recentSpread(uint8_t n) {
  n = min(n, histCount);
  if (n == 0) return 1e9;
  float lo = 1e9, hi = -1e9;
  for (uint8_t i = 0; i < n; i++) {
    float g = gramsHist[(histIdx + HIST_N - 1 - i) % HIST_N];
    lo = min(lo, g);
    hi = max(hi, g);
  }
  return hi - lo;
}

int32_t averageRaw(uint8_t n) {
  int64_t sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    while (!scale.ready()) delay(1);
    sum += scale.read();
  }
  return (int32_t)(sum / n);
}

// ---------------------------------------------------------------- inputs

float potRpm() {
  static float avg = -1;
  static uint32_t last = 0;
  if (avg < 0 || millis() - last >= 50) {
    last = millis();
    int v = analogRead(PIN_POT);
    avg = avg < 0 ? v : avg * 0.8f + v * 0.2f;
  }
  float t = constrain((avg - POT_LOW) / float(POT_HIGH - POT_LOW), 0.0f, 1.0f);
  return MIN_RPM + t * (MAX_RPM - MIN_RPM);
}

float vbat = 0;
bool lowBattery = false;

void updateBattery() {
#if BATTERY_SENSE
  static uint32_t last = 0, lowSince = 0;
  if (millis() - last < 1000) return;
  last = millis();
  float v = analogRead(PIN_VBAT) * 3.3f / 4095.0f * VBAT_DIVIDER;
  vbat = vbat == 0 ? v : vbat * 0.8f + v * 0.2f;
  if (!lowBattery) {
    if (vbat >= VBAT_LOW) lowSince = millis();
    else if (millis() - lowSince >= VBAT_LOW_MS) lowBattery = true;
  } else if (vbat >= VBAT_RECOVER) {
    lowBattery = false;
    lowSince = millis();
  }
#endif
}

// ---------------------------------------------------------------- state

enum State : uint8_t { IDLE, RUNNING, LOW_BATT };
const char *STATE_NAMES[] = {"idle", "running", "lowbatt"};
State state = IDLE;
bool forceRun = false;
uint32_t aboveSince = 0, belowSince = 0, negativeSince = 0, idleSince = 0;

void tareNow() {
  settings.tareRaw = rawMedian;
  histCount = 0;
}

void updatePresence() {
  if (!newSample) return;
  newSample = false;
  uint32_t now = millis();
  float g = grams();

  if (g <= ON_GRAMS) aboveSince = now;
  if (g >= OFF_GRAMS) belowSince = now;
  if (g >= NEGATIVE_RETARE_GRAMS) negativeSince = now;

  if (lowBattery) {
    state = LOW_BATT;
  } else if (state == LOW_BATT) {
    state = IDLE;
    idleSince = now;
  }

  if (state == IDLE && now - aboveSince >= ON_DEBOUNCE_MS) state = RUNNING;
  if (state == RUNNING && now - belowSince >= OFF_DEBOUNCE_MS) {
    state = IDLE;
    idleSince = now;
  }

  if (state != IDLE || forceRun) return;

  // Nothing can weigh less than the empty platform: stale tare.
  if (now - negativeSince >= NEGATIVE_RETARE_MS) {
    tareNow();
    negativeSince = now;
    return;
  }

  // Auto-zero: track slow drift while empty, steady and stopped.
  if (now - idleSince >= AUTOZERO_AFTER_IDLE_MS && fabsf(g) < OFF_GRAMS &&
      recentSpread(10) < STABLE_BAND_GRAMS && currentRpm == 0) {
    settings.tareRaw += (int32_t)((rawMedian - settings.tareRaw) * AUTOZERO_RATE);

    // Persist the empty reading so a bottle placed before power-on is detected at boot.
    float drift = (settings.tareRaw - savedTareRaw) / settings.countsPerGram;
    if ((!haveStoredTare || fabsf(drift) > SAVE_TARE_DELTA_GRAMS) &&
        (!savedThisBoot || now - lastSaveMs >= SAVE_TARE_MIN_INTERVAL_MS) &&
        now - idleSince >= 2 * AUTOZERO_AFTER_IDLE_MS) {
      saveSettings();
    }
  }
}

// ---------------------------------------------------------------- LED

void updateLed() {
  uint32_t t = millis();
  bool on;
  switch (state) {
    case RUNNING: on = true; break;
    case LOW_BATT: on = (t % 1000) < 100 || ((t % 1000) > 200 && (t % 1000) < 300); break;
    default: on = (t % 3000) < 30; break;  // brief blip while waiting
  }
  digitalWrite(PIN_STATUS_LED, on);
}

// ---------------------------------------------------------------- serial

float sq_f(float x) { return x * x; }

void printStats() {
  float sum = 0, sq = 0;
  for (uint8_t i = 0; i < histCount; i++) sum += gramsHist[i];
  float mean = histCount ? sum / histCount : 0;
  for (uint8_t i = 0; i < histCount; i++) sq += sq_f(gramsHist[i] - mean);
  Serial.print("# n="); Serial.print(histCount);
  Serial.print(" mean="); Serial.print(mean, 2);
  Serial.print(" std="); Serial.print(histCount ? sqrtf(sq / histCount) : 0, 2);
  Serial.print(" spread="); Serial.print(recentSpread(HIST_N), 2);
  Serial.print(" rpm="); Serial.println(currentRpm, 1);
}

void handleSerial() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "t") {
    tareNow();
    saveSettings();
    Serial.println("# tared");
  } else if (line.startsWith("c ")) {
    float known = line.substring(2).toFloat();
    if (known <= 0) {
      Serial.println("# usage: c <grams>");
      return;
    }
    int32_t raw = averageRaw(20);
    settings.countsPerGram = (raw - settings.tareRaw) / known;
    saveSettings();
    Serial.print("# countsPerGram=");
    Serial.println(settings.countsPerGram, 3);
  } else if (line == "m") {
    forceRun = !forceRun;
    Serial.print("# forceRun=");
    Serial.println(forceRun);
  } else if (line == "s") {
    printStats();
  }
}

void logSample() {
  static uint8_t lastHist = 255;
  if (!Serial || histIdx == lastHist || Serial.availableForWrite() < 64) return;
  lastHist = histIdx;
  Serial.print(millis()); Serial.print(',');
  Serial.print(rawMedian); Serial.print(',');
  Serial.print(grams(), 2); Serial.print(',');
  Serial.print(STATE_NAMES[state]); Serial.print(',');
  Serial.print(currentRpm, 1); Serial.print(',');
  Serial.println(vbat, 2);
}

// ---------------------------------------------------------------- main

void setup() {
  for (uint8_t i = 0; i < 4; i++) pinMode(MOTOR_PINS[i], OUTPUT);
  releaseCoils();
  pinMode(PIN_STATUS_LED, OUTPUT);
  analogReadResolution(12);
  Serial.begin(115200);
  Serial.setTimeout(20);

  scale.begin(PIN_HX_DOUT, PIN_HX_SCK);
  loadSettings();
  delay(400);  // HX711 settling after power-up
  int32_t raw = averageRaw(MEDIAN_N);
  for (uint8_t i = 0; i < MEDIAN_N; i++) rawBuf[i] = raw;
  rawCount = MEDIAN_N;
  rawMedian = raw;

  // No stored empty reading: assume the platform is empty now.
  if (!haveStoredTare) settings.tareRaw = raw;

  uint32_t now = millis();
  aboveSince = belowSince = negativeSince = idleSince = now;
  lastRampUs = micros();
}

void loop() {
  updateScale();
  updateBattery();
  updatePresence();
  targetRpm = ((state == RUNNING || forceRun) && !lowBattery) ? potRpm() : 0;
  updateMotor();
  updateLed();
  handleSerial();
  logSample();
}
