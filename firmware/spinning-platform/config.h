#pragma once

// ---- Pins (XIAO) ----
// Motor: ULN2003 IN1..IN4. Same wiring as V1.
const uint8_t PIN_IN1 = D10;
const uint8_t PIN_IN2 = D9;
const uint8_t PIN_IN3 = D8;
const uint8_t PIN_IN4 = D7;

const uint8_t PIN_POT = A0;         // pot wiper (pot on 3V3)
const uint8_t PIN_VBAT = A1;        // battery via 100k/100k divider
const uint8_t PIN_HX_DOUT = D2;     // HX711 breakout, powered from 3V3
const uint8_t PIN_HX_SCK = D3;
const uint8_t PIN_STATUS_LED = D6;         // status LED (active high)

// Set to 0 when no battery divider is wired (e.g. the V1 board test rig).
#define BATTERY_SENSE 1

// ---- Motor ----
const bool HALF_STEP = false;       // false = two-phase full step (max torque, as V1)
const bool REVERSE = false;
const float STEPS_PER_REV_FULL = 2048.0f;
const float MIN_RPM = 2.0f;
const float MAX_RPM = 15.0f;
const float ACCEL_RPM_PER_S = 8.0f; // ramp so a heavy bottle doesn't slip
const float START_RPM = 1.0f;       // ramp starts/ends here

// ---- Pot ----
const int POT_LOW = 80;             // 12-bit; below this = MIN_RPM
const int POT_HIGH = 4095;

// ---- Scale ----
// Default until calibrated over serial ("c <grams>"). Sign depends on cell orientation.
const float DEFAULT_COUNTS_PER_GRAM = 860.0f;  // 5 kg cell, 2 mV/V, 3V3 excitation
const float ON_GRAMS = 5.0f;                 // something there if above this...
const uint32_t ON_DEBOUNCE_MS = 500;         // ...for this long
const float OFF_GRAMS = 3.0f;                // empty if below this...
const uint32_t OFF_DEBOUNCE_MS = 3000;       // ...for this long
const float NEGATIVE_RETARE_GRAMS = -5.0f;   // reading this far below zero = tare is stale
const uint32_t NEGATIVE_RETARE_MS = 2000;
const float STABLE_BAND_GRAMS = 1.5f;        // auto-zero only when this steady
const uint32_t AUTOZERO_AFTER_IDLE_MS = 5000;
const float AUTOZERO_RATE = 0.1f;            // fraction of error removed per sample
const float SAVE_TARE_DELTA_GRAMS = 0.5f;
const uint32_t SAVE_TARE_MIN_INTERVAL_MS = 10UL * 60 * 1000;

// ---- Battery ----
const float VBAT_DIVIDER = 2.0f;
const float VBAT_LOW = 3.3f;                 // stop below this (under load)...
const uint32_t VBAT_LOW_MS = 10000;          // ...for this long
const float VBAT_RECOVER = 3.6f;             // resume above this (e.g. charging)
