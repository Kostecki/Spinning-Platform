#pragma once

// Minimal HX711 driver: channel A, gain 128, non-blocking ready check.
class HX711 {
public:
  void begin(uint8_t dout, uint8_t sck) {
    _dout = dout;
    _sck = sck;
    pinMode(_dout, INPUT);
    pinMode(_sck, OUTPUT);
    digitalWrite(_sck, LOW);
  }

  bool ready() const { return digitalRead(_dout) == LOW; }

  // Call only when ready(). Takes ~60 us; SCK must not stay high > 60 us.
  int32_t read() {
    uint32_t v = 0;
    noInterrupts();
    for (uint8_t i = 0; i < 24; i++) {
      digitalWrite(_sck, HIGH);
      delayMicroseconds(1);
      v = (v << 1) | (digitalRead(_dout) ? 1 : 0);
      digitalWrite(_sck, LOW);
      delayMicroseconds(1);
    }
    // 25th pulse selects channel A / gain 128 for the next conversion.
    digitalWrite(_sck, HIGH);
    delayMicroseconds(1);
    digitalWrite(_sck, LOW);
    interrupts();
    if (v & 0x800000) v |= 0xFF000000;  // sign-extend 24 -> 32 bit
    return (int32_t)v;
  }

private:
  uint8_t _dout = 0, _sck = 0;
};
