#pragma once

#include <Arduino.h>
#include "BoardConfig.h"

// Reads the board-address rotary dial (SW1). Only weight-1/2/4 are wired, so
// the readable range is 0-7 even though the switch itself has 10 positions.
inline uint8_t readBoardAddress() {
    pinMode(PIN_DIAL_BIT1, INPUT_PULLUP);
    pinMode(PIN_DIAL_BIT2, INPUT_PULLUP);
    pinMode(PIN_DIAL_BIT4, INPUT_PULLUP);
    // TODO(verify on hardware): assumes the switch pulls a bit low when set
    // (active-low common), matching INPUT_PULLUP idle-high wiring.
    uint8_t value = 0;
    if (!digitalRead(PIN_DIAL_BIT1)) value |= 0x01;
    if (!digitalRead(PIN_DIAL_BIT2)) value |= 0x02;
    if (!digitalRead(PIN_DIAL_BIT4)) value |= 0x04;
    return value;
}
