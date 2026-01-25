/**
 * SevSegShift - 7-Segment Display Library with Shift Register Support
 *
 * Fork of SevSeg library adapted for daisy-chained shift registers.
 * Uses optimized assembly routine for shift register timing.
 */

#include "SevSegShift.h"

// External assembly function for fast shift register output
extern "C" void shiftOut16_asm(uint8_t segments, uint8_t digitSelect);

// Segment patterns (bit order: DP G F E D C B A)
static const uint8_t SEGMENT_MAP[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111, // 9
    0b01110111, // A
    0b01111100, // b
    0b00111001, // C
    0b01011110, // d
    0b01111001, // E
    0b01110001, // F
};

static const uint8_t CHAR_BLANK = 0b00000000;
static const uint8_t CHAR_DASH  = 0b01000000;

SevSegShift::SevSegShift() {
    _numDigits = 4;
    _currentDigit = 0;
    _decimalPoint = 0;
    _refreshDelay = 2000; // 2ms default
}

void SevSegShift::begin(uint8_t displayType, uint8_t numDigits,
                        uint8_t dataPin, uint8_t clockPin, uint8_t latchPin,
                        bool leadingZeros) {
    _displayType = displayType;
    _numDigits = (numDigits > MAX_DIGITS) ? MAX_DIGITS : numDigits;
    _dataPin = dataPin;
    _clockPin = clockPin;
    _latchPin = latchPin;
    _leadingZeros = leadingZeros;

    pinMode(_dataPin, OUTPUT);
    pinMode(_clockPin, OUTPUT);
    pinMode(_latchPin, OUTPUT);

    // Inverted polarity - all signals idle HIGH
    digitalWrite(_dataPin, HIGH);
    digitalWrite(_clockPin, HIGH);
    digitalWrite(_latchPin, HIGH);

    blank();
}

void SevSegShift::shiftOut16(uint8_t segments, uint8_t digitSelect) {
    // Invert segments for common anode display
    if (_displayType == COMMON_ANODE) {
        segments = ~segments;
    }

    // Use optimized assembly routine for precise timing
    shiftOut16_asm(segments, digitSelect);
}

void SevSegShift::refreshDisplay() {
    uint8_t segments = _digitCodes[_currentDigit];

    // Add decimal point if this is the right digit
    if (_decimalPoint > 0 && _currentDigit == (_numDigits - _decimalPoint)) {
        segments |= 0b10000000;
    }

    // Create digit select pattern (active LOW for PNP transistors)
    uint8_t digitSelect = ~(1 << _currentDigit);

    shiftOut16(segments, digitSelect);

    // Move to next digit
    _currentDigit = (_currentDigit + 1) % _numDigits;

    delayMicroseconds(_refreshDelay);
}

void SevSegShift::setNumber(int32_t num, int8_t decPlace) {
    bool negative = (num < 0);
    if (negative) num = -num;

    _decimalPoint = decPlace;

    // Clear display buffer
    for (uint8_t i = 0; i < _numDigits; i++) {
        _digitCodes[i] = CHAR_BLANK;
    }

    // Fill digits from right to left
    uint8_t pos = _numDigits - 1;
    if (num == 0) {
        _digitCodes[pos] = SEGMENT_MAP[0];
    } else {
        while (num > 0 && pos < _numDigits) {
            _digitCodes[pos] = SEGMENT_MAP[num % 10];
            num /= 10;
            if (num > 0 || _leadingZeros) pos--;
        }
    }

    // Add negative sign if needed
    if (negative && pos > 0) {
        pos--;
        _digitCodes[pos] = CHAR_DASH;
    }

    // Fill leading zeros if enabled
    if (_leadingZeros) {
        while (pos > 0) {
            pos--;
            _digitCodes[pos] = SEGMENT_MAP[0];
        }
    }
}

void SevSegShift::setNumberF(float num, int8_t decPlaces) {
    // Multiply by power of 10 to shift decimal places
    int32_t multiplier = 1;
    for (int8_t i = 0; i < decPlaces; i++) {
        multiplier *= 10;
    }

    int32_t intNum = (int32_t)(num * multiplier + (num >= 0 ? 0.5f : -0.5f));
    setNumber(intNum, decPlaces);
}

void SevSegShift::setChars(const char* str) {
    _decimalPoint = 0;

    for (uint8_t i = 0; i < _numDigits; i++) {
        if (str[i] == '\0') {
            // Fill remaining with blanks
            for (; i < _numDigits; i++) {
                _digitCodes[i] = CHAR_BLANK;
            }
            break;
        }
        _digitCodes[i] = charToSegment(str[i]);
    }
}

uint8_t SevSegShift::charToSegment(char c) {
    if (c >= '0' && c <= '9') return SEGMENT_MAP[c - '0'];
    if (c >= 'A' && c <= 'F') return SEGMENT_MAP[c - 'A' + 10];
    if (c >= 'a' && c <= 'f') return SEGMENT_MAP[c - 'a' + 10];
    if (c == '-') return CHAR_DASH;
    if (c == ' ') return CHAR_BLANK;
    return CHAR_BLANK;
}

void SevSegShift::blank() {
    for (uint8_t i = 0; i < _numDigits; i++) {
        _digitCodes[i] = CHAR_BLANK;
    }
    _decimalPoint = 0;
    shiftOut16(0x00, 0xFF); // All off
}

void SevSegShift::setBrightness(uint8_t brightness) {
    // Adjust refresh delay (lower = brighter due to faster multiplexing)
    // Range: 500us (bright) to 5000us (dim)
    _refreshDelay = 5000 - (brightness * 45);
    if (_refreshDelay < 500) _refreshDelay = 500;
}
