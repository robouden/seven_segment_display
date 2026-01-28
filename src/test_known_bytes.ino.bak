/**
 * Test sketch - sends exact bytes captured from working MCU
 * Should display "Er.4" with heater LED
 *
 * Captured byte pairs from scope:
 *   0x97, 0x01  (or inverted...)
 *   0x00, 0x00
 *   0xDA, 0x02
 *   etc.
 *
 * From Er.4 display capture:
 *   Byte1 (segments), Byte2 (digit/LEDs)
 */

#include <Arduino.h>

// Pin definitions (directly manipulating ports)
#define DATA_PIN  PB3   // Pin 11 on ATmega8
#define CLOCK_PIN PB5   // Pin 13 on ATmega8
#define LATCH_PIN PD5   // Pin 5 on ATmega8

// Declare external assembly function
extern "C" void shiftOut16_asm(uint8_t segments, uint8_t digitSelect);

// Manual bit-bang version for comparison
void shiftOut16_manual(uint8_t segments, uint8_t digitSelect) {
    // Ensure idle state (both HIGH)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);

    // PRE-SEQUENCE: Data LOW first, then Clock LOW
    // This signals start of word (data LOW before clock goes LOW)
    // Step 1: Data LOW (clock still HIGH)
    PORTB &= ~(1 << DATA_PIN);
    delayMicroseconds(16);

    // Step 2: Clock LOW (data still LOW) - ready for first bit
    PORTB &= ~(1 << CLOCK_PIN);
    delayMicroseconds(16);

    // Shift out first byte - MSB first (NO inversion - values are raw wire values)
    for (int8_t i = 7; i >= 0; i--) {
        // Clock is LOW - set data directly (no inversion)
        if ((segments >> i) & 0x01) {
            PORTB |= (1 << DATA_PIN);   // Bit 1 -> HIGH
        } else {
            PORTB &= ~(1 << DATA_PIN);  // Bit 0 -> LOW
        }
        delayMicroseconds(2);  // Data setup time

        // Clock HIGH (rising edge samples data)
        PORTB |= (1 << CLOCK_PIN);
        delayMicroseconds(40);

        // Clock LOW (prepare for next bit)
        PORTB &= ~(1 << CLOCK_PIN);
        delayMicroseconds(20);
    }

    // Shift out second byte - MSB first (NO inversion - values are raw wire values)
    for (int8_t i = 7; i >= 0; i--) {
        // Clock is LOW - set data directly (no inversion)
        if ((digitSelect >> i) & 0x01) {
            PORTB |= (1 << DATA_PIN);   // Bit 1 -> HIGH
        } else {
            PORTB &= ~(1 << DATA_PIN);  // Bit 0 -> LOW
        }
        delayMicroseconds(2);  // Data setup time

        // Clock HIGH (rising edge samples data)
        PORTB |= (1 << CLOCK_PIN);
        delayMicroseconds(40);

        // Clock LOW (prepare for next bit)
        PORTB &= ~(1 << CLOCK_PIN);
        delayMicroseconds(20);
    }

    // Return to idle HIGH
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);

    // Latch pulse
    delayMicroseconds(2);
    PORTD &= ~(1 << LATCH_PIN);  // Latch LOW
    delayMicroseconds(40);
    PORTD |= (1 << LATCH_PIN);   // Latch HIGH
}

// Known byte pairs from working MCU capture (Er.4 display)
// Format: {segments, digitSelect}
// These are the RAW values before any inversion in shiftOut
const uint8_t testPatterns[][2] = {
    {0x97, 0x01},  // From scope capture
    {0x00, 0x00},
    {0xDA, 0x02},
    {0x04, 0x97},
};

// Alternative: Try the values we decoded for Er.4
// Digit cathodes: QP5=left, QP7=middle, QP6=right (active LOW via PNP)
// Segment patterns from readme:
//   E = 0b10010111 = 0x97
//   r. = 0b10100010 = 0xA2
//   4 = 0b11001100 = 0xCC

void setup() {
    // Set pin modes
    DDRB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    DDRD |= (1 << LATCH_PIN);

    // Set idle state (all HIGH)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    PORTD |= (1 << LATCH_PIN);
}

void loop() {
    // Test pattern: Display "Er.4" by cycling through digits
    // Using manual function for testing

    // Left digit: 'E' with cathode 1 selected
    // Cathode 1 = QP5 = bit 5, active LOW = ~(1<<5) = 0xDF with heater = 0xDF & ~0x01 or with heater = varies
    // Let's try the exact captured values first

    // From the scope, we saw pairs like:
    // 0xA1, 0xFB from first capture
    // Let me try sending simple known values

    // Simple test: All segments ON, digit 1 selected
    // Segments all ON = 0xFF (before inversion)
    // Digit 1 (left) = QP5 LOW = bit 5 = 0, others HIGH = 0b11011111 = 0xDF

    static uint8_t digit = 0;
    static uint8_t heaterState = 0;  // Toggle for flashing

    // Correct byte values from working MCU scope capture
    // Format: {byte1 (digit/LEDs), byte2 (segments)}
    // Heater LED = bit 1 (0x02) in byte1

    uint8_t byte1, byte2;

    switch (digit) {
        case 0:  // E on left digit
            byte1 = 0xE9;
            byte2 = 0xC2;
            break;
        case 1:  // r on middle digit (no decimal point)
            byte1 = 0x41;
            byte2 = 0x62;
            break;
        case 2:  // 4 on right digit
            byte1 = 0x33;
            byte2 = 0xA2;
            break;
    }

    // NOTE: byte order is swapped - byte1 first, then byte2
    shiftOut16_manual(byte1, byte2);
    // shiftOut16_asm(byte1, byte2);

    digit++;
    if (digit >= 3) {
        digit = 0;
        heaterState = !heaterState;  // Toggle heater every full cycle
    }

    delayMicroseconds(5000);  // 5ms refresh
}
