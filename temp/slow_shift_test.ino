/**
 * Scope-Friendly Shift Register Test
 *
 * Connections for oscilloscope:
 *   CH1 (Yellow) -> CLOCK (PB5, ATmega8 pin 19)
 *   CH2 (Blue)   -> DATA  (PB3, ATmega8 pin 17)
 *   EXT TRIG or CH3 -> STROBE (PD5, ATmega8 pin 11) [optional]
 *   GND clip -> Board GND
 *
 * Scope settings:
 *   Timebase: 20ms/div (shows full byte transfer)
 *   Voltage: 2V/div
 *   Trigger: CH1 rising edge, ~2.5V
 *   Mode: Normal (not Auto)
 */

#define DATA_PIN   13   // PB5 - swapped
#define CLOCK_PIN  11   // PB3 (CP) - swapped
#define LATCH_PIN  5    // PD5 (STR)

// Bit timing - use microseconds for realistic speed
// Good capture showed ~5.47kHz = ~183µs per clock cycle
#define BIT_DELAY_US  50   // 50µs per phase (~6.6kHz clock)

void setup() {
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);

    // All signals inverted - idle HIGH
    digitalWrite(DATA_PIN, HIGH);
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(LATCH_PIN, HIGH);
}

// Shift out one byte with INVERTED polarity on all signals
void scopeShiftOut(uint8_t data) {
    digitalWrite(CLOCK_PIN, HIGH);  // Start with clock HIGH
    delayMicroseconds(BIT_DELAY_US);

    for (int i = 7; i >= 0; i--) {
        // Set data bit - INVERTED (1 = LOW, 0 = HIGH)
        uint8_t bit = (data >> i) & 1;
        digitalWrite(DATA_PIN, !bit);  // Inverted
        delayMicroseconds(BIT_DELAY_US);

        // Clock LOW - data is sampled on falling edge
        digitalWrite(CLOCK_PIN, LOW);
        delayMicroseconds(BIT_DELAY_US);

        // Clock HIGH
        digitalWrite(CLOCK_PIN, HIGH);
        delayMicroseconds(BIT_DELAY_US);
    }

    // Small gap before latch
    delayMicroseconds(BIT_DELAY_US);

    // Latch pulse - INVERTED (active LOW)
    digitalWrite(LATCH_PIN, LOW);
    delayMicroseconds(BIT_DELAY_US);
    digitalWrite(LATCH_PIN, HIGH);
}

void loop() {
    // === SYNC PULSE === (use this to trigger scope)
    // 3 quick pulses on LATCH to mark start of sequence
    for (int i = 0; i < 3; i++) {
        digitalWrite(LATCH_PIN, HIGH);
        delay(5);
        digitalWrite(LATCH_PIN, LOW);
        delay(5);
    }
    delay(100);

    // === TEST PATTERN 1: 0xA5 = 10100101 ===
    // Easy to identify: two 1s, two 0s, one 0, one 1, zero, one
    // On scope: HIGH-LOW-HIGH-LOW-LOW-HIGH-LOW-HIGH
    scopeShiftOut(0xA5);
    delay(500);

    // === TEST PATTERN 2: 0x0F = 00001111 ===
    // First half LOW, second half HIGH - very distinctive
    scopeShiftOut(0x0F);
    delay(500);

    // === TEST PATTERN 3: 0xF0 = 11110000 ===
    // Opposite of above - first half HIGH, second half LOW
    scopeShiftOut(0xF0);
    delay(500);

    // === TEST PATTERN 4: 0x55 = 01010101 ===
    // Perfect alternating - should look like square wave on DATA
    scopeShiftOut(0x55);
    delay(500);

    // Long pause before repeating
    delay(1000);
}
