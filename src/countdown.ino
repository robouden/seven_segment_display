/**
 * Countdown Timer Example
 * Counts down from 60 seconds, displays "End" when finished
 */

#include <SevSegShift.h>

#define DATA_PIN 13  // PB5 - swapped
#define CLOCK_PIN 11 // PB3 (CP) - swapped
#define LATCH_PIN 5  // PD5 (STR)

// Bit timing - use microseconds for realistic speed
// Good capture showed ~5.47kHz = ~183µs per clock cycle
#define BIT_DELAY_US 50 // 50µs per phase (~6.6kHz clock)

SevSegShift display;

void setup()
{
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);

    // All signals inverted - idle HIGH
    digitalWrite(DATA_PIN, HIGH);
    digitalWrite(CLOCK_PIN, HIGH);
    digitalWrite(LATCH_PIN, HIGH);

    display.begin(COMMON_ANODE, 3, DATA_PIN, CLOCK_PIN, LATCH_PIN);
    display.setNumber(60);
}

void loop()
{
    static int16_t seconds = 60;
    static unsigned long lastTick = 0;

    display.refreshDisplay();

    if (millis() - lastTick >= 1000 && seconds > 0)
    {
        lastTick = millis();
        seconds--;
        display.setNumber(seconds);
    }

    if (seconds == 0)
    {
        display.setChars("End");
    }
}
