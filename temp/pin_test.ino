/**
 * Pin Test - Toggle each shift register pin slowly
 * Use a multimeter or LED to verify signals on each pin
 */

#define DATA_PIN   13   // PB5
#define CLOCK_PIN  11   // PB3 (CP)
#define LATCH_PIN  5    // PD5 (STR)

void setup() {
    pinMode(DATA_PIN, OUTPUT);
    pinMode(CLOCK_PIN, OUTPUT);
    pinMode(LATCH_PIN, OUTPUT);

    digitalWrite(DATA_PIN, LOW);
    digitalWrite(CLOCK_PIN, LOW);
    digitalWrite(LATCH_PIN, LOW);
}

void loop() {
    // Toggle DATA pin (PB3) - 1 second cycle
    digitalWrite(DATA_PIN, HIGH);
    delay(500);
    digitalWrite(DATA_PIN, LOW);
    delay(500);

    // Toggle CLOCK pin (PB5) - 1 second cycle
    digitalWrite(CLOCK_PIN, HIGH);
    delay(500);
    digitalWrite(CLOCK_PIN, LOW);
    delay(500);

    // Toggle LATCH pin (PD5) - 1 second cycle
    digitalWrite(LATCH_PIN, HIGH);
    delay(500);
    digitalWrite(LATCH_PIN, LOW);
    delay(500);

    // Pause before repeating
    delay(1000);
}
