/**
 * Temperature Display Example
 * Displays temperature with 1 decimal place (e.g., 23.5)
 */

#include <SevSegShift.h>

#define DATA_PIN   5
#define CLOCK_PIN  6
#define LATCH_PIN  7

SevSegShift display;

// Replace with your actual sensor reading function
float readSensor() {
    // Simulated temperature reading
    static float temp = 20.0;
    temp += 0.1;
    if (temp > 40.0) temp = 20.0;
    return temp;
}

void displayTemperature(float tempC) {
    display.setNumberF(tempC, 1);
}

void setup() {
    display.begin(COMMON_ANODE, 4, DATA_PIN, CLOCK_PIN, LATCH_PIN);
    displayTemperature(0.0);
}

void loop() {
    display.refreshDisplay();

    static unsigned long lastRead = 0;
    if (millis() - lastRead >= 2000) {
        lastRead = millis();
        float temp = readSensor();
        displayTemperature(temp);
    }
}
