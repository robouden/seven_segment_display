/**
 * Hex Display Example
 * Cycles through hex words: CAFE, DEAD, BEEF, FACE
 */

#include <SevSegShift.h>

#define DATA_PIN   5
#define CLOCK_PIN  6
#define LATCH_PIN  7

SevSegShift display;

const char* hexWords[] = {"CAFE", "DEAD", "BEEF", "FACE"};
const uint8_t numWords = 4;

void setup() {
    display.begin(COMMON_ANODE, 4, DATA_PIN, CLOCK_PIN, LATCH_PIN);
    display.setChars(hexWords[0]);
}

void loop() {
    static uint8_t index = 0;
    static unsigned long lastChange = 0;

    display.refreshDisplay();

    if (millis() - lastChange >= 2000) {
        lastChange = millis();
        index = (index + 1) % numWords;
        display.setChars(hexWords[index]);
    }
}
