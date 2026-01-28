/**
 * Spa Control Firmware
 *
 * Controls temperature and displays sensor status on 7-segment display.
 * ATmega8A-AU with HEF4094B shift registers.
 *
 * Hardware:
 *   - 3-digit 7-segment display (common anode)
 *   - 2x HEF4094B shift registers (daisy-chained)
 *   - Temperature sensor on ADC
 *   - Status LEDs: Heater, Auto, Air, Pump
 *   - Relay outputs for heater and pump control
 *
 * See spa_config.h for pin assignments and configuration options.
 */

#include <Arduino.h>
#include <math.h>
#include "spa_config.h"

// ============================================================================
// TEST MODE - uncomment to use exact bytes from working test_known_bytes.ino
// ============================================================================
#define TEST_MODE  // Comment out to use normal display logic

// ============================================================================
// SEGMENT PATTERNS
// ============================================================================

// Segment map (bit order: G C DP D B F E A - specific to this hardware)
static const uint8_t SEGMENT_MAP[] = {
    0b01011111, // 0
    0b01001000, // 1
    0b10011011, // 2
    0b11011001, // 3
    0b11001100, // 4
    0b11010101, // 5
    0b11010111, // 6
    0b01001001, // 7
    0b11011111, // 8
    0b11011101, // 9
    0b11001111, // A
    0b11010110, // b
    0b00010111, // C
    0b11011010, // d
    0b10010111, // E
    0b10000111, // F
};

static const uint8_t CHAR_BLANK = 0b00000000;
static const uint8_t CHAR_DASH  = 0b10000000;
static const uint8_t CHAR_r     = 0b10000010;  // 'r' for error display
static const uint8_t CHAR_H     = 0b11001110;  // 'H' for Heat
static const uint8_t CHAR_t     = 0b10110110;  // 't' for Heat
static const uint8_t CHAR_DP    = 0b00100000;  // Decimal point bit

// ============================================================================
// SYSTEM STATE
// ============================================================================

// Operating modes
enum SystemMode {
    MODE_NORMAL,
    MODE_ERROR,
    MODE_STANDBY
};

// Current system state
struct {
    // Temperature
    float currentTemp;          // Current temperature reading
    float setpoint;             // Target temperature
    float lastGoodTemp;         // Last valid temperature reading

    // Control states
    bool heaterOn;              // Heater relay state
    bool pumpOn;                // Pump relay state
    bool airOn;                 // Air jets state (future use)
    bool autoMode;              // Auto temperature control enabled

    // Error handling
    uint8_t sensorErrorCount;   // Consecutive sensor read errors
    SystemMode mode;            // Current operating mode

    // Display
    uint8_t digitCodes[NUM_DIGITS];  // Segment patterns for each digit
    uint8_t decimalPosition;         // Decimal point position (0=none)
    uint8_t currentDigit;            // Current digit being refreshed
    uint8_t ledStatus;               // LED status bits

    // Timing
    unsigned long lastSensorRead;    // Last sensor read timestamp
    unsigned long lastDisplayUpdate; // Last display content update

} state;

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

void initializeSystem();
void readSensors();
void processTemperature();
void controlTemperature();
void updateDisplay();
void refreshDisplay();
void shiftOut16(uint8_t byte1, uint8_t byte2);
void setDisplayNumber(float num, uint8_t decPlaces);
void setDisplayChars(const char* str);
void setDisplayError(uint8_t errorCode);
void blankDisplay();
void updateLEDs();
void enterErrorState(uint8_t errorCode);
void exitErrorState();
float readTemperatureSensor();
uint8_t getCathodeBit(uint8_t digit);

// ============================================================================
// INITIALIZATION
// ============================================================================

#ifdef TEST_MODE
// Minimal setup for test mode (matches test_known_bytes.ino exactly)
void setup() {
    // Set pin modes
    DDRB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    DDRD |= (1 << LATCH_PIN);

    // Set idle state (all HIGH)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    PORTD |= (1 << LATCH_PIN);
}
#else
void setup() {
    initializeSystem();
}
#endif

void initializeSystem() {
    // Initialize state
    state.currentTemp = 0.0f;
    state.setpoint = DEFAULT_SETPOINT;
    state.lastGoodTemp = 25.0f;  // Assume room temp initially
    state.heaterOn = false;
    state.pumpOn = false;
    state.airOn = false;
    state.autoMode = true;
    state.sensorErrorCount = 0;
    state.mode = MODE_NORMAL;
    state.currentDigit = 0;
    state.ledStatus = 0;
    state.lastSensorRead = 0;
    state.lastDisplayUpdate = 0;
    state.decimalPosition = 0;

    // Clear display buffer
    for (uint8_t i = 0; i < NUM_DIGITS; i++) {
        state.digitCodes[i] = CHAR_BLANK;
    }

    // Configure shift register pins as outputs
    DDRB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    DDRD |= (1 << LATCH_PIN);

    // Set shift register idle state (all HIGH)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);
    PORTD |= (1 << LATCH_PIN);

    // Configure digital inputs with pull-ups
    DDRD &= ~((1 << PUMP_SWITCH_PIN) | (1 << AIR_SWITCH_PIN) | (1 << AUTO_SWITCH_PIN));
    PORTD |= (1 << PUMP_SWITCH_PIN) | (1 << AIR_SWITCH_PIN) | (1 << AUTO_SWITCH_PIN);

    // Configure relay outputs
    DDRD |= (1 << HEATER_RELAY_PIN) | (1 << PUMP_RELAY_PIN);
    PORTD &= ~((1 << HEATER_RELAY_PIN) | (1 << PUMP_RELAY_PIN));  // Start with relays off

    // Configure ADC
    ADMUX = (1 << REFS0);  // AVcc reference, ADC0 selected
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, prescaler 64

    // Initial display
    blankDisplay();

    // Read initial sensor value
    state.currentTemp = readTemperatureSensor();
    if (state.currentTemp > 0) {
        state.lastGoodTemp = state.currentTemp;
    }
}

// ============================================================================
// MAIN LOOP
// ============================================================================

#ifdef TEST_MODE
// Test mode: Use exact bytes from working test_known_bytes.ino
void loop() {
    static uint8_t digit = 0;

    uint8_t byte1, byte2;

    // Exact values from working test_known_bytes.ino for "Er.4" display
    switch (digit) {
        case 0:  // E on left digit
            byte1 = 0xE9;  // digit/LEDs
            byte2 = 0xC2;  // segments
            break;
        case 1:  // r on middle digit
            byte1 = 0x41;
            byte2 = 0x62;
            break;
        case 2:  // 4 on right digit
            byte1 = 0x33;
            byte2 = 0xA2;
            break;
        default:
            byte1 = 0x00;
            byte2 = 0x00;
            break;
    }

    // Call shiftOut16 exactly like test_known_bytes.ino does
    shiftOut16(byte1, byte2);

    digit++;
    if (digit >= 3) {
        digit = 0;
    }

    delayMicroseconds(5000);  // 5ms refresh (same as test code)
}

#else
// Normal mode: Full spa control functionality
void loop() {
    unsigned long currentTime = millis();

    // ---- Sensor Reading (every SENSOR_READ_INTERVAL ms) ----
    if (currentTime - state.lastSensorRead >= SENSOR_READ_INTERVAL) {
        state.lastSensorRead = currentTime;

        // Read all sensors
        readSensors();

        // Process temperature data
        processTemperature();

        // Run temperature control logic
        controlTemperature();

        // Update display content
        updateDisplay();

        // Update LED states
        updateLEDs();
    }

    // ---- Display Multiplexing (continuous) ----
    refreshDisplay();
}
#endif

// ============================================================================
// SENSOR READING
// ============================================================================

void readSensors() {
    // Read temperature sensor
    float tempReading = readTemperatureSensor();

    // Validate reading
    if (tempReading < -10.0f || tempReading > 100.0f) {
        // Invalid reading
        state.sensorErrorCount++;

        if (state.sensorErrorCount >= SENSOR_ERROR_THRESHOLD) {
            enterErrorState(1);  // Error code 1: Sensor fault
        }
    } else {
        // Valid reading
        state.currentTemp = tempReading;
        state.lastGoodTemp = tempReading;
        state.sensorErrorCount = 0;

        // Exit error state if we were in one due to sensor issues
        if (state.mode == MODE_ERROR) {
            exitErrorState();
        }
    }

    // Read digital inputs (active LOW with pull-ups)
    bool pumpSwitch = !(PIND & (1 << PUMP_SWITCH_PIN));
    bool airSwitch = !(PIND & (1 << AIR_SWITCH_PIN));
    bool autoSwitch = !(PIND & (1 << AUTO_SWITCH_PIN));

    // Update states based on switch positions
    state.pumpOn = pumpSwitch;
    state.airOn = airSwitch;
    state.autoMode = autoSwitch;
}

float readTemperatureSensor() {
    uint32_t adcSum = 0;

    // Take multiple samples for averaging
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        // Select ADC0 channel
        ADMUX = (ADMUX & 0xF0) | 0x00;

        // Start conversion
        ADCSRA |= (1 << ADSC);

        // Wait for conversion to complete
        while (ADCSRA & (1 << ADSC));

        // Read result
        adcSum += ADC;
    }

    // Calculate average
    uint16_t adcAvg = adcSum / ADC_SAMPLES;

    float temperature;

#ifdef SENSOR_TYPE_NTC_10K
    // NTC Thermistor calculation using Beta equation
    // R = Rpullup * (ADC / (1023 - ADC))
    // T = 1 / (1/T0 + (1/Beta) * ln(R/R0))

    if (adcAvg < 10 || adcAvg > 1013) {
        // Invalid reading (open or shorted sensor)
        return -999.0f;
    }

    float resistance = NTC_PULLUP_RESISTANCE * ((float)adcAvg / (1023.0f - (float)adcAvg));

    // Simplified Beta equation
    float steinhart = log(resistance / NTC_RESISTANCE_25C) / NTC_BETA;
    steinhart += 1.0f / NTC_REFERENCE_TEMP;
    temperature = (1.0f / steinhart) - 273.15f;  // Convert Kelvin to Celsius

#else  // SENSOR_TYPE_LINEAR (default)
    // Linear voltage output sensor (TMP36, LM35, etc.)
    float voltage_mv = (adcAvg * 5000.0f) / 1024.0f;  // Convert to millivolts
    temperature = (voltage_mv - LINEAR_OFFSET_MV) / LINEAR_MV_PER_DEGREE;
#endif

    // Apply calibration adjustments
    temperature = (temperature * TEMP_SCALE) + TEMP_OFFSET;

    return temperature;
}

// ============================================================================
// TEMPERATURE PROCESSING
// ============================================================================

void processTemperature() {
    // Apply moving average filter (simple exponential smoothing)
    static float filteredTemp = 25.0f;
    const float alpha = 0.3f;  // Smoothing factor (0-1, lower = more smoothing)

    filteredTemp = alpha * state.currentTemp + (1.0f - alpha) * filteredTemp;
    state.currentTemp = filteredTemp;

    // Safety check - immediate shutdown if over max safe temp
    if (state.currentTemp > MAX_TEMP_SAFE) {
        state.heaterOn = false;
        PORTD &= ~(1 << HEATER_RELAY_PIN);  // Immediate relay off
        enterErrorState(2);  // Error code 2: Over temperature
    }
}

// ============================================================================
// TEMPERATURE CONTROL
// ============================================================================

void controlTemperature() {
    // Skip control if in error state or not in auto mode
    if (state.mode == MODE_ERROR || !state.autoMode) {
        // In manual mode or error, ensure heater is off for safety
        if (state.mode == MODE_ERROR) {
            state.heaterOn = false;
        }
        return;
    }

    // Hysteresis control logic
    float tempLow = state.setpoint - TEMP_HYSTERESIS;
    float tempHigh = state.setpoint + TEMP_HYSTERESIS;

    if (state.currentTemp < tempLow) {
        // Temperature too low - turn heater ON
        state.heaterOn = true;
    } else if (state.currentTemp > tempHigh) {
        // Temperature too high - turn heater OFF
        state.heaterOn = false;
    }
    // Within hysteresis band - maintain current state

    // Apply relay outputs
    if (state.heaterOn) {
        PORTD |= (1 << HEATER_RELAY_PIN);
    } else {
        PORTD &= ~(1 << HEATER_RELAY_PIN);
    }

    if (state.pumpOn) {
        PORTD |= (1 << PUMP_RELAY_PIN);
    } else {
        PORTD &= ~(1 << PUMP_RELAY_PIN);
    }
}

// ============================================================================
// DISPLAY FUNCTIONS
// ============================================================================

void updateDisplay() {
    if (state.mode == MODE_ERROR) {
        // Display error code
        // Already set by enterErrorState()
        return;
    }

    // Display current temperature
    setDisplayNumber(state.currentTemp, 1);  // One decimal place
}

void refreshDisplay() {
    // Get segment pattern for current digit
    uint8_t segments = state.digitCodes[state.currentDigit];

    // Add decimal point if this is the right digit
    if (state.decimalPosition > 0 &&
        state.currentDigit == (NUM_DIGITS - state.decimalPosition)) {
        segments |= CHAR_DP;
    }

    // Create digit select byte
    // Start with all cathodes HIGH (off) and LED status
    uint8_t digitSelect = 0b11100000 | state.ledStatus;

    // Set current cathode LOW (active)
    uint8_t cathodeBit = getCathodeBit(state.currentDigit);
    digitSelect &= ~(1 << cathodeBit);

    // Send to shift registers
    // NOTE: digitSelect sent FIRST, then segments (matches working test code)
    shiftOut16(digitSelect, segments);

    // Move to next digit
    state.currentDigit = (state.currentDigit + 1) % NUM_DIGITS;

    // Delay for persistence of vision
    delayMicroseconds(REFRESH_DELAY_US);
}

uint8_t getCathodeBit(uint8_t digit) {
    // Map digit index to cathode bit position
    switch (digit) {
        case 0: return CATHODE_LEFT_BIT;    // Left digit
        case 1: return CATHODE_MIDDLE_BIT;  // Middle digit
        case 2: return CATHODE_RIGHT_BIT;   // Right digit
        default: return CATHODE_LEFT_BIT;
    }
}

void shiftOut16(uint8_t byte1, uint8_t byte2) {
    // Ensure idle state (both HIGH)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);

    // PRE-SEQUENCE: Data LOW first, then Clock LOW
    // This signals start of word (data LOW before clock goes LOW)
    PORTB &= ~(1 << DATA_PIN);
    delayMicroseconds(16);

    PORTB &= ~(1 << CLOCK_PIN);
    delayMicroseconds(16);

    // Shift out first byte - MSB first (matches working test code)
    for (int8_t i = 7; i >= 0; i--) {
        if ((byte1 >> i) & 0x01) {
            PORTB |= (1 << DATA_PIN);
        } else {
            PORTB &= ~(1 << DATA_PIN);
        }
        delayMicroseconds(2);

        PORTB |= (1 << CLOCK_PIN);   // Rising edge samples data
        delayMicroseconds(40);

        PORTB &= ~(1 << CLOCK_PIN);  // Falling edge
        delayMicroseconds(20);
    }

    // Shift out second byte - MSB first (matches working test code)
    for (int8_t i = 7; i >= 0; i--) {
        if ((byte2 >> i) & 0x01) {
            PORTB |= (1 << DATA_PIN);
        } else {
            PORTB &= ~(1 << DATA_PIN);
        }
        delayMicroseconds(2);

        PORTB |= (1 << CLOCK_PIN);
        delayMicroseconds(40);

        PORTB &= ~(1 << CLOCK_PIN);
        delayMicroseconds(20);
    }

    // Return to idle HIGH (matches working test code)
    PORTB |= (1 << DATA_PIN) | (1 << CLOCK_PIN);

    // Latch pulse (active LOW) - transfers shift register to outputs
    delayMicroseconds(2);
    PORTD &= ~(1 << LATCH_PIN);
    delayMicroseconds(40);
    PORTD |= (1 << LATCH_PIN);
}

void setDisplayNumber(float num, uint8_t decPlaces) {
    // Handle negative numbers
    bool negative = (num < 0);
    if (negative) num = -num;

    // Store decimal position
    state.decimalPosition = decPlaces;

    // Convert to integer with decimal places
    int32_t multiplier = 1;
    for (uint8_t i = 0; i < decPlaces; i++) {
        multiplier *= 10;
    }
    int32_t intNum = (int32_t)(num * multiplier + 0.5f);

    // Clear display buffer
    for (uint8_t i = 0; i < NUM_DIGITS; i++) {
        state.digitCodes[i] = CHAR_BLANK;
    }

    // Fill digits from right to left
    uint8_t pos = NUM_DIGITS - 1;
    if (intNum == 0) {
        state.digitCodes[pos] = SEGMENT_MAP[0];
    } else {
        while (intNum > 0 && pos < NUM_DIGITS) {
            state.digitCodes[pos] = SEGMENT_MAP[intNum % 10];
            intNum /= 10;
            if (intNum > 0) pos--;
        }
    }

    // Add negative sign if needed
    if (negative && pos > 0) {
        pos--;
        state.digitCodes[pos] = CHAR_DASH;
    }
}

void setDisplayChars(const char* str) {
    state.decimalPosition = 0;

    for (uint8_t i = 0; i < NUM_DIGITS && str[i] != '\0'; i++) {
        char c = str[i];

        if (c >= '0' && c <= '9') {
            state.digitCodes[i] = SEGMENT_MAP[c - '0'];
        } else if (c >= 'A' && c <= 'F') {
            state.digitCodes[i] = SEGMENT_MAP[c - 'A' + 10];
        } else if (c >= 'a' && c <= 'f') {
            state.digitCodes[i] = SEGMENT_MAP[c - 'a' + 10];
        } else if (c == '-') {
            state.digitCodes[i] = CHAR_DASH;
        } else if (c == 'r' || c == 'R') {
            state.digitCodes[i] = CHAR_r;
        } else if (c == 'H') {
            state.digitCodes[i] = CHAR_H;
        } else if (c == 't') {
            state.digitCodes[i] = CHAR_t;
        } else if (c == 'E' || c == 'e') {
            state.digitCodes[i] = SEGMENT_MAP[14];  // 'E'
        } else {
            state.digitCodes[i] = CHAR_BLANK;
        }
    }
}

void setDisplayError(uint8_t errorCode) {
    // Display "Er#" where # is error code
    state.digitCodes[0] = SEGMENT_MAP[14];  // 'E'
    state.digitCodes[1] = CHAR_r;           // 'r'
    state.digitCodes[2] = SEGMENT_MAP[errorCode % 10];
    state.decimalPosition = 0;
}

void blankDisplay() {
    for (uint8_t i = 0; i < NUM_DIGITS; i++) {
        state.digitCodes[i] = CHAR_BLANK;
    }
    state.decimalPosition = 0;
}

// ============================================================================
// LED CONTROL
// ============================================================================

void updateLEDs() {
    state.ledStatus = 0;

    // Set LED bits based on current state
    if (state.heaterOn) {
        state.ledStatus |= (1 << LED_HEATER_BIT);
    }

    if (state.autoMode) {
        state.ledStatus |= (1 << LED_AUTO_BIT);
    }

    if (state.airOn) {
        state.ledStatus |= (1 << LED_AIR_BIT);
    }

    if (state.pumpOn) {
        state.ledStatus |= (1 << LED_PUMP_BIT);
    }
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

void enterErrorState(uint8_t errorCode) {
    state.mode = MODE_ERROR;

    // Safety shutdown - turn off heater immediately
    state.heaterOn = false;
    PORTD &= ~(1 << HEATER_RELAY_PIN);

    // Display error code
    setDisplayError(errorCode);

    // Flash heater LED to indicate error
    state.ledStatus |= (1 << LED_HEATER_BIT);
}

void exitErrorState() {
    state.mode = MODE_NORMAL;
    state.sensorErrorCount = 0;
}
