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
// #define TEST_MODE  // Comment out to use normal display logic

// ============================================================================
// SEGMENT PATTERNS
// ============================================================================

// Segment map (bit order: G C DP D B F E A - specific to this hardware)
// Segment map (bit order: A E F B D DP C G - specific to this hardware)
static const uint8_t SEGMENT_MAP[] = {
    0b11111010, // 0
    0b00010010, // 1
    0b11011001, // 2
    0b10011011, // 3 
    0b00110011, // 4 
    0b10101011, // 5
    0b11101011, // 6 
    0b10010010, // 7
    0b11111011, // 8
    0b10111011, // 9
    0b11110011, // A
    0b01101011, // B
    0b11101000, // C
    0b01011011, // D
    0b11101001, // E
    0b11100001, // F

 //   0b01011111, // 0
 //   0b01001000, // 1
 //   0b10011011, // 2
 //   0b11011001, // 3
 //   0b11001100, // 4
 //   0b11010101, // 5
 //   0b11010111, // 6
 //   0b01001001, // 7
 //   0b11011111, // 8
 //   0b11011101, // 9
 //   0b11001111, // A
 //   0b11010110, // b
 //   0b00010111, // C
 //   0b11011010, // d
 //   0b10010111, // E
 //   0b10000111, // F

};

static const uint8_t CHAR_BLANK = 0b00000000;
static const uint8_t CHAR_DASH  = 0b00000001;
static const uint8_t CHAR_r     = 0b01000001;  // 'r' for error display
static const uint8_t CHAR_H     = 0b01110011;  // 'H' for Heat/H20
static const uint8_t CHAR_t     = 0b01101101;  // 't' for Heat
static const uint8_t CHAR_DP    = 0b00000100;  // Decimal point bit

// SEGMENT_MAP[0] is '0' for H20 display
// SEGMENT_MAP[2] is '2' for H20 display

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
    bool heaterOn;              // Heater element relay (K4) state
    bool safetyRelayOn;         // Safety relay (K1) state - must be ON for heater
    bool pumpHighOn;            // Pump high speed relay (K3) state
    bool pumpLowOn;             // Pump low speed / Aux relay (K2) state
    bool pumpOn;                // Legacy: true if either pump speed is on
    bool airOn;                 // Air jets state (future use)
    bool autoMode;              // Auto temperature control enabled
    bool waterPresent;          // Water detected in heater housing
    bool pohFeedback;           // POH feedback - true if heater has power

    // Error handling
    uint8_t sensorErrorCount;   // Consecutive sensor read errors
    SystemMode mode;            // Current operating mode
    uint8_t currentError;       // Current error code (0 = none)

    // HEF4021 button/DIP switch state
    uint8_t buttonRawState;          // Raw HEF4021 byte (D7..D0)
    uint8_t lastButtonState;         // Previous state for edge detection

    // Prime recovery (Error 1 - H20)
    bool primeRecoveryActive;        // Currently attempting prime recovery
    unsigned long primeRecoveryStart; // Timestamp when prime recovery started

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
// uint8_t readHEF4021();
void readButtons();
void processTemperature();
void controlTemperature();
void updateDisplay();
void refreshDisplay();
void shiftOut16(uint8_t byte1, uint8_t byte2);
void setDisplayNumber(float num, uint8_t decPlaces);
void setDisplayChars(const char* str);
void setDisplayError(uint8_t errorCode);
void setDisplayH20();
void blankDisplay();
void updateLEDs();
void enterErrorState(uint8_t errorCode);
void exitErrorState();
float readTemperatureSensor();
bool readWaterSensor();
bool readPohFeedback();
void checkPrimeFailure();
void checkPohFeedback();
void handlePrimeRecovery();
void updateRelayOutputs();
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
    state.safetyRelayOn = false;
    state.pumpHighOn = false;
    state.pumpLowOn = false;
    state.pumpOn = false;
    state.airOn = false;
    state.autoMode = true;
    state.waterPresent = false;
    state.pohFeedback = false;
    state.sensorErrorCount = 0;
    state.mode = MODE_NORMAL;
    state.currentError = ERROR_NONE;
    state.buttonRawState = 0xFF;     // All buttons idle HIGH
    state.lastButtonState = 0xFF;    // No edges on first read
    state.primeRecoveryActive = false;
    state.primeRecoveryStart = 0;
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

    // Configure HEF4021 serial data input (PB4)
    DDRB &= ~(1 << BUTTON_DATA_PIN);   // Input
    PORTB &= ~(1 << BUTTON_DATA_PIN);  // No pull-up (driven by HEF4021)

    // Configure DS18B20 OneWire pin (PD0) - input with pull-up
#ifdef SENSOR_TYPE_DS18B20
    DDRD &= ~(1 << DS18B20_PIN);    // Input
    PORTD |= (1 << DS18B20_PIN);    // Internal pull-up (external 4.7k required)
#endif

    // Configure water sensor pin as input (PC4)
    DDRC &= ~(1 << WATER_SENSOR_PIN);  // Input
    PORTC |= (1 << WATER_SENSOR_PIN); // Internal pull-up enabled


    // Configure POH feedback pin as input (PC5/A5)
    DDRC &= ~(1 << POH_FEEDBACK_PIN);  // Input
    PORTC &= ~(1 << POH_FEEDBACK_PIN); // No pull-up (external circuit)

    // Configure relay outputs on PORTD
    // K4 (PD6) - Heater element, K3 (PD7) - Pump high speed
    DDRD |= (1 << HEATER_RELAY_PIN) | (1 << PUMP_HIGH_RELAY_PIN);
    PORTD &= ~((1 << HEATER_RELAY_PIN) | (1 << PUMP_HIGH_RELAY_PIN));

    // Configure relay outputs on PORTB
    // K1 (PB2) - Safety relay, K2 (PB0) - Pump low speed / Aux
    DDRB |= (1 << SAFETY_RELAY_PIN) | (1 << PUMP_LOW_RELAY_PIN);
    PORTB &= ~((1 << SAFETY_RELAY_PIN) | (1 << PUMP_LOW_RELAY_PIN));  // Start with relays off

    // Configure ADC (only needed for analog sensor types)
#ifndef SENSOR_TYPE_DS18B20
    ADMUX = (1 << REFS0);  // AVcc reference, ADC0 selected
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  // Enable ADC, prescaler 64
#endif

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

    // ---- Handle prime recovery if active ----
    if (state.primeRecoveryActive) {
        handlePrimeRecovery();
    }

    // ---- Sensor Reading (every SENSOR_READ_INTERVAL ms) ----
    if (currentTime - state.lastSensorRead >= SENSOR_READ_INTERVAL) {
        state.lastSensorRead = currentTime;

        // Read all sensors
        readSensors();


        // Check for presence of water sensor (no water sensor connected)
 //       readWaterSensorConnection();

        // Check for water sensor failure (no water in heater)
        checkPrimeFailure();


        // Check POH feedback (stuck relay detection)
        checkPohFeedback();

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
// DS18B20 ONEWIRE (bit-bang on PD0)
// ============================================================================

#ifdef SENSOR_TYPE_DS18B20

static void owDriveLow() {
    DDRD |= (1 << DS18B20_PIN);     // Output
    PORTD &= ~(1 << DS18B20_PIN);   // Drive LOW
}

static void owRelease() {
    DDRD &= ~(1 << DS18B20_PIN);    // Input (external pull-up pulls HIGH)
    PORTD |= (1 << DS18B20_PIN);    // Enable internal pull-up as backup
}

static uint8_t owReadPin() {
    return (PIND >> DS18B20_PIN) & 0x01;
}

static bool owReset() {
    owDriveLow();
    delayMicroseconds(480);
    owRelease();
    delayMicroseconds(70);
    bool presence = !owReadPin();    // LOW = device present
    delayMicroseconds(410);
    return presence;
}

static void owWriteBit(uint8_t bit) {
    if (bit) {
        owDriveLow();
        delayMicroseconds(6);
        owRelease();
        delayMicroseconds(64);
    } else {
        owDriveLow();
        delayMicroseconds(60);
        owRelease();
        delayMicroseconds(10);
    }
}

static uint8_t owReadBit() {
    owDriveLow();
    delayMicroseconds(6);
    owRelease();
    delayMicroseconds(9);
    uint8_t bit = owReadPin();
    delayMicroseconds(55);
    return bit;
}

static void owWriteByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        owWriteBit(byte & 0x01);
        byte >>= 1;
    }
}

static uint8_t owReadByte() {
    uint8_t byte = 0;
    for (uint8_t i = 0; i < 8; i++) {
        byte >>= 1;
        if (owReadBit()) byte |= 0x80;
    }
    return byte;
}

#endif // SENSOR_TYPE_DS18B20

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
            enterErrorState(ERROR_SENSOR_FAULT);  // Error code 3: Sensor fault
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

    // Read buttons and DIP switches via HEF4021
    readButtons();

    // Read water sensor
    state.waterPresent = readWaterSensor();
}

float readTemperatureSensor() {
    float temperature;

#ifdef SENSOR_TYPE_DS18B20
    // DS18B20 non-blocking: start conversion, read result on next call
    static bool conversionStarted = false;
    static unsigned long conversionStart = 0;

    if (!conversionStarted) {
        // Start first conversion
        if (owReset()) {
            owWriteByte(0xCC);  // Skip ROM (single sensor on bus)
            owWriteByte(0x44);  // Convert Temperature
            conversionStarted = true;
            conversionStart = millis();
        }
        return state.lastGoodTemp;  // Return last known good value
    }

    // Check if conversion is complete
    if (millis() - conversionStart < DS18B20_CONVERSION_MS) {
        return state.lastGoodTemp;  // Still converting
    }

    // Read scratchpad
    if (!owReset()) {
        conversionStarted = false;
        return -999.0f;  // Sensor not responding
    }
    owWriteByte(0xCC);  // Skip ROM
    owWriteByte(0xBE);  // Read Scratchpad

    uint8_t lsb = owReadByte();
    uint8_t msb = owReadByte();

    // Start next conversion immediately
    if (owReset()) {
        owWriteByte(0xCC);
        owWriteByte(0x44);
        conversionStart = millis();
    } else {
        conversionStarted = false;
    }

    // Check for read error (all 1s = bus not connected)
    if (lsb == 0xFF && msb == 0xFF) {
        return -999.0f;
    }

    // Parse 12-bit signed temperature
    int16_t raw = ((int16_t)msb << 8) | lsb;
    temperature = raw / 16.0f;

#elif defined(SENSOR_TYPE_NTC_10K)
    // NTC Thermistor calculation using Beta equation
    uint32_t adcSum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        ADMUX = (ADMUX & 0xF0) | 0x00;
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));
        adcSum += ADC;
    }
    uint16_t adcAvg = adcSum / ADC_SAMPLES;

    if (adcAvg < 10 || adcAvg > 1013) {
        return -999.0f;
    }

    float resistance = NTC_PULLUP_RESISTANCE * ((float)adcAvg / (1023.0f - (float)adcAvg));
    float steinhart = log(resistance / NTC_RESISTANCE_25C) / NTC_BETA;
    steinhart += 1.0f / NTC_REFERENCE_TEMP;
    temperature = (1.0f / steinhart) - 273.15f;

#else  // SENSOR_TYPE_LINEAR (default)
    // Linear voltage output sensor (TMP36, LM35, etc.)
    uint32_t adcSum = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
        ADMUX = (ADMUX & 0xF0) | 0x00;
        ADCSRA |= (1 << ADSC);
        while (ADCSRA & (1 << ADSC));
        adcSum += ADC;
    }
    uint16_t adcAvg = adcSum / ADC_SAMPLES;
    float voltage_mv = (adcAvg * 5000.0f) / 1024.0f;
    temperature = (voltage_mv - LINEAR_OFFSET_MV) / LINEAR_MV_PER_DEGREE;
#endif

    // Apply calibration adjustments
    temperature = (temperature * TEMP_SCALE) + TEMP_OFFSET;

    return temperature;
}

// ============================================================================
// WATER SENSOR & PRIME FAILURE HANDLING
// ============================================================================


bool readWaterSensorConnection() {
    // Read if water sensor present on PC3
    // Returns true if water is detected in heater housing
    // Assumes LOW = water sensor connected, HIGH = no water sensore detected
    return (PINC & (1 << WATER_SENSOR_PRESENT_PIN)) != 0;
}


bool readWaterSensor() {
    // Read water presence sensor on PC4
    // Returns true if water is detected in heater housing
    // Assumes HIGH = water present, LOW = no water
    return (PINC & (1 << WATER_SENSOR_PIN)) != 0;
}

void checkPrimeFailure() {
    // Skip check if already in error state or during prime recovery
    if (state.mode == MODE_ERROR || state.primeRecoveryActive) {
        return;
    }

    // Check if heater is trying to run but no water is present
    // This indicates a prime failure - no water in heater housing
    if (state.heaterOn && !state.waterPresent) {
        enterErrorState(ERROR_PRIME_FAILED);
    }
}

void handlePrimeRecovery() {
    unsigned long currentTime = millis();

    // Check if pump button was just pressed (active LOW via HEF4021)
    bool pumpButtonPressed = !(state.buttonRawState & (1 << BTN_PUMP_BIT));
    bool pumpButtonWasPressed = !(state.lastButtonState & (1 << BTN_PUMP_BIT));

    if (state.mode == MODE_ERROR && state.currentError == ERROR_PRIME_FAILED) {
        // Detect rising edge of pump button press
        if (pumpButtonPressed && !pumpButtonWasPressed && !state.primeRecoveryActive) {
            // Start prime recovery - run pump for 10 seconds
            state.primeRecoveryActive = true;
            state.primeRecoveryStart = currentTime;

            // Turn on pump high speed for priming
            state.pumpHighOn = true;
            state.pumpOn = true;
            PORTD |= (1 << PUMP_HIGH_RELAY_PIN);
        }
    }

    // Handle active prime recovery
    if (state.primeRecoveryActive) {
        unsigned long elapsedTime = currentTime - state.primeRecoveryStart;

        // Keep pump running
        PORTD |= (1 << PUMP_HIGH_RELAY_PIN);

        // After initial delay, start checking for water
        if (elapsedTime >= PRIME_CHECK_DELAY_MS) {
            state.waterPresent = readWaterSensor();

            if (state.waterPresent) {
                // Water detected - recovery successful!
                state.primeRecoveryActive = false;
                exitErrorState();
                return;
            }
        }

        // Check if recovery time has expired
        if (elapsedTime >= PRIME_RECOVERY_TIME_MS) {
            // Recovery failed - stop pump and show error again
            state.primeRecoveryActive = false;
            state.pumpHighOn = false;
            state.pumpOn = false;
            PORTD &= ~(1 << PUMP_HIGH_RELAY_PIN);

            // Re-display H20 error
            setDisplayH20();
        }
    }
}

// ============================================================================
// HEF4021 BUTTON / DIP SWITCH READING
// ============================================================================

uint8_t readHEF4021() {
    uint8_t data = 0;

    // PL is HIGH (idle) - parallel data is already loaded into HEF4021
    // Pull PL LOW to hold data and enable shift mode
    PORTD &= ~(1 << LATCH_PIN);
    delayMicroseconds(2);

    // Ensure clock starts LOW
    PORTB &= ~(1 << CLOCK_PIN);
    delayMicroseconds(2);

    // Read 8 bits, MSB first (D7 out first from Q7)
    for (uint8_t i = 0; i < 8; i++) {
        // Read Q7 on PB4
        if (PINB & (1 << BUTTON_DATA_PIN)) {
            data |= (1 << (7 - i));
        }

        // Clock rising edge to shift next bit out
        PORTB |= (1 << CLOCK_PIN);
        delayMicroseconds(2);
        PORTB &= ~(1 << CLOCK_PIN);
        delayMicroseconds(2);
    }

    // Return clock to idle HIGH
    PORTB |= (1 << CLOCK_PIN);

    // Leave PL LOW - do NOT return it to HIGH here
    // Returning PL HIGH would strobe the HEF4094 output register with garbage
    // The next shiftOut16() latch pulse will reload HEF4021 and strobe HEF4094

    return data;
}

void readButtons() {
    // Save previous state for edge detection
    uint8_t prevState = state.buttonRawState;

    // Read raw byte from HEF4021
    state.buttonRawState = readHEF4021();
    state.lastButtonState = prevState;

    // Detect falling edges (1->0 = button pressed, active LOW)
    // fallingEdge bit is set where previous was 1 and current is 0
    uint8_t fallingEdge = prevState & ~state.buttonRawState;

    // Pump button (D4) - toggle pump on/off
    if (fallingEdge & (1 << BTN_PUMP_BIT)) {
        state.pumpOn = !state.pumpOn;
    }

    // Air button (D5) - toggle air on/off
    if (fallingEdge & (1 << BTN_AIR_BIT)) {
        state.airOn = !state.airOn;
    }

    // Temp Up button (D6) - increase setpoint
    if (fallingEdge & (1 << BTN_TEMP_UP_BIT)) {
        state.setpoint += TEMP_STEP;
        if (state.setpoint > MAX_SETPOINT) {
            state.setpoint = MAX_SETPOINT;
        }
    }

    // Temp Down button (D7) - decrease setpoint
    if (fallingEdge & (1 << BTN_TEMP_DN_BIT)) {
        state.setpoint -= TEMP_STEP;
        if (state.setpoint < MIN_SETPOINT) {
            state.setpoint = MIN_SETPOINT;
        }
    }
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
        state.safetyRelayOn = false;
        PORTD &= ~(1 << HEATER_RELAY_PIN);   // K4 off immediately
        PORTB &= ~(1 << SAFETY_RELAY_PIN);    // K1 off immediately
        enterErrorState(ERROR_OVER_TEMP);
    }
}

// ============================================================================
// TEMPERATURE CONTROL
// ============================================================================

void controlTemperature() {
    // Skip control if in error state or not in auto mode
    if (state.mode == MODE_ERROR || !state.autoMode) {
        if (state.mode == MODE_ERROR) {
            state.heaterOn = false;
            state.safetyRelayOn = false;
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

    // Safety relay (K1) must be ON whenever heater is ON
    // K4 heater element is wired in series with K1 safety relay
    state.safetyRelayOn = state.heaterOn;

    // Update pump state from switches
    // Pump high (K3) from pump switch, Pump low (K2) from air switch
    state.pumpHighOn = state.pumpOn;
    state.pumpLowOn = state.airOn;

    // Apply all relay outputs
    updateRelayOutputs();
}

void updateRelayOutputs() {
    // K1 - Safety Relay (PB2)
    if (state.safetyRelayOn) {
        PORTB |= (1 << SAFETY_RELAY_PIN);
    } else {
        PORTB &= ~(1 << SAFETY_RELAY_PIN);
    }

    // K2 - Pump Low Speed / Aux (PB0)
    if (state.pumpLowOn) {
        PORTB |= (1 << PUMP_LOW_RELAY_PIN);
    } else {
        PORTB &= ~(1 << PUMP_LOW_RELAY_PIN);
    }

    // K3 - Pump High Speed (PD7)
    if (state.pumpHighOn) {
        PORTD |= (1 << PUMP_HIGH_RELAY_PIN);
    } else {
        PORTD &= ~(1 << PUMP_HIGH_RELAY_PIN);
    }

    // K4 - Heater Element (PD6) - only if safety relay is also on
    if (state.heaterOn && state.safetyRelayOn) {
        PORTD |= (1 << HEATER_RELAY_PIN);
    } else {
        PORTD &= ~(1 << HEATER_RELAY_PIN);
    }
}

// ============================================================================
// POH FEEDBACK MONITORING
// ============================================================================

bool readPohFeedback() {
    // Read POH optocoupler feedback on PC5 (A5)
    // HIGH = power to heater element confirmed
    return (PINC & (1 << POH_FEEDBACK_PIN)) != 0;
}

void checkPohFeedback() {
    state.pohFeedback = readPohFeedback();

    // Check for stuck relay: POH says heater has power but we didn't turn it on
    if (state.pohFeedback && !state.heaterOn) {
        // Relay is stuck on - immediate safety shutdown
        state.safetyRelayOn = false;
        PORTB &= ~(1 << SAFETY_RELAY_PIN);    // K1 off immediately
        PORTD &= ~(1 << HEATER_RELAY_PIN);    // K4 off immediately
        enterErrorState(ERROR_RELAY_STUCK);
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

//            segments = 0x33;
//         segments = SEGMENT_MAP[13];  // display d


    // Create digit select byte
    // Start with all cathodes HIGH (off) and LED status
    uint8_t digitSelect = 0b11100000 | state.ledStatus;

    // Set current cathode LOW (active)
    uint8_t cathodeBit = getCathodeBit(state.currentDigit);
    digitSelect &= ~(1 << cathodeBit);

    // Send to shift registers
    // NOTE: digitSelect sent FIRST, then segments (matches working test code)
//    shiftOut16(digitSelect, segments);
      shiftOut16(segments, digitSelect);

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
  //  PORTB &= ~(1 << DATA_PIN);
  //  delayMicroseconds(16);

 //   PORTB &= ~(1 << CLOCK_PIN);
 //   delayMicroseconds(16);

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
    // Display "Er#" where # is error code (for generic errors)
    state.digitCodes[0] = SEGMENT_MAP[14];  // 'E'
    state.digitCodes[1] = CHAR_r;           // 'r'
    state.digitCodes[2] = SEGMENT_MAP[errorCode % 10];
    state.decimalPosition = 0;
}

void setDisplayH20() {
    // Display "H20" for Error 1 - Prime Failed (no water in heater)
    state.digitCodes[0] = CHAR_H;           // 'H'
    state.digitCodes[1] = SEGMENT_MAP[2];   // '2'
    state.digitCodes[2] = SEGMENT_MAP[0];   // '0'
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
    state.currentError = errorCode;

    // Safety shutdown - turn off heater and safety relay immediately
    state.heaterOn = false;
    state.safetyRelayOn = false;
    PORTD &= ~(1 << HEATER_RELAY_PIN);    // K4 off
    PORTB &= ~(1 << SAFETY_RELAY_PIN);    // K1 off

    // Display error code
    if (errorCode == ERROR_PRIME_FAILED) {
        // Error 1: Display "H20" for prime failure
        setDisplayH20();
    } else {
        // Other errors: Display "Er#"
        setDisplayError(errorCode);
    }

    // Flash heater LED to indicate error
    state.ledStatus |= (1 << LED_HEATER_BIT);
}

void exitErrorState() {
    state.mode = MODE_NORMAL;
    state.currentError = ERROR_NONE;
    state.sensorErrorCount = 0;
    state.primeRecoveryActive = false;
}
