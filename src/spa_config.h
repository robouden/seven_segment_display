/**
 * Spa Control Configuration
 *
 * Adjust these values to match your hardware and preferences.
 */

#ifndef SPA_CONFIG_H
#define SPA_CONFIG_H

// ============================================================================
// TEMPERATURE SETTINGS
// ============================================================================

// Default target temperature in Celsius
#define DEFAULT_SETPOINT      38.0f

// Hysteresis band - heater turns ON below (setpoint - hysteresis)
// and turns OFF above (setpoint + hysteresis)
#define TEMP_HYSTERESIS       0.5f

// Minimum and maximum allowed setpoint values
#define MIN_SETPOINT          20.0f
#define MAX_SETPOINT          42.0f

// Safety cutoff - heater will shut off immediately above this temperature
#define MAX_TEMP_SAFE         45.0f

// ============================================================================
// SENSOR CONFIGURATION
// ============================================================================

// How often to read sensors (milliseconds)
#define SENSOR_READ_INTERVAL  500

// Number of ADC samples to average for noise reduction
#define ADC_SAMPLES           8

// Number of consecutive sensor errors before entering fault mode
#define SENSOR_ERROR_THRESHOLD 3

// Temperature sensor calibration
// Adjust these for your specific sensor
#define TEMP_OFFSET           0.0f    // Add this to final temperature reading (after scaling)
#define TEMP_SCALE            1.0f    // Multiply raw temperature by this value

// ADC reference voltage in millivolts
#define ADC_VREF_MV           5000.0f // 5V = 5000mV (change to 3300 for 3.3V systems)

// ============================================================================
// TEMPERATURE SENSOR TYPE
// ============================================================================

// Uncomment ONE of these to select your sensor type:

// Linear voltage output sensor (TMP36, LM35, etc.)
#define SENSOR_TYPE_LINEAR

// NTC Thermistor (10K at 25C with 10K pullup)
// #define SENSOR_TYPE_NTC_10K

// ============================================================================
// LINEAR SENSOR PARAMETERS (for TMP36, LM35, etc.)
// ============================================================================

#ifdef SENSOR_TYPE_LINEAR
    // TMP36: 500mV at 0C, 10mV per degree
    // LM35:  0mV at 0C, 10mV per degree
    #define LINEAR_OFFSET_MV      500.0f   // Voltage at 0C in millivolts
    #define LINEAR_MV_PER_DEGREE  10.0f    // Millivolts per degree Celsius
#endif

// ============================================================================
// NTC THERMISTOR PARAMETERS (Steinhart-Hart coefficients)
// ============================================================================

#ifdef SENSOR_TYPE_NTC_10K
    #define NTC_RESISTANCE_25C    10000.0f   // Resistance at 25C (ohms)
    #define NTC_PULLUP_RESISTANCE 10000.0f   // Pullup resistor value (ohms)
    #define NTC_BETA              3950.0f    // Beta coefficient (typical for 10K NTC)
    #define NTC_REFERENCE_TEMP    298.15f    // 25C in Kelvin
#endif

// ============================================================================
// DISPLAY SETTINGS
// ============================================================================

// Number of digits on display
#define NUM_DIGITS            3

// Refresh delay per digit in microseconds
// Lower = brighter but more CPU usage
// Higher = dimmer but less CPU usage
// Recommended: 3000-6000
#define REFRESH_DELAY_US      5000

// Number of decimal places to show for temperature
#define TEMP_DECIMAL_PLACES   1

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================

// Shift register pins (directly on PORTB)
#define DATA_PIN    PB3   // Pin 17 on ATmega8 (DIP), pin 15 on TQFP
#define CLOCK_PIN   PB5   // Pin 19 on ATmega8 (DIP), pin 17 on TQFP
#define LATCH_PIN   PD5   // Pin 11 on ATmega8 (DIP), pin 9 on TQFP

// Analog input for temperature sensor
#define TEMP_SENSOR_PIN  A0   // PC0

// Digital inputs for switches (directly on PORTD)
#define PUMP_SWITCH_PIN   PD2   // Pin 4 (DIP)
#define AIR_SWITCH_PIN    PD3   // Pin 5 (DIP)
#define AUTO_SWITCH_PIN   PD4   // Pin 6 (DIP)

// Relay outputs
#define HEATER_RELAY_PIN  PD6   // Pin 12 (DIP)
#define PUMP_RELAY_PIN    PD7   // Pin 13 (DIP)

// ============================================================================
// SHIFT REGISTER BIT MAPPING
// ============================================================================

// LED status bits in digit select byte (active HIGH for common anode)
#define LED_HEATER_BIT   0   // QP0 - Heater status LED
#define LED_AUTO_BIT     1   // QP1 - Auto mode LED
#define LED_AIR_BIT      3   // QP3 - Air jets LED
#define LED_PUMP_BIT     4   // QP4 - Pump LED

// Digit cathode bits (active LOW via PNP transistors)
#define CATHODE_LEFT_BIT    5   // QP5 - Left digit (tens)
#define CATHODE_RIGHT_BIT   6   // QP6 - Right digit (decimal)
#define CATHODE_MIDDLE_BIT  7   // QP7 - Middle digit (ones)

// ============================================================================
// FEATURE TOGGLES
// ============================================================================

// Enable/disable features
#define ENABLE_AUTO_MODE      1   // Auto temperature control
#define ENABLE_PUMP_CONTROL   1   // Pump relay control
#define ENABLE_ERROR_DISPLAY  1   // Show error codes on display

// Debug options (disable for production)
#define DEBUG_SERIAL          0   // Enable serial debug output (uses extra RAM)

#endif // SPA_CONFIG_H
