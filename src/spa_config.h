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

// DS18B20 digital temperature probe (OneWire on PD0)
#define SENSOR_TYPE_DS18B20

// Linear voltage output sensor (TMP36, LM35, etc.)
// #define SENSOR_TYPE_LINEAR

// NTC Thermistor (10K at 25C with 10K pullup)
// #define SENSOR_TYPE_NTC_10K

// ============================================================================
// DS18B20 PARAMETERS
// ============================================================================

#ifdef SENSOR_TYPE_DS18B20
    // Conversion time in milliseconds (depends on resolution)
    // 9-bit: 94ms, 10-bit: 188ms, 11-bit: 375ms, 12-bit: 750ms
    #define DS18B20_CONVERSION_MS  750     // 12-bit resolution (default)
#endif

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

// HEF4021 serial data input (shared clock/latch with HEF4094)
#define BUTTON_DATA_PIN  PB4   // Pin 18 on ATmega8 (DIP) - HEF4021 Q7 output

// DS18B20 OneWire data pin (NOTE: PD0 is also RXD - do not use with DEBUG_SERIAL)
#define DS18B20_PIN      PD0   // Pin 2 on ATmega8 (DIP) - requires 4.7k external pull-up to Vcc

// Analog input for temperature sensor (used by LINEAR and NTC sensor types)
#define TEMP_SENSOR_PIN  A0   // PC0

// Water/flow sensor present input (detects if water sensor is connected)
#define WATER_SENSOR_PRESENT_PIN  PC3   // PC3 - Water presence sensor (HIGH = water Sensor error)

// Water/flow sensor input (detects water in heater housing)
#define WATER_SENSOR_PIN  PC4   // PC4 - Water presence sensor (HIGH = water present)

// POH (Power On Heater) feedback - confirms heater power and detects stuck relay
#define POH_FEEDBACK_PIN  PC5   // A5 - Optocoupler feedback (HIGH = heater powered)

// Relay outputs (active HIGH to turn relay ON)
// K1 - Safety Relay (in series with heater element)
#define SAFETY_RELAY_PIN  PB2   // Pin 16 (DIP) - Must be ON for heater to work

// K2 - Aux / Pump Low Speed
#define PUMP_LOW_RELAY_PIN  PB0   // Pin 14 (DIP) - Pump low speed

// K3 - Pump High Speed
#define PUMP_HIGH_RELAY_PIN  PD7   // Pin 13 (DIP) - Pump high speed

// K4 - Heating Element (wired in series with Safety Relay K1)
#define HEATER_RELAY_PIN  PD6   // Pin 12 (DIP) - Heater element

// Legacy compatibility
#define PUMP_RELAY_PIN    PUMP_HIGH_RELAY_PIN

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
// HEF4021 INPUT BIT MAPPING
// ============================================================================

// DIP switches (D0-D3)
#define DIP1_BIT          0   // D0 - DIP switch 1
#define DIP2_BIT          1   // D1 - DIP switch 2
#define DIP3_BIT          2   // D2 - DIP switch 3
#define DIP4_BIT          3   // D3 - DIP switch 4

// Push buttons (D4-D7) - active LOW (pressed = 0, released = 1)
#define BTN_PUMP_BIT      4   // D4 - Pump push button
#define BTN_AIR_BIT       5   // D5 - Air push button
#define BTN_TEMP_UP_BIT   6   // D6 - Temp Up push button
#define BTN_TEMP_DN_BIT   7   // D7 - Temp Down push button

// Temperature setpoint adjustment step (degrees per button press)
#define TEMP_STEP         0.5f

// ============================================================================
// PRIME FAILURE SETTINGS (Error 1 - H20)
// ============================================================================

// Time to run pump when attempting to recover from prime failure (milliseconds)
#define PRIME_RECOVERY_TIME_MS     10000   // 10 seconds

// Delay before checking water sensor after pump starts (milliseconds)
#define PRIME_CHECK_DELAY_MS       2000    // 2 seconds for water to reach sensor

// ============================================================================
// ERROR CODES
// ============================================================================

#define ERROR_NONE              0   // No error
#define ERROR_PRIME_FAILED      1   // H20 - No water in heater housing
#define ERROR_OVER_TEMP         2   // Over temperature safety shutdown
#define ERROR_SENSOR_FAULT      3   // Temperature sensor failure
#define ERROR_RELAY_STUCK       4   // Relay stuck on - POH feedback when heater should be off

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
