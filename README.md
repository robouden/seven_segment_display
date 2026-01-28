# Seven Segment Display Driver

Firmware for a multiplexed 3-digit 7-segment display with status LEDs, designed for the Spa Control device. Uses the SevSegShift library for shift register-based multiplexing on an ATmega8A.

## Hardware

- **MCU**: ATmega8A-AU @ 16MHz
- **Display**: 3-digit common anode 7-segment display
- **Shift Registers**: Two daisy-chained HEF4094B 8-bit shift registers
- **Digit Drivers**: PNP transistors (active LOW cathode selection)
- **Status LEDs**: Heater, Auto, Air, Pump

## Pin Configuration

| Pin | Function |
|-----|----------|
| PB3 | DATA - Serial data to shift registers |
| PB5 | CLOCK - Clock signal to shift registers |
| PD5 | LATCH/STROBE - Latch signal to shift registers |

## Shift Register Chain

Two cascaded 4094 shift registers receive 16 bits per update:

**First 4094** (LED status + digit selection):
| Output | Function |
|--------|----------|
| QP0 | Heater LED |
| QP1 | Auto LED |
| QP3 | Air LED |
| QP4 | Pump LED |
| QP5 | Cathode 1 (left digit) |
| QP6 | Cathode 3 (right digit) |
| QP7 | Cathode 2 (middle digit) |
| QS2 | Cascade to second 4094 |

**Second 4094** (segment anodes):
| Output | Segment |
|--------|---------|
| QP0 | g |
| QP1 | c |
| QP2 | d.p. |
| QP3 | d |
| QP4 | b |
| QP5 | f |
| QP6 | e |
| QP7 | a |

## Signal Timing

All signals idle HIGH:
- **Data**: Sent directly (no inversion - caller provides raw wire values)
- **Clock**: Rising edge (LOW→HIGH) samples data (per HEF4094B datasheet)
- **Latch**: Active LOW pulse transfers data to outputs

**Pre-sequence** (start of each 16-bit word):
1. Data LOW (clock still HIGH) - signals start of transmission
2. Clock LOW (data still LOW) - ready for first bit

**Per-bit timing**:
1. Set data while clock is LOW
2. Clock HIGH (rising edge samples data)
3. Clock LOW (prepare for next bit)

See [lib/readme.txt](lib/readme.txt) for detailed timing specifications.

## SevSegShift Library

This project includes a custom fork of the [SevSeg](https://github.com/DeanIsMe/SevSeg) library modified to support shift register-based displays. The library is located in [lib/SevSegShift/](lib/SevSegShift/).

### Usage

```cpp
#include <SevSegShift.h>

SevSegShift display;

void setup() {
    // begin(displayType, numDigits, dataPin, clockPin, latchPin, leadingZeros)
    display.begin(COMMON_ANODE, 3, PB3, PB5, PD5);
}

void loop() {
    display.refreshDisplay();  // Call continuously for multiplexing
}
```

### API

```cpp
// Initialize display
display.begin(COMMON_ANODE, 3, dataPin, clockPin, latchPin);

// Display integer
display.setNumber(123);

// Display with decimal point (2nd param = decimal places from right)
display.setNumber(1234, 2);  // Shows "12.34"

// Display float
display.setNumberF(12.3, 1);  // Shows "12.3"

// Display characters (0-9, A-F, dash, space)
display.setChars("Er4");

// Clear display
display.blank();

// Adjust brightness (0-100)
display.setBrightness(80);
```

### Examples

Example sketches are available in the [samples/](samples/) folder:

| File | Description |
|------|-------------|
| [countdown.ino](samples/countdown.ino) | Countdown timer example |

## Spa Control Firmware

The main firmware for the Spa Control device is in [src/spa_control.ino](src/spa_control.ino). It provides:

- Temperature monitoring with ADC averaging
- Hysteresis-based heater control
- Status LED control (Heater, Auto, Air, Pump)
- Error detection and safety shutdown
- Support for linear (TMP36) and NTC thermistor sensors

### Configuration

All settings are in [src/spa_config.h](src/spa_config.h):

#### Temperature Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `DEFAULT_SETPOINT` | 38.0°C | Target water temperature |
| `TEMP_HYSTERESIS` | 0.5°C | Heater turns ON below (setpoint - 0.5°C), OFF above (setpoint + 0.5°C) |
| `MIN_SETPOINT` | 20.0°C | Minimum allowed setpoint |
| `MAX_SETPOINT` | 42.0°C | Maximum allowed setpoint |
| `MAX_TEMP_SAFE` | 45.0°C | Safety cutoff - heater shuts off immediately above this |

#### Sensor Configuration

| Setting | Default | Description |
|---------|---------|-------------|
| `SENSOR_READ_INTERVAL` | 500ms | How often to read the temperature sensor |
| `ADC_SAMPLES` | 8 | Number of ADC samples to average (reduces noise) |
| `SENSOR_ERROR_THRESHOLD` | 3 | Consecutive bad readings before entering error mode |
| `TEMP_OFFSET` | 0.0 | Calibration offset added to final temperature |
| `TEMP_SCALE` | 1.0 | Calibration multiplier for temperature |
| `ADC_VREF_MV` | 5000 | ADC reference voltage in mV (use 3300 for 3.3V systems) |

#### Sensor Type Selection

Uncomment ONE of these in `spa_config.h`:

```c
// Linear voltage output sensor (TMP36, LM35, etc.)
#define SENSOR_TYPE_LINEAR

// NTC Thermistor (10K at 25C with 10K pullup)
// #define SENSOR_TYPE_NTC_10K
```

**Linear Sensor Parameters** (for TMP36, LM35):

| Setting | Default | Description |
|---------|---------|-------------|
| `LINEAR_OFFSET_MV` | 500 | Voltage at 0°C in millivolts (TMP36=500, LM35=0) |
| `LINEAR_MV_PER_DEGREE` | 10 | Millivolts per degree Celsius |

**NTC Thermistor Parameters**:

| Setting | Default | Description |
|---------|---------|-------------|
| `NTC_RESISTANCE_25C` | 10000 | Thermistor resistance at 25°C (ohms) |
| `NTC_PULLUP_RESISTANCE` | 10000 | Pullup resistor value (ohms) |
| `NTC_BETA` | 3950 | Beta coefficient (check thermistor datasheet) |

#### Display Settings

| Setting | Default | Description |
|---------|---------|-------------|
| `NUM_DIGITS` | 3 | Number of display digits |
| `REFRESH_DELAY_US` | 5000 | Microseconds per digit (lower = brighter, range: 3000-6000) |
| `TEMP_DECIMAL_PLACES` | 1 | Decimal places shown for temperature |

#### Pin Assignments

| Setting | Default | Description |
|---------|---------|-------------|
| `DATA_PIN` | PB3 | Serial data to shift registers |
| `CLOCK_PIN` | PB5 | Clock signal to shift registers |
| `LATCH_PIN` | PD5 | Latch/strobe to shift registers |
| `TEMP_SENSOR_PIN` | A0 | Temperature sensor ADC input |
| `PUMP_SWITCH_PIN` | PD2 | Pump button input |
| `AIR_SWITCH_PIN` | PD3 | Air jets button input |
| `AUTO_SWITCH_PIN` | PD4 | Auto mode button input |
| `HEATER_RELAY_PIN` | PD6 | Heater relay output |
| `PUMP_RELAY_PIN` | PD7 | Pump relay output |

#### Feature Toggles

| Setting | Default | Description |
|---------|---------|-------------|
| `ENABLE_AUTO_MODE` | 1 | Enable automatic temperature control |
| `ENABLE_PUMP_CONTROL` | 1 | Enable pump relay control |
| `ENABLE_ERROR_DISPLAY` | 1 | Show error codes on display |
| `DEBUG_SERIAL` | 0 | Enable serial debug output (disable for production)

## Documentation

Detailed documentation and flowcharts are available in the [docs/](docs/) folder:

| Document | Description |
|----------|-------------|
| [spa_control_flowchart.md](docs/spa_control_flowchart.md) | Flowcharts for the spa control firmware |
| [software_flowchart.md](docs/software_flowchart.md) | General software design flowcharts |
| [schematic.pdf](docs/schematic.pdf) | Hardware schematic |

## Building

This project uses PlatformIO with the MiniCore board package.

```bash
# Build
pio run

# Upload (requires USBASP programmer)
pio run --target upload
```

## License

This project is licensed under the MIT License - see [LICENSE](LICENSE) for details.
