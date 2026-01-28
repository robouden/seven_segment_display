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

Configuration options are in [src/spa_config.h](src/spa_config.h).

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
