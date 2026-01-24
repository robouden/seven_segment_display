# Seven Segment Display Driver

Firmware for a 4-digit 7-segment display driver for the Davies SPA-QUIP v6 device. Provides multiplexed display control using shift registers on an ATmega8A microcontroller.

## Hardware

- **MCU**: ATmega8A-AU @ 4MHz external crystal
- **Display**: CA56-125URWA 4-digit common anode 7-segment display
- **Shift Registers**: Two daisy-chained HEF4094B 8-bit shift registers
- **Digit Drivers**: BC856 PNP transistors for digit selection

## Pin Configuration

| Pin | Function |
|-----|----------|
| PD5 (Pin 11) | DATA - Serial data to shift registers |
| PD6 (Pin 12) | CLOCK - Clock signal to shift registers |
| PD7 (Pin 13) | STROBE - Latch signal to shift registers |

## Features

- Display numbers 0-9999 with optional leading zeros
- Temperature display with decimal point (e.g., 38.5°C)
- Individual decimal point control
- Hexadecimal character support (0-F)
- Efficient multiplexing using only 3 control pins

## API

```cpp
// Display a number (0-9999)
displayNumber(uint16_t number, bool leadingZeros);

// Display temperature in tenths of a degree (e.g., 385 = 38.5°C)
displayTemperature(int16_t tempTenths);

// Set individual digit (position 0-3, value 0-15)
setDigit(uint8_t position, uint8_t value);

// Control decimal points
setDecimalPoint(uint8_t position, bool on);

// Refresh display (call repeatedly in main loop)
refreshDisplay();

// Clear all digits
clearDisplay();
```

## Building

This project uses PlatformIO with the MiniCore board package.

```bash
# Build
pio run

# Upload (requires USBASP programmer)
pio run --target upload
```

## Schematic

See [schematic.pdf](schematic.pdf) for the hardware design.
