Multiplexed Display Unit
========================

A multiplexed display unit containing status LEDs and a three-digit
7-segment LED display, driven by two cascaded 4094 shift registers.


AVR Pin Connections
-------------------
  Data (D)        - PB3
  Clock Pulse (CP) - PB5
  Latch/Strobe (STR) - PD5

The QS2 output from the first 4094 cascades into the Data input of the second.


First 4094 (Control & Digit Selection)
--------------------------------------
Inputs:
  D   <- PB3 (Data)
  CP  <- PB5 (Clock)
  STR <- PD5 (Latch)

Outputs:
  QP0 -> Heater LED
  QP1 -> Auto LED
  QP2    (n.c.)
  QP3 -> Air LED
  QP4 -> Pump LED
  QP5 -> Cathode 1 (left digit)
  QP6 -> Cathode 3 (right digit)
  QP7 -> Cathode 2 (middle digit)
  QS1    (n.c.)
  QS2 -> Cascade to second 4094


Second 4094 (Segment Control)
-----------------------------
Inputs:
  D   <- QS2 from first 4094
  CP  <- PB5 (Clock)
  STR <- PD5 (Latch)

Outputs:
  QP0 -> Anode seg g
  QP1 -> Anode seg c
  QP2 -> Anode seg d.p.
  QP3 -> Anode seg d
  QP4 -> Anode seg b
  QP5 -> Anode seg f
  QP6 -> Anode seg e
  QP7 -> Anode seg a
  QS1    (n.c.)
  QS2    (n.c.)


Segment Patterns
----------------
Bit order: G C DP D B F E A

  0b01011111  // 0
  0b01001000  // 1
  0b10011011  // 2
  0b11011001  // 3
  0b11001100  // 4
  0b11010101  // 5
  0b11010110  // 6
  0b01001001  // 7
  0b11011111  // 8
  0b11001101  // 9
  0b11001111  // A
  0b11010110  // b
  0b00010111  // C
  0b11011010  // d
  0b10010111  // E
  0b10000111  // F
  0b10100010  // r.  (used for error codes, e.g., Er.4)


Shift Register Timing
---------------------
All signals idle HIGH (active-LOW logic). Each display update sends
a 16-bit word as two sequential 8-bit bytes:

  Byte 1: Segment anode data (shifted to second 4094)
  Byte 2: LED status + digit cathode selection (shifted to first 4094)

Signal Behavior:
  - Data is inverted: logic 1 = LOW output, logic 0 = HIGH output
  - Clock: RISING edge (LOW->HIGH) samples data (per HEF4094B datasheet)
  - Latch: active LOW pulse transfers shift register to outputs

Pre-Sequence (start of each word):
  1. Data LOW (clock still HIGH) - signals start of transmission
  2. Wait ~16µs
  3. Clock LOW (data still LOW) - ready for first bit
  4. Wait ~16µs

Bit Timing (per bit, at 16MHz):
  1. Set data line (inverted) while clock is LOW
  2. Wait ~2µs (data setup time)
  3. Clock HIGH (RISING edge - 4094 samples data here)
  4. Wait ~40µs (clock HIGH hold time)
  5. Clock LOW (prepare for next bit)
  6. Wait ~20µs (before next bit)

Transmission Sequence:
  1. Ensure Clock and Data are HIGH (idle)
  2. Pre-sequence: Data LOW, wait, Clock LOW, wait
  3. Shift out 8 bits of segment data (MSB first)
  4. Shift out 8 bits of digit/LED data (MSB first)
  5. Return Data and Clock to HIGH (idle)
  6. Latch LOW for ~40µs, then HIGH (transfers data to outputs)
  7. Wait ~5ms before next digit refresh (~167Hz multiplexing)
