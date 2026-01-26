; =============================================================================
; HEF4094B Shift Register Assembly Routine for ATmega8
; Optimized for inverted signal polarity (all signals active LOW)
;
; Pin assignments:
;   DATA  = PB3 (bit 3 of PORTB)
;   CLOCK = PB5 (bit 5 of PORTB)
;   LATCH = PD5 (bit 5 of PORTD)
;
; Signal polarity (active LOW / inverted):
;   - All signals idle HIGH
;   - Data: 1=LOW, 0=HIGH (inverted output)
;   - Clock: RISING edge (LOW->HIGH) samples data (per HEF4094B datasheet)
;   - Latch: active LOW pulse transfers shift register to outputs
;
; Pre-sequence before each word:
;   1. Data LOW (while clock still HIGH)
;   2. Clock LOW
;   3. Then begin bit shifting
;
; Timing (at 16MHz, 1 cycle = 62.5ns):
;   - Pre-sequence delays: ~16µs each
;   - Data setup time: ~2µs
;   - Clock HIGH hold: ~40µs
;   - Clock LOW time: ~20µs
;   - Latch pulse width: ~40µs
; =============================================================================

#include <avr/io.h>

; Pin definitions
.equ DATA_BIT,  3       ; PB3
.equ CLK_BIT,   5       ; PB5
.equ STR_BIT,   5       ; PD5

; Masks for bit manipulation
.equ DATA_MASK, (1 << DATA_BIT)
.equ CLK_MASK,  (1 << CLK_BIT)
.equ STR_MASK,  (1 << STR_BIT)

; Delay loop counts (at 16MHz)
; Each loop iteration = 3 cycles (dec + brne)
; delay_us = (count * 3) / 16
.equ DELAY_2US,   10    ; ~2µs  (10 * 3 = 30 cycles = 1.875µs)
.equ DELAY_16US,  85    ; ~16µs (85 * 3 = 255 cycles = 15.9µs)
.equ DELAY_20US,  106   ; ~20µs (106 * 3 = 318 cycles = 19.875µs)
.equ DELAY_40US,  213   ; ~40µs (213 * 3 = 639 cycles = 39.9µs)

    .section .text
    .global shiftOut16_asm

; =============================================================================
; void shiftOut16_asm(uint8_t segments, uint8_t digitSelect)
;
; Shifts out 16 bits to cascaded 4094 shift registers.
; First byte (segments) goes to second register in chain (segment anodes).
; Second byte (digitSelect) goes to first register (LEDs + cathodes).
;
; Parameters:
;   r24 = segments (first byte - segment data for second 4094)
;   r22 = digitSelect (second byte - LEDs/cathodes for first 4094)
;
; Registers used:
;   r24 = current byte being shifted
;   r22 = second byte (digitSelect), saved
;   r18 = bit counter
;   r19 = delay loop counter
;   r20 = PORTB shadow register
; =============================================================================

shiftOut16_asm:
    ; Save registers we'll modify
    push r18
    push r19
    push r20

    ; =========================================================================
    ; PRE-SEQUENCE: Data LOW first, then Clock LOW
    ; This signals the start of a new word (data LOW before clock goes LOW)
    ; =========================================================================

    ; Ensure we start from idle state (both HIGH)
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, (CLK_MASK | DATA_MASK)
    out _SFR_IO_ADDR(PORTB), r20

    ; Step 1: Take Data LOW (clock still HIGH)
    andi r20, ~DATA_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; Wait ~16µs
    ldi r19, DELAY_16US
preseq_delay1:
    dec r19
    brne preseq_delay1

    ; Step 2: Take Clock LOW (data still LOW) - ready for first bit
    andi r20, ~CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; Wait ~16µs
    ldi r19, DELAY_16US
preseq_delay2:
    dec r19
    brne preseq_delay2

    ; =========================================================================
    ; Shift out first byte (segments) - 8 bits, MSB first
    ; Clock is LOW, ready to set data and pulse clock HIGH
    ; =========================================================================
    ldi r18, 8                      ; Bit counter

shift_byte1_loop:
    ; --- Set data line (inverted: 1->LOW, 0->HIGH) while clock is LOW ---
    in r20, _SFR_IO_ADDR(PORTB)
    andi r20, ~CLK_MASK             ; Ensure clock stays LOW

    sbrc r24, 7                     ; Skip if bit 7 is clear
    rjmp byte1_bit_set
    ; Bit is 0 -> output HIGH (inverted)
    ori r20, DATA_MASK
    rjmp byte1_output
byte1_bit_set:
    ; Bit is 1 -> output LOW (inverted)
    andi r20, ~DATA_MASK
byte1_output:
    out _SFR_IO_ADDR(PORTB), r20    ; Data set, clock still LOW

    ; --- Data setup delay (~2µs) ---
    ldi r19, DELAY_2US
byte1_setup_delay:
    dec r19
    brne byte1_setup_delay

    ; --- Clock HIGH (RISING edge - 4094 samples data here) ---
    ori r20, CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; --- Clock HIGH hold time (~40µs) ---
    ldi r19, DELAY_40US
byte1_high_delay:
    dec r19
    brne byte1_high_delay

    ; --- Clock LOW (prepare for next bit) ---
    andi r20, ~CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; --- Clock LOW time (~20µs) ---
    ldi r19, DELAY_20US
byte1_low_delay:
    dec r19
    brne byte1_low_delay

    ; Shift to next bit
    lsl r24
    dec r18
    brne shift_byte1_loop

    ; =========================================================================
    ; Shift out second byte (digitSelect) - 8 bits, MSB first
    ; Clock is already LOW from last bit of first byte
    ; =========================================================================
    mov r24, r22                    ; Move digitSelect to working register
    ldi r18, 8                      ; Reset bit counter

shift_byte2_loop:
    ; --- Set data line (inverted: 1->LOW, 0->HIGH) while clock is LOW ---
    in r20, _SFR_IO_ADDR(PORTB)
    andi r20, ~CLK_MASK             ; Ensure clock stays LOW

    sbrc r24, 7                     ; Skip if bit 7 is clear
    rjmp byte2_bit_set
    ; Bit is 0 -> output HIGH (inverted)
    ori r20, DATA_MASK
    rjmp byte2_output
byte2_bit_set:
    ; Bit is 1 -> output LOW (inverted)
    andi r20, ~DATA_MASK
byte2_output:
    out _SFR_IO_ADDR(PORTB), r20    ; Data set, clock still LOW

    ; --- Data setup delay (~2µs) ---
    ldi r19, DELAY_2US
byte2_setup_delay:
    dec r19
    brne byte2_setup_delay

    ; --- Clock HIGH (RISING edge - 4094 samples data here) ---
    ori r20, CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; --- Clock HIGH hold time (~40µs) ---
    ldi r19, DELAY_40US
byte2_high_delay:
    dec r19
    brne byte2_high_delay

    ; --- Clock LOW (prepare for next bit) ---
    andi r20, ~CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; --- Clock LOW time (~20µs) ---
    ldi r19, DELAY_20US
byte2_low_delay:
    dec r19
    brne byte2_low_delay

    ; Shift to next bit
    lsl r24
    dec r18
    brne shift_byte2_loop

    ; =========================================================================
    ; Latch pulse (active LOW) - transfers shift register to outputs
    ; =========================================================================

    ; Return clock and data to idle HIGH
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, (DATA_MASK | CLK_MASK)
    out _SFR_IO_ADDR(PORTB), r20

    ; Small delay before latch
    ldi r19, DELAY_2US
pre_latch_delay:
    dec r19
    brne pre_latch_delay

    ; Latch LOW (active - begin transfer)
    cbi _SFR_IO_ADDR(PORTD), STR_BIT

    ; Latch pulse width (~40µs)
    ldi r19, DELAY_40US
latch_delay:
    dec r19
    brne latch_delay

    ; Latch HIGH (idle - transfer complete)
    sbi _SFR_IO_ADDR(PORTD), STR_BIT

    ; Restore registers
    pop r20
    pop r19
    pop r18
    ret
