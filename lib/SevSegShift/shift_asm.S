; =============================================================================
; HEF4094B Shift Register Assembly Routine for ATmega8
; Optimized for inverted signal polarity (all signals active LOW)
;
; Pin assignments:
;   DATA  = PB3 (bit 3 of PORTB)
;   CLOCK = PB5 (bit 5 of PORTB)
;   LATCH = PD5 (bit 5 of PORTD)
;
; Signal polarity (inverted):
;   - All signals idle HIGH
;   - Data: 1=LOW, 0=HIGH (inverted)
;   - Clock: falling edge (HIGH->LOW) samples data
;   - Latch: active LOW pulse
; =============================================================================

#include <avr/io.h>

; Pin definitions
.equ DATA_BIT,  3       ; PB3
.equ CLK_BIT,   5       ; PB5
.equ STR_BIT,   5       ; PD5

; Masks for quick bit manipulation
.equ DATA_MASK, (1 << DATA_BIT)
.equ CLK_MASK,  (1 << CLK_BIT)
.equ STR_MASK,  (1 << STR_BIT)

; Combined masks for PORTB (DATA and CLOCK)
.equ CLK_HIGH_DATA_HIGH, (CLK_MASK | DATA_MASK)
.equ CLK_HIGH_DATA_LOW,  (CLK_MASK)
.equ CLK_LOW_DATA_HIGH,  (DATA_MASK)
.equ CLK_LOW_DATA_LOW,   (0)

    .section .text
    .global shiftOut16_asm

; =============================================================================
; void shiftOut16_asm(uint8_t segments, uint8_t digitSelect)
;
; Parameters:
;   r24 = segments (first byte to shift out)
;   r22 = digitSelect (second byte to shift out)
;
; Registers used:
;   r24 = current byte being shifted
;   r22 = second byte (digitSelect)
;   r18 = bit counter
;   r19 = temp for port manipulation
;   r20 = PORTB shadow
; =============================================================================

shiftOut16_asm:
    ; Save registers we'll modify
    push r18
    push r19
    push r20

    ; Read current PORTB state and ensure clock is HIGH
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, CLK_MASK           ; Set clock HIGH
    out _SFR_IO_ADDR(PORTB), r20

    ; =========================================================================
    ; Shift out first byte (segments) - 8 bits, MSB first
    ; =========================================================================
    ldi r18, 8                  ; Bit counter

shift_segments_loop:
    ; Prepare PORTB value with clock HIGH
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, CLK_MASK           ; Clock HIGH

    ; Check MSB of segments (r24)
    ; Data is INVERTED: if bit=1, output LOW; if bit=0, output HIGH
    sbrc r24, 7                 ; Skip next if bit 7 is clear (0)
    rjmp seg_bit_is_one

    ; Bit is 0 -> output HIGH (inverted)
    ori r20, DATA_MASK
    rjmp seg_output_data

seg_bit_is_one:
    ; Bit is 1 -> output LOW (inverted)
    andi r20, ~DATA_MASK

seg_output_data:
    ; Output data with clock still HIGH
    out _SFR_IO_ADDR(PORTB), r20
    nop                         ; Small setup time

    ; Clock LOW (falling edge - data is sampled here)
    andi r20, ~CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20
    nop                         ; Hold time
    nop

    ; Clock HIGH (return to idle)
    ori r20, CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; Shift to next bit
    lsl r24
    dec r18
    brne shift_segments_loop

    ; =========================================================================
    ; Shift out second byte (digitSelect) - 8 bits, MSB first
    ; =========================================================================
    mov r24, r22                ; Move digitSelect to r24
    ldi r18, 8                  ; Reset bit counter

shift_digit_loop:
    ; Prepare PORTB value with clock HIGH
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, CLK_MASK           ; Clock HIGH

    ; Check MSB of digitSelect (r24)
    ; Data is INVERTED: if bit=1, output LOW; if bit=0, output HIGH
    sbrc r24, 7                 ; Skip next if bit 7 is clear (0)
    rjmp dig_bit_is_one

    ; Bit is 0 -> output HIGH (inverted)
    ori r20, DATA_MASK
    rjmp dig_output_data

dig_bit_is_one:
    ; Bit is 1 -> output LOW (inverted)
    andi r20, ~DATA_MASK

dig_output_data:
    ; Output data with clock still HIGH
    out _SFR_IO_ADDR(PORTB), r20
    nop                         ; Small setup time

    ; Clock LOW (falling edge - data is sampled here)
    andi r20, ~CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20
    nop                         ; Hold time
    nop

    ; Clock HIGH (return to idle)
    ori r20, CLK_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; Shift to next bit
    lsl r24
    dec r18
    brne shift_digit_loop

    ; =========================================================================
    ; Latch pulse (active LOW)
    ; =========================================================================
    ; Set data line back to idle HIGH
    in r20, _SFR_IO_ADDR(PORTB)
    ori r20, DATA_MASK
    out _SFR_IO_ADDR(PORTB), r20

    ; Latch LOW (active)
    cbi _SFR_IO_ADDR(PORTD), STR_BIT
    nop
    nop
    nop

    ; Latch HIGH (idle)
    sbi _SFR_IO_ADDR(PORTD), STR_BIT

    ; Restore registers
    pop r20
    pop r19
    pop r18
    ret
