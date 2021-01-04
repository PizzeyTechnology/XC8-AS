    ;================================================
    ;== Original code by PIZZEY TECHNOLOGY         ==
    ;== Please like, subscribe and comment         ==
    ;== https://www.youtube.com/c/PizzeyTechnology ==
    ;================================================
    
   
    ; Datasheet
    ; https://ww1.microchip.com/downloads/en/DeviceDoc/40300c.pdf
    
    ; MPLAB® XC8 PIC® Assembler User's Guide
    ; https://ww1.microchip.com/downloads/en/DeviceDoc/MPLAB%20XC8%20PIC%20Assembler%20User%27s%20Guide%2050002974A.pdf
    
    ; MPASM to MPLAB® XC8 PIC® Assembler Migration Guide
    ; https://ww1.microchip.com/downloads/en/DeviceDoc/MPASM%20to%20MPLAB%20XC8%20PIC%20Assembler%20Migration%20Guide%2050002973A.pdf
    
    ; Include files
    ; C:\Program Files\Microchip\MPLABX\v5.45\packs\Microchip\PIC16Fxxx_DFP\1.2.33\xc8\pic\include\proc

    PROCESSOR 16F627
    
    ; Microcontroller	= PIC16F62X
    ; Package		= PDIP/SOIC
    ;
    ;		    --------
    ;		 --| 1   18 |--
    ;		 --| 2   17 |--
    ;		 --| 3   16 |--
    ;	  (MCLR) --| 4   15 |--
    ;	     VSS --| 5   14 |-- VDD
    ;	INT0/RB0 >>| 6   13 |-- PGD
    ;	     RB1 --| 7   12 |-- PGC
    ;	     RB2 --| 8   11 |>> RB5
    ;	     RB3 <<| 9   10 |-- RB4
    ;		    --------

    
#include <xc.inc>
#include "pic16f627.inc"
    
;=====================================   
  CONFIG  FOSC = INTOSCCLK      ; Oscillator Selection bits (INTRC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = ON            ; RA5/MCLR pin function select (RA5/MCLR pin function is MCLR)
  CONFIG  BOREN = ON            ; Brown-out Reset Enable bit (BOD Reset enabled)
  CONFIG  LVP = OFF             ; Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection off)
  CONFIG  CP = OFF              ; Code Protection bits (Program memory code protection off)
;===================================== 
  ;CBLOCK is an MPASM directive. Consider using SET/EQU directives, or DS
  
temp		EQU	70h	; general purpose
w_save		EQU	71h	; working file register context save
s_save		EQU	72h	; status file register context save
		
d1		EQU	73h	; delay routine
d2		EQU	74h
d3		EQU	75h	
		
isr_d1		EQU	76h	; isr delay routine
isr_d2		EQU	77h
isr_d3		EQU	78h		
;=====================================   
;Instruction opcodes are 14 bits wide on this baseline device (PIC16F627).
;delta=2 flag indicates that 2 bytes reside at each address in memory space.

    ;Specify psect position for linker: 
    ;    -Pres_vect=0h     
  
PSECT res_vect, class=CODE, delta=2
res_vect:
    goto main
;=====================================   
;Data words on this device are 8 bits. Any PSECT for data should therefore
;omit the delta flag or explicitly set this flag (delta=1).
    
    ;To do: add a data psect?
;=====================================   
    ;Specify psect position for linker:
    ;    -Pint_vect=4h 
    
PSECT int_vect, class=code, delta=2    
    ;INTCON available on all memory banks	
    btfss	INTF		; external interrupt flag set?
    retfie
    bcf		INTF		; clear external interrupt flag
    
    ;save context
    movwf	w_save
    movf	STATUS,W
    movwf	s_save
    
    BANKSEL	PORTB
    bsf		RB3
    call	isr_delay	;don't risk using general purpose delay!
    bcf		RB3
    
    ;restore context
    movf	s_save,w
    movwf	STATUS
    movf	w_save,w
    
    retfie
;===================================== 
;Linker can place the following code. No need for linker flags.
PSECT code
;---------------------------------------- 
delay_500ms: 
			;499994 cycles
	movlw	0x03
	movwf	d1
	movlw	0x18
	movwf	d2
	movlw	0x02
	movwf	d3
delay_500ms_0:
	decfsz	d1, f
	goto	$+2
	decfsz	d2, f
	goto	$+2
	decfsz	d3, f
	goto	delay_500ms_0
			;2 cycles
	goto	$+1
			;4 cycles (including call)
	return		
;---------------------------------------- 
; Delay = 1 seconds
; Clock frequency = 4 MHz
isr_delay:
			;999990 cycles
	movlw	0x07
	movwf	isr_d1
	movlw	0x2F
	movwf	isr_d2
	movlw	0x03
	movwf	isr_d3
isr_delay_0:
	decfsz	isr_d1, f
	goto	$+2
	decfsz	isr_d2, f
	goto	$+2
	decfsz	isr_d3, f
	goto	isr_delay_0
			;6 cycles
	goto	$+1
	goto	$+1
	goto	$+1
			;4 cycles (including call)
	return	
;---------------------------------------- 	
main:
    ;configure port bits
    BANKSEL	TRISB
    movlw	00000001B	;RB0 input, RB1:7 output
    movwf	TRISB
    
    BANKSEL	PORTB
    CLRF	PORTB
    BSF		RB5

poll_input:			;Wait for switch press
    btfsc	RB0
    goto	poll_input
    call	delay_500ms	;Delay before enable interrupt

    ;configure external interrupt
    BANKSEL	OPTION_REG
    bcf		INTEDG		;0 = Interrupt on falling edge of RB0/INT pin
    BANKSEL	INTCON
    bcf		INTF		;clear flag before enable interrupt
    bsf		INTE		;enable external interrupt
    bsf		GIE		;enable all unmasked interrupts
    
main_loop:
    bsf		RB5
    call	delay_500ms
    bcf		RB5
    call	delay_500ms
    goto	main_loop 
    
END res_vect 
    
