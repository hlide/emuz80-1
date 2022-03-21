/*
  PIC18F47Q43 ROM RAM and UART emulation firmware

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Christophe Avoinne
*/

//#define Z80_CLK  4000000UL // Z80 clock frequency
#define Z80_CLK  4600000UL // Z80 clock frequency
    
/*
 * Memory mapping to ease address decoding:
 * 
 * +------------------------------------+ $0000
 * |                                    |
 * :              ROM 16KB              : RO(A15..13 = 000b) => ROM
 * :                                    : WO(A15..13 = 000b) => Data bus HI-Z
 * |                                    |
 * +------------------------------------+ $3FFF-$7FFF
 * |                                    |
 * :            Data bus HI-Z           : RW(A15..13 = 001b) => Data bus HI-Z
 * |                                    |
 * +------------------------------------+ $7FFF-$8000
 * |                                    |
 * :              RAM 4KB               : RW(A15..12 = 1000b) => RAM
 * |                                    |
 * +------------------------------------+ $8FFF-$9000
 * |                                    |
 * :            Data bus HI-Z           : RW(A15..12 != 1000b) => Data bus HI-Z
 * |                                    |
 * +------------------------------------+ $DFFF-$E000
 * |                                    |
 * :        I/O registers               : RW(A14 = 1b) => UART
 * |[--------U3TXB/U3RXB---------] $E000| 
 * |[-----------PIR9-------------] $E001|
 * |                                    |
 * +------------------------------------+ $EFFF-$F000
 * |                                    |
 * :            Data bus HI-Z           : RW(A15..12 == 1111b) => Data bus HI-Z
 * |                                    |
 * +------------------------------------+ $FFFF
 * 
 */

#define nIOREQ    RA0
#define nMREQ     RA1
#define nRFSH     RA2
#define CLK       RA3
#define nWAIT     RA4
#define nRD       RA5
#define nWR       RE0
#define nRESET    RE1
#define nINT      RE2
#define nCLK      RE3

#define A0_7      PORTB
#define A8_15     PORTD
#define D0_7      PORTC

#define D0_7_dir  TRISC

#define CLK_q     LATA3
#define D0_7_q    LATC
#define nRESET_q  LATE1

#define nWAIT_d   nMREQ 
#define nWAIT_q   nWAIT 
#define nWAIT_c   nRFSH 
#define nWAIT_r   G3POL 
#define nWAIT_s   G4POL 

// Equivalent to ROM for a Z80, see the file "rom.h' for the details
extern const unsigned char z80rom[];
// Equivalent to RAM for a Z80
extern unsigned char z80ram[];

