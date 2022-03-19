/*
  PIC18F47Q43 ROM RAM and UART emulation firmware

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Christophe Avoinne
*/

#include "cfg.h"
#include "z80.h"
#include "rom.h"

extern const unsigned char rom[]; // Equivalent to ROM, see the file "rom.c'
unsigned char ram[RAM_SIZE]; // Equivalent to RAM

#define PCFG(x, a, b, c, d, e)  x##E = e; x##A = a; x##B = b; x##C = c; x##D = d

// main routine
void main(void) {
    
    //  |Ports|   PORT A  |   PORT B  |   PORT C  |   PORT D  |  PORT E   |
    //  | Bits|   76543210|   76543210|   76543210|   76543210|  -----3210|
    //  +-----+-----------+-----------+-----------+-----------+-----------+
    PCFG(LAT  , 0b11100111, 0b11111111, 0b11111111, 0b11111111, 0b11111101);
    PCFG(ANSEL, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000);
    PCFG(TRIS , 0b10100111, 0b11111111, 0b11111111, 0b11111111, 0b00001101);
    PCFG(WPU  , 0b11100111, 0b11111111, 0b11111111, 0b11111111, 0b00001001);
    //            ||||||||    ||||||||    ||||||||    ||||||||        ||||
    //            ||||||||    ||||||||    ||||||||    ||||||||        |||'- /WR input
    //            ||||||||    ||||||||    ||||||||    ||||||||        ||'- /RESET input
    //            ||||||||    ||||||||    ||||||||    ||||||||        |'- /INT output
    //            ||||||||    ||||||||    ||||||||    ||||||||        '- /MCLR input
    //            ||||||||    ||||||||    ||||||||    '+++++++- A15-A8 input
    //            ||||||||    ||||||||    '+++++++- D7-D0 input/output   
    //            ||||||||    '+++++++- A7-A0 input
    //            |||||||'- /IORQ input
    //            ||||||'- /MREQ input
    //            |||||'- /RFSH input
    //            ||||'- CLK output
    //            |||'- /WAIT output
    //            ||'- /RD input
    //            |'- TXD input (U3TX)
    //            '- RXD output (U3RX)
    
    // System initialize
    OSCFRQ = 0x08; // 64MHz internal OSC

#if USE_RA7_FOR_TEST_PIN    
    // TEST output pin (RA7)
    TRISA7 = 0;     // A7 output
    LATA7 = 0;      // pin value is 0
    RA7PPS = 0;     // PPS as LATA7
#else
    // UART3 initialize
    U3BRG = 416; // 9600bps @ 64MHz
    U3RXEN = 1; // Receiver enable
    U3TXEN = 1; // Transmitter enable

    // UART3 Receiver
    U3RXPPS = 0x07; //RA7->UART3:RX3;

    // UART3 Transmitter
    RA6PPS = 0x26;  //RA6->UART3:TX3;

    U3ON = 1; // Serial port enable
#endif
   
    // RA3(CLK)---------------.
    //                        |
    //                     .-----.
    // RA2(/RFSH)------|>O-|D S Q|---RA4(/WAIT)
    //                     |     |
    // RA1(/MREQ)------|>O-|>   _|
    //                     |  R Q|
    //                     '-----'
    //                       CLC1
    
    RA4PPS = 0x01; // RA4(/WAIT)
    
    CLCIN0PPS = 0x01; // RA1(/MREQ)    
    CLCIN1PPS = 0x02; // RA2(/RFSH)
    CLCIN4PPS = 0x03; // RA3(CLK)

    // CLC1 logic configuration
   
    CLCSELECT = 0;   // Select CLC1 registers  
    CLCnPOL = 0x03;  // G1POL and G2POL inverted
    // CLC1 data inputs select
    CLCnSEL0 = 0x00; // D-FF CLK - CLCIN0PPS(RA3 = CLK)
    CLCnSEL1 = 0x01; // D-FF D   - CLCIN1PPS(RA1 = /MREQ)
    CLCnSEL2 = 0x7F; // D-FF R   - none
    CLCnSEL3 = 0x04; // D-FF S   - CLCIN4PPS(RA2 = /RFSH)
    // CLC1 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x08; // G2D2T enabled
    CLCnGLS2 = 0x00; // none
    CLCnGLS3 = 0x80; // G4D4T enabled
    CLCnCON = 0x04;  // Disabled, 1-input D flip-flop with S and R, no interrupt
       
    // Z80 clock by NCO FDC mode (RA3)
    RA3PPS = 0x00;
    
    // Z80 start

    // Prepare ROM access once
    asm("movlw	low (_rom shr (0+16))");
    asm("movwf	tblptru,c");

    CLCnCON = 0x84;

    CLK_q = 1, CLK_q = 0;
    CLK_q = 1, CLK_q = 0;
    CLK_q = 1, CLK_q = 0;
    
    nRESET_q = 1;

    CLK_q = 1, CLK_q = 0;
    CLK_q = 1, CLK_q = 0;
    CLK_q = 1, CLK_q = 0;
    
    for (;;) {
        TEST(^= 1);
        while (nWAIT) { CLK_q = 1; CLK_q = 0; };
        TEST(^= 1);
        
        if (!nRD) {
            // Z80 memory read cycle (/RD active)
            
            if ((A8_15 & 0x80) == 0) {
                // $0000-$3FFF -> ROM (16KB)
                // $4000-$7FFF -> garbage (16KB)

                // D0_7_q = rom[A0_15];
                asm("movff	PORTB,tblptrl");
                asm("movff	PORTD,tblptrh");
                asm("tblrd	*");
                asm("movff	tablat,LATC");

            } else if ((A8_15 & 0x40) == 0) {
                // $8000-$BFFF -> RAM (16KB = 4 x 4KB mirrored)

                // D0_7_q = ram[(A0_15 & (RAM_SIZE-1)];
                asm("movf   PORTB,w");
                asm("movwf	fsr2l,c");
                asm("movf   PORTD,w");
                asm("andlw	15"); // (high RAM_SIZE)-1
                asm("addlw	(high _ram)");
                asm("movwf	fsr2h,c");
                asm("movf	indf2,w,c");
                asm("movwf	LATC,c");                   
            } else {
                // $C000-$FFFF -> I/O registers
                
                // I/O UART (A0 = 1: U3 flag / 0: U3 RX buffer)
                D0_7_q = (A0_7 & 1) ? PIR9 : U3RXB;
            }
           
            D0_7_dir = 0b00000000;

            CLK_q = 1, CLK_q = 0; // T1:H->T2:L
            CLK_q = 1, CLK_q = 0; // T2:H->T3:L

            D0_7_dir = 0b11111111;
            
        } else {
            // Z80 memory write cycle (/RD inactive, /WR implicitly active)
            
            if ((A8_15 & 0xf0) == 0x80) {
                // $8000-$8FFF -> RAM 4KB

                // ram[A0_15 & (RAM_SIZE-1)] = D0_7;
                asm("movf   PORTB,w");
                asm("movwf	fsr2l,c");
                asm("movf   PORTD,w");
                asm("andlw	15"); // (high RAM_SIZE)-1
                asm("addlw	(high _ram)");
                asm("movwf	fsr2h,c");
                asm("movff	PORTC,indf2");
            } else if ((A8_15 & 0x40) == 0x40) {
                // $4000-$7FFF or $C000-$FFFF -> I/O UART (U3 TX buffer)
                
                U3TXB = D0_7;
            } else {
                // store nothing
            }
            
            CLK_q = 0, CLK_q = 1; // T2->T3
        }
    }
}
