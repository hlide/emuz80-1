/*
  PIC18F47Q43 ROM RAM and UART emulation firmware

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Christophe Avoinne
*/

#include "cfg.h"
#include "z80.h"
#include "rom.h"

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
   
    // CLC1.G4POL-------------.
    //                        |
    //                     .-----.
    // RA2(/RFSH)------|>O-|D S Q|---RA4(/WAIT)
    //                     |     |
    // RA1(/MREQ)------|>O-|>   _|
    //                     |  R Q|
    //                     '-----'
    //                       CLC1
    
    RA4PPS    = 0x01; // RA4(/WAIT)
    
    CLCIN0PPS = 0x01; // RA1(/MREQ)    
    CLCIN1PPS = 0x02; // RA2(/RFSH)    

    // CLC1 logic configuration
   
    CLCSELECT = 0;   // Select CLC1 registers  
    CLCnPOL = 0x03;  // G1POL and G2POL inverted
    // CLC1 data inputs select
    CLCnSEL0 = 0x00; // D-FF CLK - CLCIN0PPS(RA1 = /MREQ)
    CLCnSEL1 = 0x01; // D-FF D   - CLCIN1PPS(RA2 = /RFSH)
    CLCnSEL2 = 0x7F; // D-FF S   - none
    CLCnSEL3 = 0x7F; // D-FF R   - none
    // CLC1 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x08; // G2D2T enabled
    CLCnGLS2 = 0x00; // none
    CLCnGLS3 = 0x00; // none
    CLCnCON = 0x04;  // Disabled, 1-input D flip-flop with S and R, no interrupt
       
    // Z80 clock by NCO FDC mode (RA3)
    RA3PPS = 0x3f; // RA3 assign NCO1
    NCO1INCU = (unsigned char)((Z80_CLK*2/61/65536) & 0xff);
    NCO1INCH = (unsigned char)((Z80_CLK*2/61/256) & 0xff);
    NCO1INCL = (unsigned char)((Z80_CLK*2/61) & 0xff);
    NCO1CLK = 0x00; // Clock source Fosc
    NCO1PFM = 0;  // FDC mode
    NCO1OUT = 1;  // NCO output enable
    NCO1EN = 1;   // NCO enable
    
    // Z80 start

    // Prepare ROM access once
    asm("movlw	low (_z80rom shr (0+16))");
    asm("movwf	tblptru,c");

    CLCnCON = 0x84;

    // Force /WAIT = 1
    nWAIT_s = 1;

    // Do not release /RESET right after starting CLK generation
    loop_if (CLK == 1);
    loop_if (CLK == 0);
    loop_if (CLK == 1);
    loop_if (CLK == 0);
    loop_if (CLK == 1);
    loop_if (CLK == 0);
    loop_if (CLK == 1);
    loop_if (CLK == 0);

    // Release /RESET
    nRESET_q = 1;

    nWAIT_s = 0;
    
    while (1) {
    
        // Wait for /WAIT falling down
        loop_if (nWAIT == 1);

        TEST(^= 1);

        if (!nRD) {
            // Z80 memory read cycle (/RD active)
            
            if ((A8_15 & 0x80) == 0) {
                // $0000-$3FFF -> ROM (16KB)
                // $4000-$7FFF -> garbage (16KB)

                // D0_7_q = z80rom[A0_15];
                asm("movff	PORTB,tblptrl");
                asm("movff	PORTD,tblptrh");
                asm("tblrd	*");
                asm("movff	tablat,LATC");
            } else if ((A8_15 & 0x40) == 0) {
                // $8000-$BFFF -> RAM (16KB = 4 x 4KB mirrored)

                // D0_7_q = z80ram[(A0_15 & (RAM_SIZE-1)];
                asm("movf   PORTB,w");
                asm("movwf	fsr2l,c");
                asm("movf   PORTD,w");
                asm("andlw	15"); // (high RAM_SIZE)-1
                asm("addlw	(high _z80ram)");
                asm("movwf	fsr2h,c");
                asm("movf	indf2,w,c");
                asm("movwf	LATC,c");                   
            } else {
                // $C000-$FFFF -> I/O registers
                
                // I/O UART (A0 = 1: U3 flag / 0: U3 RX buffer)
                D0_7_q = (A0_7 & 1) ? PIR9 : U3RXB;
            }

            // Set data bus as output
            D0_7_dir = 0x00;

            // Release /WAIT
            nWAIT_s = 1;

            // Rearm /WAIT D-FF
            nWAIT_s = 0;

            // Wait /RD termination
            loop_if (nRD = 0);

            // Set data bus as input
            D0_7_dir = 0xff;
        } else {
            // Z80 memory write cycle (/RD inactive, /WR implicitly active)
            
            if ((A8_15 & 0xf0) == 0x80) {
                // $8000-$8FFF -> RAM 4KB

                // z80ram[A0_15 & (RAM_SIZE-1)] = D0_7;
                asm("movf   PORTB,w");
                asm("movwf	fsr2l,c");
                asm("movf   PORTD,w");
                asm("andlw	15"); // (high RAM_SIZE)-1
                asm("addlw	(high _z80ram)");
                asm("movwf	fsr2h,c");
                asm("movff	PORTC,indf2");
            } else if ((A8_15 & 0x40) == 0x40) {
                // $4000-$7FFF or $C000-$FFFF -> I/O UART (U3 TX buffer)
                
                U3TXB = D0_7;
            } else {
                // store nothing
            }
            // Release /WAIT
            nWAIT_s = 1;

            // Rearm /WAIT D-FF
            nWAIT_s = 0;
        }

        TEST(^= 1);
    }
}
