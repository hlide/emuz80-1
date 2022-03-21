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

    CLCIN0PPS = 0x01; // RA1(/MREQ)    
    CLCIN1PPS = 0x02; // RA2(/RFSH)    
    CLCIN2PPS = 0x1C; // RD4(A12)    
    CLCIN3PPS = 0x1D; // RD5(A13)    
    CLCIN4PPS = 0x05; // RA5(/RD)    
    CLCIN6PPS = 0x1E; // RD6(A14)    
    CLCIN7PPS = 0x1F; // RD7(A15)    
    
    // CLC1 logic configuration
    //
    // CLC1.G4POL-------------.
    //                        |
    //                     .-----.
    // RA2(/RFSH)------|>O-|D S Q|---RA4(/WAIT)
    //                     |     |
    // RA1(/MREQ)------|>O-|>   _|
    //                     |  R Q|
    //                     '-----'
    //                       CLC1   
    RA4PPS = 0x01;   // RA4(/WAIT)  
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
    // CLC1 data output select    
       
    // CLC2 logic configuration (16K bytes - Z80 ROM selection - $0000-$3FFF)
    //                        __
    // RA5(/RD)----------|>O-|  |
    // RD6(A14)----------|>O-|& |---CLC2_OUT
    // RD7(A12)----------|>O-|__|
    //                       CLC2
    CLCSELECT = 1;   // Select CLC2 registers  
    CLCnPOL = 0x0F;  // G1POL, G2POL, G3POL and G4POL inverted
    // CLC2 data inputs select
    CLCnSEL0 = 0x04; // AND #1 - CLCIN4PPS(RA5 = /RD)
    CLCnSEL1 = 0x7F; // AND #2 - none
    CLCnSEL2 = 0x06; // AND #3 - CLCIN6PPS(RD6 = A14)
    CLCnSEL3 = 0x07; // AND #4 - CLCIN7PPS(RD7 = A15)
    // CLC2 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x00; // none
    CLCnGLS2 = 0x20; // G3D3T enabled
    CLCnGLS3 = 0x80; // G4D4T enabled
    CLCnCON = 0x82;  // Enabled, 4-input AND, no interrupt
    
    // CLC3 logic configuration (4K bytes - Z80 RAM selection - $8000-$8FFF)
    //                        __
    // RD4(A12)----------|>O-|  |
    // RD5(A13)----------|>O-|& |---CLC3_OUT
    // RD6(A14)----------|>O-|  |
    // RD7(A15)--------------|__|
    //                       CLC3
    CLCSELECT = 2;   // Select CLC3 registers  
    CLCnPOL = 0x07;  // G1POL, G2POL, G3POL inverted
    // CLC3 data inputs select
    CLCnSEL0 = 0x02; // AND #1 - CLCIN2PPS(RD6 = A12)
    CLCnSEL1 = 0x03; // AND #2 - CLCIN3PPS(RD6 = A13)
    CLCnSEL2 = 0x06; // AND #3 - CLCIN6PPS(RD6 = A14)
    CLCnSEL3 = 0x07; // AND #4 - CLCIN7PPS(RD7 = A15)
    // CLC3 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x08; // G2D2T enabled
    CLCnGLS2 = 0x20; // G3D3T enabled
    CLCnGLS3 = 0x80; // G4D4T enabled
    CLCnCON = 0x82;  // Enabled, 4-input AND, no interrupt

    // CLC4 logic configuration (4K bytes - Z80 I/O area selection - $E000-$EFFF)
    //                        __
    // RD4(A12)--------------|  |
    // RD5(A13)----------|>O-|& |---CLC4_OUT
    // RD6(A14)----------|>O-|  |
    // RD7(A15)----------|>O-|__|
    //                       CLC4
    CLCSELECT = 3;   // Select CLC4 registers  
    CLCnPOL = 0x01;  // G1POL inverted
    // CLC4 data inputs select
    CLCnSEL0 = 0x02; // AND #1 - CLCIN2PPS(RD4 = A12)
    CLCnSEL1 = 0x03; // AND #2 - CLCIN3PPS(RD5 = A13)
    CLCnSEL2 = 0x06; // AND #3 - CLCIN6PPS(RD6 = A14)
    CLCnSEL3 = 0x07; // AND #4 - CLCIN7PPS(RD7 = A15)
    // CLC4 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x08; // G2D2T enabled
    CLCnGLS2 = 0x20; // G3D3T enabled
    CLCnGLS3 = 0x80; // G4D4T enabled
    CLCnCON = 0x82;  // Enabled, 4-input AND, no interrupt

    // CLC5 logic configuration (memory reading/writing selection)    
    //                        __
    //                       |  |
    // RA5(/RD)--------------|& |---CLC5_OUT
    //                       |__|
    //                       CLC5
    CLCSELECT = 4;   // Select CLC5 registers  
    CLCnPOL = 0x0E;  // G2POL, G3POL and G4POL inverted
    // CLC5 data inputs select
    CLCnSEL0 = 0x04; // AND #1 - CLCIN4PPS(RA5 = /RD)
    CLCnSEL1 = 0x7F; // AND #2 - none
    CLCnSEL2 = 0x7F; // AND #3 - none
    CLCnSEL3 = 0x7F; // AND #4 - none
    // CLC5 gates logic select
    CLCnGLS0 = 0x02; // G1D1T enabled 
    CLCnGLS1 = 0x00; // none
    CLCnGLS2 = 0x20; // G3D3T enabled
    CLCnGLS3 = 0x80; // G4D4T enabled
    CLCnCON = 0x82;  // Enabled, 4-input AND, no interrupt
    
   
    // Z80 clock by NCO FDC mode (RA3)
    RA3PPS = 0x3F;   // RA3 assign NCO1
    NCO1INCU = (unsigned char)((Z80_CLK*2/61/65536) & 0xFF);
    NCO1INCH = (unsigned char)((Z80_CLK*2/61/256) & 0xFF);
    NCO1INCL = (unsigned char)((Z80_CLK*2/61) & 0xFF);
    NCO1CLK = 0x00;  // Clock source Fosc
    NCO1PFM = 0;     // FDC mode
    NCO1OUT = 1;     // NCO output enable
    NCO1EN = 1;      // NCO enable
    
    // Z80 start

    CLCSELECT = 0;

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

    asm(
"\n"    "           movlw	low (_z80rom shr (0+16))"
"\n"    "           movwf	tblptru,c"
"\n"    "           bsf     LATE,1,c"
"\n"    "           movlb   0"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     MAIN_JUMP"
"\n"    "           ALIGN   256"
"\n"    "MAIN_JUMP: movf    PCL,w,c"
"\n"    "WAIT_LOOP:"TEST_SET
"\n"    "POLL_LOOP: btfsc   CLCDATA,0,b"
"\n"    "           bra     POLL_LOOP"
"\n"    "           movf    CLCDATA,w,b"    
"\n"    "          "TEST_CLR
"\n"    "           andlw   0b00011110"
"\n"    "           addwf   PCL,f,c"
"\n"    "           bra     HIZ_R0" // W  = xxx0000x --> RD: Hi-Z            
"\n"    "           bra     ROM_R1" // W  = xxx0001x --> RD: Z80 ROM          
"\n"    "           bra     RAM_R2" // W  = xxx0010x --> RD: Z80 RAM            
"\n"    "           bra     HIZ_R3" // W  = xxx0011x --> RD: Hi-Z         
"\n"    "           bra     IOA_R4" // W  = xxx0100x --> RD: I/O Area            
"\n"    "           bra     HIZ_R5" // W  = xxx0101x --> RD: Hi-Z          
"\n"    "           bra     HIZ_R6" // W  = xxx0110x --> RD: Hi-Z            
"\n"    "           bra     HIZ_R7" // W  = xxx0111x --> RD: Hi-Z         
"\n"    "           bra     HIZ_W0" // W  = xxx1000x --> WR: Hi-Z            
"\n"    "           bra     HIZ_W1" // W  = xxx1001x --> WR: Hi-Z          
"\n"    "           bra     RAM_W2" // W  = xxx1010x --> WR: Z80 RAM            
"\n"    "           bra     HIZ_W3" // W  = xxx1011x --> WR: Hi-Z         
"\n"    "           bra     IOA_W4" // W  = xxx1100x --> WR: I/O Area            
"\n"    "           bra     HIZ_W5" // W  = xxx1101x --> WR: Hi-Z          
"\n"    "           bra     HIZ_W6" // W  = xxx1110x --> WR: Hi-Z            
"\n"    "           bra     HIZ_W7" // W  = xxx1111x --> WR: Hi-Z         
"\n"    "ROM_R1:    movff	PORTB,tblptrl"
"\n"    "           movff	PORTD,tblptrh"
"\n"    "           tblrd	*"
"\n"    "           movff	tablat,LATC"
"\n"    "           clrf	TRISC^1024,c"
"\n"    "           bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"
"\n"    "RAM_R2:    movff   PORTB,fsr2l"
"\n"    "           movf    PORTD,w"
"\n"    "           andlw	15" // (high RAM_SIZE)-1
"\n"    "           addlw	high _z80ram"
"\n"    "           movwf	fsr2h,c"
"\n"    "           movf	indf2,w,c"
"\n"    "           movwf	LATC,c"    
"\n"    "           clrf	TRISC^1024,c"
"\n"    "           bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"       
"\n"    "IOA_R4:    clrf	TRISC^1024,c"
"\n"    "           btfsc	PORTB,0,c"
"\n"    "           bra     IOA_R4_1"       
"\n"    "IOA_R4_0:  movff   U3RXB,LATC"
"\n"    "           bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"
#if USE_RA7_FOR_TEST_PIN
"\n"    "IOA_R4_1:  setf    LATC,c"
#else
"\n"    "IOA_R4_1:  movff   PIR9,LATC"
#endif
"\n"    "           bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"
"\n"    "HIZ_R7:"
"\n"    "HIZ_R6:"
"\n"    "HIZ_R5:"
"\n"    "HIZ_R3:"
"\n"    "HIZ_R0:"
"\n"    "          "TEST_SET
"\n"    "          "TEST_CLR
"\n"    "HIZ_W7:"
"\n"    "HIZ_W6:"
"\n"    "HIZ_W5:"
"\n"    "HIZ_W3:"
"\n"    "HIZ_W1:"
"\n"    "HIZ_W0:"
"\n"    "HIZ_XX:   "TEST_SET
"\n"    "          "TEST_CLR
"\n"    "           setf	TRISC^1024,c"
"\n"    "           bsf     CLCnPOL,3,b"   
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"
"\n"    "RAM_W2:    setf	TRISC^1024,c"
"\n"    "           movff   PORTB,fsr2l"
"\n"    "           movf    PORTD,w"
"\n"    "           andlw	15" // (high RAM_SIZE)-1
"\n"    "           addlw	high _z80ram"
"\n"    "           movwf	fsr2h,c"
"\n"    "           movff	PORTC,indf2"
"\n"    "           bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"       
"\n"    "IOA_W4:    setf	TRISC^1024,c"
"\n"    "           btfsc	PORTB,0,c"
"\n"    "           bra     IOA_W4_1"       
"\n"    "IOA_W4_0:  setf	TRISC^1024,c"
"\n"    "           movff	PORTC,U3TXB"
"\n"    "IOA_W4_1:  bsf     CLCnPOL,3,b"
"\n"    "           bcf     CLCnPOL,3,b"
"\n"    "           bra     WAIT_LOOP"       
    );
}
