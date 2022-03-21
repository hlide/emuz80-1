/*
  PIC18F47Q43 ROM RAM and UART emulation firmware

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Christophe Avoinne
*/

asm("PSECT Z80RAM,class=BIGRAM,reloc=1000h");

unsigned char __section("Z80RAM") z80ram[RAM_SIZE];
