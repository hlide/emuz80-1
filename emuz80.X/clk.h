/*
  PIC18F47Q43 ROM RAM and UART emulation firmware

  Target: EMUZ80 - The computer with only Z80 and PIC18F47Q43
  Compiler: MPLAB XC8 v2.36
  Written by Christophe Avoinne
*/

#include "cfg.h"
#include "xc.h"
#include "z80.h"

#include <stdio.h>

#define USE_HARDWARE_CLK_GENERATOR 1

#define Z80_CLK  4500000UL // Z80 clock frequency
#define PIC_CLK 16000000UL // 16MIPS
