//*****************************************************************************
//*****************************************************************************
//  FILENAME:  PGA_3.h  ( PGA )
//  Version: 3.2, Updated on 2009/10/13 at 23:59:44
//  Generated by PSoC Designer 5.0.1127.0
//
//  DESCRIPTION:  PGA User Module C Language interface file.
//-----------------------------------------------------------------------------
//      Copyright (c) Cypress Semiconductor 2009. All Rights Reserved.
//*****************************************************************************
//*****************************************************************************
#include <M8C.h>

//-------------------------------------------------
// Constants for PGA_3 API's.
//-------------------------------------------------
#define PGA_3_OFF         0
#define PGA_3_LOWPOWER    1
#define PGA_3_MEDPOWER    2
#define PGA_3_HIGHPOWER   3

#define PGA_3_G48_0    0x0C
#define PGA_3_G24_0    0x1C

#define PGA_3_G16_0    0x08
#define PGA_3_G8_00    0x18
#define PGA_3_G5_33    0x28
#define PGA_3_G4_00    0x38
#define PGA_3_G3_20    0x48
#define PGA_3_G2_67    0x58
#define PGA_3_G2_27    0x68
#define PGA_3_G2_00    0x78
#define PGA_3_G1_78    0x88
#define PGA_3_G1_60    0x98
#define PGA_3_G1_46    0xA8
#define PGA_3_G1_33    0xB8
#define PGA_3_G1_23    0xC8
#define PGA_3_G1_14    0xD8
#define PGA_3_G1_06    0xE8
#define PGA_3_G1_00    0xF8
#define PGA_3_G0_93    0xE0
#define PGA_3_G0_87    0xD0
#define PGA_3_G0_81    0xC0
#define PGA_3_G0_75    0xB0
#define PGA_3_G0_68    0xA0
#define PGA_3_G0_62    0x90
#define PGA_3_G0_56    0x80
#define PGA_3_G0_50    0x70
#define PGA_3_G0_43    0x60
#define PGA_3_G0_37    0x50
#define PGA_3_G0_31    0x40
#define PGA_3_G0_25    0x30
#define PGA_3_G0_18    0x20
#define PGA_3_G0_12    0x10
#define PGA_3_G0_06    0x00



#pragma fastcall16 PGA_3_Start
#pragma fastcall16 PGA_3_SetPower
#pragma fastcall16 PGA_3_SetGain
#pragma fastcall16 PGA_3_Stop

//-------------------------------------------------
// Prototypes of the PGA_3 API.
//-------------------------------------------------
extern void PGA_3_Start(BYTE bPowerSetting);     // Proxy class 2
extern void PGA_3_SetPower(BYTE bPowerSetting);  // Proxy class 2
extern void PGA_3_SetGain(BYTE bGainSetting);    // Proxy class 2
extern void PGA_3_Stop(void);                    // Proxy class 1

//-------------------------------------------------
// Register Addresses for PGA_3
//-------------------------------------------------

#pragma ioport  PGA_3_GAIN_CR0: 0x079
BYTE            PGA_3_GAIN_CR0;
#pragma ioport  PGA_3_GAIN_CR1: 0x07a
BYTE            PGA_3_GAIN_CR1;
#pragma ioport  PGA_3_GAIN_CR2: 0x07b
BYTE            PGA_3_GAIN_CR2;
#pragma ioport  PGA_3_GAIN_CR3: 0x078
BYTE            PGA_3_GAIN_CR3;


// end of file PGA_3.h
