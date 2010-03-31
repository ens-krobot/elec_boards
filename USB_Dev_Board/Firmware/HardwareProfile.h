/********************************************************************
 FileName:         HardwareProfile.h
 Dependencies:    See INCLUDES section
 Processor:        PIC18 or PIC24 USB Microcontrollers
 Hardware:        The code is natively intended to be used on the following
                 hardware platforms: PICDEM� FS USB Demo Board, 
                 PIC18F87J50 FS USB Plug-In Module, or
                 Explorer 16 + PIC24 USB PIM.  The firmware may be
                 modified for use on other USB platforms by editing this
                 file (HardwareProfile.h).
 Complier:      Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:        Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the �Company�) for its PIC� Microcontroller is intended and
 supplied to you, the Company�s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN �AS IS� CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Date         Description
  1.0   11/19/2004   Initial release
  2.1   02/26/2007   Updated for simplicity and to use common
                     coding style

********************************************************************/

#ifndef HARDWARE_PROFILE_H
#define HARDWARE_PROFILE_H

    //#define REV_1_0
    #define REV_2_0

    /*******************************************************************/
    /******** USB stack hardware selection options *********************/
    /*******************************************************************/
    //This section is the set of definitions required by the MCHPFSUSB
    //  framework.  These definitions tell the firmware what mode it is
    //  running in, and where it can find the results to some information
    //  that the stack needs.
    //These definitions are required by every application developed with
    //  this revision of the MCHPFSUSB framework.  Please review each
    //  option carefully and determine which options are desired/required
    //  for your application.

    //The PICDEM FS USB Demo Board platform supports the USE_SELF_POWER_SENSE_IO
    //and USE_USB_BUS_SENSE_IO features.  Uncomment the below line(s) if
    //it is desireable to use one or both of the features.
    #if defined(REV_2_0)
        #define USE_SELF_POWER_SENSE_IO
	#endif

    #define tris_self_power     TRISBbits.TRISB7    // Input
    #if defined(USE_SELF_POWER_SENSE_IO)
    #define self_power          PORTBbits.RB7
    #else
    #define self_power          1
    #endif

    #if defined(REV_2_0)
        #define USE_USB_BUS_SENSE_IO
	#endif

    #define tris_usb_bus_sense  TRISBbits.TRISB6    // Input
    #if defined(USE_USB_BUS_SENSE_IO)
    #define USB_BUS_SENSE       PORTBbits.RB6
    #else
    #define USB_BUS_SENSE       1
    #endif

    //Uncomment the following line to make the output HEX of this  
    //  project work with the MCHPUSB Bootloader    
    #define PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER
    
    //Uncomment the following line to make the output HEX of this 
    //  project work with the HID Bootloader
    //#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER        

    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/
    /******** Application specific definitions *************************/
    /*******************************************************************/
    /*******************************************************************/
    /*******************************************************************/

    /** Board definition ***********************************************/
    #define CLOCK_FREQ 48000000

    /** TRIS ***********************************************************/
    #define INPUT_PIN           1
    #define OUTPUT_PIN          0

    /** LED ************************************************************/
	#if defined(REV_1_0)
        #define mInitAllLEDs()      LATE &= 0b11111000; TRISE &= 0b11111000;
        
        #define mLED_1              LATEbits.LATE0
        #define mLED_2              LATEbits.LATE1
        #define mLED_3              LATEbits.LATE2
	#elif defined(REV_2_0)
        #define mInitAllLEDs()      LATC &= 0b11111000; TRISC &= 0b11111000;
    
        #define mLED_1              LATCbits.LATC0
        #define mLED_2              LATCbits.LATC1
        #define mLED_3              LATCbits.LATC2
	#else
		#error Unknown board revision
	#endif

    #define mGetLED_1()         mLED_1
    #define mGetLED_2()         mLED_2
    #define mGetLED_3()         mLED_3

    #define mLED_1_On()         mLED_1 = 1;
    #define mLED_2_On()         mLED_2 = 1;
    #define mLED_3_On()         mLED_3 = 1;

    #define mLED_1_Off()        mLED_1 = 0;
    #define mLED_2_Off()        mLED_2 = 0;
    #define mLED_3_Off()        mLED_3 = 0;

    #define mLED_1_Toggle()     mLED_1 = !mLED_1;
    #define mLED_2_Toggle()     mLED_2 = !mLED_2;
    #define mLED_3_Toggle()     mLED_3 = !mLED_3;

    /** SWITCH *********************************************************/
    #if defined(REV_2_0)
        #define mInitAllSwitches()  TRISBbits.TRISB4=1;TRISBbits.TRISB5=1;
        #define mInitSwitch1()      TRISBbits.TRISB4=1;
        #define mInitSwitch2()      TRISBbits.TRISB5=1;
        #define sw1                 PORTBbits.RB4
        #define sw2                 PORTBbits.RB5
	#endif

#endif  //HARDWARE_PROFILE_H
