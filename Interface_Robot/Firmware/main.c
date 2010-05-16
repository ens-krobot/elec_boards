/**
 * @mainpage Robot Interface
 * @section Introduction
 * Ce firmware est destiné à la carte Robot Interface, équipée d'un PIC18F4550.
 *
 * @section Installation
 * Pour transférer le firmware sur le PIC, procéder de la manière suivante :
 * - Mettre le PIC en mode bootloader (bouton spécifique sur la carte ou commande logiciel avec
 *   le programme RobotMainboard.exe) ;
 * - Ouvrir le programme "PICDEM FS USB Demo Tool" (Pdfsusb) de Microchip (doit être exécuté en tant 
 *   qu'Administrateur sous Windows) ;
 * - Sélectionner la carte dans la liste (la carte doit être connecté en USB au PC et alimentée) ;
 * - Cliquer sur "Load HEX File" pour charger le firmware à transférer au PIC ;
 * - Cliquer sur "Program Device", puis "Execute" pour relancer le PIC avec le nouveau firmware.
 *
 * @note Le programme ne contient pas la configuration du PIC et est destiné à être transféré via un bootloader.
 *
 * @author        Olivier BICHLER
 * @version        1.0
*/

/**
 * @file main.c
 * Contient la fonction main() du programme.
*/

/********************************************************************
 FileName:     main.c
 Dependencies: See INCLUDES section
 Processor:        PIC18 or PIC24 USB Microcontrollers
 Hardware:        The code is natively intended to be used on the following
                 hardware platforms: PICDEM™ FS USB Demo Board, 
                 PIC18F87J50 FS USB Plug-In Module, or
                 Explorer 16 + PIC24 USB PIM.  The firmware may be
                 modified for use on other USB platforms by editing the
                 HardwareProfile.h file.
 Complier:      Microchip C18 (for PIC18) or C30 (for PIC24)
 Company:        Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the “Company”) for its PIC® Microcontroller is intended and
 supplied to you, the Company’s customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
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

#ifndef MAIN_C
#define MAIN_C

/* INCLUDES *******************************************************/
#include <timers.h>     // fonctions pour les timers
#include <delays.h>
#include <usart.h>
#include <i2c.h>

#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "usb_config.h"
#include "./Usb/usb_device.h"
#include "./Usb/usb.h"
#include "./Usb/usb_function_hid.h"

#include "HardwareProfile.h"
#include "PcInterface.h"
#include "eeprom.h"
#include "error.h"
#include "ax12.h"
#include "servo.h"
#include "mcc.h"
#include "adjd-s371.h"
#include "lcd.h"
#include "infrared.h"

/* VARIABLES ******************************************************/
#pragma udata
USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;
BOOL blinkStatusValid = TRUE;
UINT8 UPClientSeq = 0;
char resetSource;

extern volatile BYTE err[32];
extern volatile BYTE errno;
BYTE errnoL = 0;

extern volatile BYTE glbServoEnabled;        // servo.h

extern volatile BOOL glbReceived;
extern volatile BYTE glbBuffer[32];            // ax12.h

#if defined(__18F14K50) || defined(__18F13K50) || defined(__18LF14K50) || defined(__18LF13K50) 
    #pragma udata usbram2
#elif defined(__18F2455) || defined(__18F2550) || defined(__18F4455) || defined(__18F4550)\
    || defined(__18F2458) || defined(__18F2453) || defined(__18F4558) || defined(__18F4553)
    #pragma udata USB_VARIABLES=0x500
#elif defined(__18F4450) || defined(__18F2450)
    #pragma udata USB_VARIABLES=0x480
#else
    #pragma udata
#endif
UP ReceivedDataBuffer;
UP ToSendDataBuffer;
#pragma udata

/* PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
char ResetSource(void);

/** VECTOR REMAPPING ***********************************************/
#if defined(__18CXX)
    //On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
    //the reset, high priority interrupt, and low priority interrupt
    //vectors.  However, the current Microchip USB bootloader 
    //examples are intended to occupy addresses 0x00-0x7FF or
    //0x00-0xFFF depending on which bootloader is used.  Therefore,
    //the bootloader code remaps these vectors to new locations
    //as indicated below.  This remapping is only necessary if you
    //wish to program the hex file generated from this project with
    //the USB bootloader.  If no bootloader is used, edit the
    //usb_config.h file and comment out the following defines:
    //#define PROGRAMMABLE_WITH_USB_HID_BOOTLOADER
    //#define PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER
    
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        #define REMAPPED_RESET_VECTOR_ADDRESS            0x1000
        #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS    0x1008
        #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS    0x1018
    #elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)    
        #define REMAPPED_RESET_VECTOR_ADDRESS            0x800
        #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS    0x808
        #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS    0x818
    #else    
        #define REMAPPED_RESET_VECTOR_ADDRESS            0x00
        #define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS    0x08
        #define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS    0x18
    #endif
    
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
    extern void _startup (void);        // See c018i.c in your C18 compiler dir
    #pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
    void _reset (void)
    {
        _asm goto _startup _endasm;
    }
    #endif
    #pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
    void Remapped_High_ISR (void)
    {
         _asm goto YourHighPriorityISRCode _endasm;
    }
    #pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
    void Remapped_Low_ISR (void)
    {
         _asm goto YourLowPriorityISRCode _endasm;
    }
    
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
    //Note: If this project is built while one of the bootloaders has
    //been defined, but then the output hex file is not programmed with
    //the bootloader, addresses 0x08 and 0x18 would end up programmed with 0xFFFF.
    //As a result, if an actual interrupt was enabled and occured, the PC would jump
    //to 0x08 (or 0x18) and would begin executing "0xFFFF" (unprogrammed space).  This
    //executes as nop instructions, but the PC would eventually reach the REMAPPED_RESET_VECTOR_ADDRESS
    //(0x1000 or 0x800, depending upon bootloader), and would execute the "goto _startup".  This
    //would effective reset the application.
    
    //To fix this situation, we should always deliberately place a 
    //"goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS" at address 0x08, and a
    //"goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS" at address 0x18.  When the output
    //hex file of this project is programmed with the bootloader, these sections do not
    //get bootloaded (as they overlap the bootloader space).  If the output hex file is not
    //programmed using the bootloader, then the below goto instructions do get programmed,
    //and the hex file still works like normal.  The below section is only required to fix this
    //scenario.
    #pragma code HIGH_INTERRUPT_VECTOR = 0x08
    void High_ISR (void)
    {
         _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm;
    }
    #pragma code LOW_INTERRUPT_VECTOR = 0x18
    void Low_ISR (void)
    {
         _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm;
    }
    #endif    //end of "#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)||defined(PROGRAMMABLE_WITH_USB_LEGACY_CUSTOM_CLASS_BOOTLOADER)"

    #pragma code
    
    
    //These are your actual interrupt handling routines.
    #pragma interrupt YourHighPriorityISRCode
    void YourHighPriorityISRCode()
    {
        unsigned char sProdL, sProdH;
    
        // sauvegarde du contenu des registres de calcul
        sProdL = PRODL;
        sProdH = PRODH;

        #if defined(USB_INTERRUPT)
            USBDeviceTasks();
        #endif

        // Interruption par le timer3
        if (PIR2bits.TMR3IF) {
            interruptServo();
    
            // On réautorise l'interruption
            PIR2bits.TMR3IF = 0;
        }

        if (PIR1bits.RCIF) {
            interruptAX12();
    
            // On réautorise l'interruption
             PIR1bits.RCIF = 0;
        }

        #ifdef KROBOT_2010
            // Interruption par le timer0
            if (INTCONbits.TMR0IF) {
                interruptMotor1();
        
                // On réautorise l'interruption
                INTCONbits.TMR0IF = 0;
            }
        #endif

        #ifndef KROBOT_2010
            // Interruption par le timer1
            if (PIR1bits.TMR1IF) {
                interruptMotor2();
        
                // On réautorise l'interruption
                PIR1bits.TMR1IF = 0;
            }
        #endif

        // restauration des registres de calcul
        PRODL = sProdL;
        PRODH = sProdH;
    
    }    //This return will be a "retfie fast", since this is in a #pragma interrupt section 
    #pragma interruptlow YourLowPriorityISRCode
    void YourLowPriorityISRCode()
    {
        unsigned char sProdL, sProdH;
    
        // sauvegarde du contenu des registres de calcul
        sProdL = PRODL;
        sProdH = PRODH;

        #ifndef KROBOT_2010
            // Interruption par le timer0
            if (INTCONbits.TMR0IF) {
                interruptMotor1();
        
                // On réautorise l'interruption
                INTCONbits.TMR0IF = 0;
            }
        #endif

        // restauration des registres de calcul
        PRODL = sProdL;
        PRODH = sProdH;
    
    }    //This return will be a "retfie", since this is in a #pragma interruptlow section 

#elif defined(__C30__)
    #if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
        /*
         *    ISR JUMP TABLE
         *
         *    It is necessary to define jump table as a function because C30 will
         *    not store 24-bit wide values in program memory as variables.
         *
         *    This function should be stored at an address where the goto instructions 
         *    line up with the remapped vectors from the bootloader's linker script.
         *  
         *  For more information about how to remap the interrupt vectors,
         *  please refer to AN1157.  An example is provided below for the T2
         *  interrupt with a bootloader ending at address 0x1400
         */
//        void __attribute__ ((address(0x1404))) ISRTable(){
//        
//            //asm("reset"); //reset instruction to prevent runaway code
//            //asm("goto %0"::"i"(&_T2Interrupt));  //T2Interrupt's address
//        }
    #endif
#endif

#pragma code

/********************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *******************************************************************/
#if defined(__18CXX)
void main(void)
#else
int main(void)
#endif
{
//    //This can be used for user entry into the bootloader  
//    #if defined(__C30__) 
//        mInitSwitch2();
//        if(sw2 == 0)
//        {
//            EnterBootloader();
//        }
//    #endif

    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
        // Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
                          // this function periodically.  This function will take care
                          // of processing and responding to SETUP transactions 
                          // (such as during the enumeration process when you first
                          // plug in).  USB hosts require that USB devices should accept
                          // and process SETUP packets in a timely fashion.  Therefore,
                          // when using polling, this function should be called 
                          // frequently (such as once about every 100 microseconds) at any
                          // time that a SETUP packet might reasonably be expected to
                          // be sent by the host to your device.  In most cases, the
                          // USBDeviceTasks() function does not take very long to
                          // execute (~50 instruction cycles) before it returns.
        #endif
                      

        // Application-specific tasks.
        // Application related code may be added here, or in the ProcessIO() function.
        ProcessIO();        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{
    #if (defined(__18CXX) & !defined(PIC18F87J50_PIM))
        ADCON1 |= 0x0F;                 // Default all pins to digital
    #elif defined(__C30__)
        AD1PCFGL = 0xFFFF;
    #elif defined(__C32__)
        AD1PCFG = 0xFFFF;
    #endif

    #if defined(PIC18F87J50_PIM) || defined(PIC18F46J50_PIM) || defined(PIC18F_STARTER_KIT_1)
    //On the PIC18F87J50 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
    //This allows the device to power up at a lower initial operating frequency, which can be
    //advantageous when powered from a source which is not gauranteed to be adequate for 48MHz
    //operation.  On these devices, user firmware needs to manually set the OSCTUNE<PLLEN> bit to
    //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        OSCTUNEbits.PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while(pll_startup_counter--);
    }
    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif

    #if defined(PIC18F87J50_PIM)
    //Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
    //use the ANCONx registers to control this, which is different from other devices which
    //use the ADCON1 register for this purpose.
    WDTCONbits.ADSHR = 1;            // Select alternate SFR location to access ANCONx registers
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    WDTCONbits.ADSHR = 0;            // Select normal SFR locations
    #endif

    #if defined(PIC18F46J50_PIM) || defined(PIC18F_STARTER_KIT_1)
    //Configure all I/O pins to use digital input buffers.  The PIC18F87J50 Family devices
    //use the ANCONx registers to control this, which is different from other devices which
    //use the ADCON1 register for this purpose.
    ANCON0 = 0xFF;                  // Default all pins to digital
    ANCON1 = 0xFF;                  // Default all pins to digital
    #endif
    
   #if defined(PIC24FJ64GB004_PIM)
    //On the PIC24FJ64GB004 Family of USB microcontrollers, the PLL will not power up and be enabled
    //by default, even if a PLL enabled oscillator configuration is selected (such as HS+PLL).
    //This allows the device to power up at a lower initial operating frequency, which can be
    //advantageous when powered from a source which is not gauranteed to be adequate for 32MHz
    //operation.  On these devices, user firmware needs to manually set the CLKDIV<PLLEN> bit to
    //power up the PLL.
    {
        unsigned int pll_startup_counter = 600;
        CLKDIVbits.PLLEN = 1;
        while(pll_startup_counter--);
    }

    //Device switches over automatically to PLL output after PLL is locked and ready.
    #endif


//    The USB specifications require that USB peripheral devices must never source
//    current onto the Vbus pin.  Additionally, USB peripherals should not source
//    current on D+ or D- when the host/hub is not actively powering the Vbus line.
//    When designing a self powered (as opposed to bus powered) USB peripheral
//    device, the firmware should make sure not to turn on the USB module and D+
//    or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//    firmware needs some means to detect when Vbus is being powered by the host.
//    A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
//     can be used to detect when Vbus is high (host actively powering), or low
//    (host is shut down or otherwise not supplying power).  The USB firmware
//     can then periodically poll this I/O pin to know when it is okay to turn on
//    the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//    peripheral device, it is not possible to source current on D+ or D- when the
//    host is not actively providing power on Vbus. Therefore, implementing this
//    bus sense feature is optional.  This firmware can be made to use this bus
//    sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//    HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//    If the host PC sends a GetStatus (device) request, the firmware must respond
//    and let the host know if the USB peripheral device is currently bus powered
//    or self powered.  See chapter 9 in the official USB specifications for details
//    regarding this request.  If the peripheral device is capable of being both
//    self and bus powered, it should not return a hard coded value for this request.
//    Instead, firmware should check if it is currently self or bus powered, and
//    respond accordingly.  If the hardware has been configured like demonstrated
//    on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//    currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//    is used for    this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//    has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
//    to it in HardwareProfile.h.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;    // See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();    //usb_device.c.  Initializes USB module SFRs and firmware
                        //variables to known states.
}//end InitializeSystem


/******************************************************************************
 * Function:        void UserInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This routine should take care of all of the demo code
 *                  initialization that is required.
 *
 * Note:            
 *
 *****************************************************************************/
void UserInit(void)
{
    BYTE eeprom;

    // ResetSource() permet de connaitre l'origine du dernier reset tout en réinitialisant correctement
    // les registres pour le prochain reset. Si elle n'est pas appelée à chaque démarrage du PIC, les
    // registres ne seront pas correctement réinitialisées et l'origine du reset renvoyée par cette
    // fonction pourra être erronée.
    resetSource = ResetSource();

    // Fail-safe to be able to enter the bootloader
    if (resetSource == RESET_SOURCE_MCLR) {
        // We check the reset source to avoid entering the bootloader mode on
        // events such as a Power-on reset followed by a Brown-out reset
        eeprom = ReadEEPROM(0x00);
        eeprom|= 0b1;
        WriteEEPROM(0x00, eeprom);
        Delay10KTCYx(0);
        eeprom&= ~0b1;
        WriteEEPROM(0x00, eeprom);
    }

    // Configuration du port B
    TRISBbits.TRISB0 = 1;    // SDA
    TRISBbits.TRISB1 = 1;    // SCL

    // Configuration du port C
    TRISCbits.TRISC6 = 1;    // TX
    TRISCbits.TRISC7 = 1;    // RX

    // Configuration interruptions
    RCONbits.IPEN = 1;        // Enable priority levels on interrupts
    INTCONbits.GIE = 1;        // Enables all high priority interrupts
    INTCONbits.PEIE = 1;    // Enables all low priority peripheral interrupts

    /**
     * Initialisation des composants externes
    */

    // Configuration de l'I2C
    SSPADD = 119;    // Vitesse de l'I2C = 100 KHz ( = FOSC/[4*(SSPADD+1)] )

    // Initialisation et effacement de l'afficheur LCD
    lcd_init(ADD_LCD);

    //Initialize all of the LED pins
    mInitAllLEDs();

    //initialize the variable holding the handle for the last
    // transmission
    USBOutHandle = 0;
    USBInHandle = 0;

    blinkStatusValid = TRUE;

    initAX12();
    initServos();
    initMCC(0);
    initAdjd();
    initIF();

}//end UserInit

/**
 * Fonction de traitement principale du programme.
*/
void ProcessIO(void) {   
    BYTE eeprom;
    UINT i;
    WORD_VAL colRed, colGreen, colBlue, colClear;
    DWORD_VAL dword;
    WORD_VAL word1, word2;
    UINT count;
    BOOL timeOut;
    BOOL traj_engaged = FALSE;

    //Blink the LEDs according to the USB device status
    if(blinkStatusValid) {
        BlinkUSBStatus();
    }

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    #ifdef KROBOT_2010
        interruptIF();
    #endif

/*
    if (!HIDTxHandleBusy(USBInHandle) && glbReceived) {
        ToSendDataBuffer.HSEQ    = 0;                        // Numéro séquence PC
        ToSendDataBuffer.DSEQ    = (UPClientSeq++);            // Numéro séquence PIC
        ToSendDataBuffer.CMD    = CMD_SEND;                    // Type requête
        ToSendDataBuffer.ERR    = 0;                        // Erreur

        ToSendDataBuffer.DATA[0] = '\0';
        sprintf((char *) &ToSendDataBuffer.DATA[0], "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", 
            glbBuffer[0], glbBuffer[1], glbBuffer[2], glbBuffer[3], glbBuffer[4], glbBuffer[5], glbBuffer[6], glbBuffer[7], 
            glbBuffer[8], glbBuffer[9], glbBuffer[10], glbBuffer[11], glbBuffer[12], glbBuffer[13], glbBuffer[14], glbBuffer[15]);

        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);

        glbReceived = 0;
//        test = 0;
    }
*/

    if (traj_engaged && checkTOR() && !HIDTxHandleBusy(USBInHandle)) {
        ToSendDataBuffer.HSEQ    = 0;                        // Numéro séquence PC
        ToSendDataBuffer.DSEQ    = (UPClientSeq++);            // Numéro séquence PIC
        ToSendDataBuffer.CMD    = CMD_MOTOR_TOR;            // Type requête
        ToSendDataBuffer.ERR    = 0;                        // Erreur
        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
    
        traj_engaged = FALSE;
    }

    if (!HIDTxHandleBusy(USBInHandle) && (err[errnoL] != 0)) {
        ToSendDataBuffer.HSEQ    = 0;                        // Numéro séquence PC
        ToSendDataBuffer.DSEQ    = (UPClientSeq++);            // Numéro séquence PIC
        ToSendDataBuffer.CMD    = CMD_ERR;                    // Type requête
        ToSendDataBuffer.ERR    = err[errnoL];                // Erreur
        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);

        err[errnoL] = 0;

        errnoL = (errnoL + 1) % 32;
    }

    if(!HIDRxHandleBusy(USBOutHandle)) {
        switch (ReceivedDataBuffer.CMD) {
            case CMD_RESET:
                // Reset
                Reset();
            break;

            case CMD_BOOTLOADER:
                // Reset sur le bootloader
                eeprom = ReadEEPROM(0x00);
                eeprom|= 0b1;
                WriteEEPROM(0x00, eeprom);
                Reset();
            break;

            case CMD_GET:
                if (!HIDTxHandleBusy(USBInHandle)) {
                    ToSendDataBuffer.HSEQ    = ReceivedDataBuffer.HSEQ;    // Numéro séquence PC
                    ToSendDataBuffer.DSEQ    = 0;                            // Numéro séquence PIC
                    ToSendDataBuffer.CMD    = CMD_RESPOND;                    // Type requête
                    ToSendDataBuffer.ERR    = 0;                            // Erreur

                    switch (ReceivedDataBuffer.DATA[0]) {
                        case GET_RESET_SOURCE:
                            ToSendDataBuffer.DATA[0] = resetSource;
                        break;
    
                        case GET_BOARD_INFO:
                            ToSendDataBuffer.DATA[0] = '\0';
                            strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) "Robot Interface 1.0\nOlivier BICHLER");
                        break;
    
                        case GET_FIRMWARE_BUILD:
                            ToSendDataBuffer.DATA[0] = '\0';
                            strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) __DATE__);
                            strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) " ");
                            strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) __TIME__);
                        break;
    
                        case GET_PORTS_CONFIG:
                            ToSendDataBuffer.DATA[0]    = TRISA;
                            ToSendDataBuffer.DATA[1]    = TRISB;
                            ToSendDataBuffer.DATA[2]    = TRISC;
                            ToSendDataBuffer.DATA[3]    = TRISD;
                            ToSendDataBuffer.DATA[4]    = TRISE;
                        break;
    
                        case GET_PORTS_STATE:
                            ToSendDataBuffer.DATA[0]    = PORTA;
                            ToSendDataBuffer.DATA[1]    = PORTB;
                            ToSendDataBuffer.DATA[2]    = PORTC;
                            ToSendDataBuffer.DATA[3]    = PORTD;
                            ToSendDataBuffer.DATA[4]    = PORTE;
                        break;

                        case GET_CMP03_DATA:
                            OpenI2C(MASTER, SLEW_OFF);
                            
                            StartI2C();                                // send start bit
                            IdleI2C();                                // and wait for it to clear
                            WriteI2C(0xC0);                            // 11000000 - write command
                            WriteI2C(0);                            // read from address 0 

                            RestartI2C();
                            IdleI2C();
                            WriteI2C(0xC1);                            // 11000001 - read command

                            // Récupère N° de révision du module CMP03
                            // Récupère la valeur de l'angle sur un octet (0 - 255)
                            // Récupère l'octet de poids fort de l'angle (0 - 3599)
                            // Récupère l'octet de poids faible de l'angle (0 - 3599)
                            for (i = 0; i < 4; i++) {
                                timeOut = TRUE;

                                SSPCON2bits.RCEN = 1;                    // start receiving

                                for (count = 0; count < 1000; count++) {
                                    if (DataRdyI2C()) {
                                        ToSendDataBuffer.DATA[i] = SSPBUF;
                                        timeOut = FALSE;
                                        break;
                                    }
                                }

                                if (timeOut) {
                                    ToSendDataBuffer.ERR = ERR_CMP03_NOT_RESPONDING;
                                    NotAckI2C();            // not acknowledge for last byte
                                    IdleI2C();                       // wait for ack. sequence to end
                                    break;
                                }
    
                                // Il n'y a pas d'acknowledge si l'octet que l'on lit est le dernier
                                // Sinon il est nécessaire d'en faire un, car si l'esclave en attend un
                                // et que l'on ne l'envoie pas, il peut rester planté à l'attendre indéfiniment...
                                if (i < 3)
                                    AckI2C();               // start acknowledge sequence
                                else
                                    NotAckI2C();            // not acknowledge for last byte

                                IdleI2C();                       // wait for ack. sequence to end
                            }

                            StopI2C();
                            IdleI2C();
                            CloseI2C();

/*
                            OpenI2C(MASTER, SLEW_OFF);

                            SSPCON2bits.SEN = 1;                    // send start bit
                            while(SSPCON2bits.SEN);                    // and wait for it to clear
                            SSPCON2bits.ACKDT = 0;                    // acknowledge bit
                        
                            PIR1bits.SSPIF = 0;
                            SSPBUF = 0xC0;                            // 11000000 - write command
                            while(!PIR1bits.SSPIF);                    // wait for interrupt
                            PIR1bits.SSPIF = 0;                        // then clear it.
                        
                            SSPBUF = 0;                                // read from address 0 
                            while(!PIR1bits.SSPIF);                    // 
                            PIR1bits.SSPIF = 0;                        // 
                        
                            SSPCON2bits.RSEN = 1;                    // send repeated start bit
                            while(SSPCON2bits.RSEN);                // and wait for it to clear
                        
                            PIR1bits.SSPIF = 0;
                            SSPBUF = 0xC1;                            // 11000001 - read command
                            while(!PIR1bits.SSPIF);                    // wait for interrupt
                            PIR1bits.SSPIF = 0;                        // then clear it.
                                
                            SSPCON2bits.RCEN = 1;                    // start receiving
                            while(!SSPSTATbits.BF);                    // wait for data
                            ToSendDataBuffer.DATA[0] = SSPBUF;        // and get it
                            SSPCON2bits.ACKEN = 1;                    // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);                // wait for ack. sequence to end
                            
                            SSPCON2bits.RCEN = 1;                    // start receiving
                            while(!SSPSTATbits.BF);                    // wait for data
                            ToSendDataBuffer.DATA[1] = SSPBUF;    // and get it
                            SSPCON2bits.ACKEN = 1;                    // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);                // wait for ack. sequence to end
                        
                            SSPCON2bits.RCEN = 1;                    // start receiving
                            while(!SSPSTATbits.BF);                    // wait for data
                            ToSendDataBuffer.DATA[2] = SSPBUF;    // and get it
                            SSPCON2bits.ACKEN = 1;                    // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);                // wait for ack. sequence to end
                        
                            SSPCON2bits.RCEN = 1;                    // start receiving
                            while(!SSPSTATbits.BF);                    // wait for data
                            ToSendDataBuffer.DATA[3] = SSPBUF;    // and get it
                            SSPCON2bits.ACKDT = 1;                    // not acknowledge for last byte
                            SSPCON2bits.ACKEN = 1;                    // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);                // wait for ack. sequence to end
                        
                            SSPCON2bits.PEN = 1;                    // send stop bit
                            while(SSPCON2bits.PEN);                    //

                            CloseI2C();
*/
                        break;

                        case GET_RANGEFINDER_STATE:
                            for (i = 0; i < 4; i++) {
                                dword.Val = getIFRange(i);
                                ToSendDataBuffer.DATA[4*i] = dword.byte.MB;
                                ToSendDataBuffer.DATA[4*i + 1] = dword.byte.UB;
                                ToSendDataBuffer.DATA[4*i + 2] = dword.byte.HB;
                                ToSendDataBuffer.DATA[4*i + 3] = dword.byte.LB;
                            }
                        break;

                        default:
                            ToSendDataBuffer.ERR = ERR_UNKNOWN_GET;
                    }

                    USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
                    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
                }
            break;

            case CMD_SET:
                switch (ReceivedDataBuffer.DATA[0]) {
                    case SET_PORTS_CONFIG_INPUTS:
                        TRISA|= ReceivedDataBuffer.DATA[1];
                        TRISB|= ReceivedDataBuffer.DATA[2];
                        TRISC|= ReceivedDataBuffer.DATA[3];
                        TRISD|= ReceivedDataBuffer.DATA[4];
                        TRISE|= ReceivedDataBuffer.DATA[5];
                    break;

                    case SET_PORTS_CONFIG_OUTPUTS:
                        TRISA&= ~ReceivedDataBuffer.DATA[1];
                        TRISB&= ~ReceivedDataBuffer.DATA[2];
                        TRISC&= ~ReceivedDataBuffer.DATA[3];
                        TRISD&= ~ReceivedDataBuffer.DATA[4];
                        TRISE&= ~ReceivedDataBuffer.DATA[5];
                    break;

                    case SET_PORTS_STATE_LOW:
                        LATA&= ~ReceivedDataBuffer.DATA[1];
                        LATB&= ~ReceivedDataBuffer.DATA[2];
                        LATC&= ~ReceivedDataBuffer.DATA[3];
                        LATD&= ~ReceivedDataBuffer.DATA[4];
                        LATE&= ~ReceivedDataBuffer.DATA[5];
                    break;

                    case SET_PORTS_STATE_HIGH:
                        LATA|= ReceivedDataBuffer.DATA[1];
                        LATB|= ReceivedDataBuffer.DATA[2];
                        LATC|= ReceivedDataBuffer.DATA[3];
                        LATD|= ReceivedDataBuffer.DATA[4];
                        LATE|= ReceivedDataBuffer.DATA[5];
                    break;

                    case SET_SERVO_CONFIG:
                        // On active d'abord puis on désactive. Ainsi si on demande à la fois d'activer et désactiver
                        // un même servo, celui-ci reste désactivé (par mesure de sécurité).
                        glbServoEnabled|= ReceivedDataBuffer.DATA[1];        // Servo à activer
                        glbServoEnabled&= ~ReceivedDataBuffer.DATA[2];        // Servo à désactiver
                    break;

                    case SET_SERVO_STATE:
                        if (ReceivedDataBuffer.DATA[1] & 1)
                            setServoAngle(0, (char) ReceivedDataBuffer.DATA[2]);

                        if (ReceivedDataBuffer.DATA[1] & 2)
                            setServoAngle(1, (char) ReceivedDataBuffer.DATA[3]);

                        if (ReceivedDataBuffer.DATA[1] & 4)
                            setServoAngle(2, (char) ReceivedDataBuffer.DATA[4]);

                        if (ReceivedDataBuffer.DATA[1] & 8)
                            setServoAngle(3, (char) ReceivedDataBuffer.DATA[5]);

                        if (ReceivedDataBuffer.DATA[1] & 16)
                            setServoAngle(4, (char) ReceivedDataBuffer.DATA[6]);
                    break;

                    default:
                        error(ERR_UNKNOWN_SET);
                }

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
            break;

            case CMD_AX12:
                if (!HIDTxHandleBusy(USBInHandle)) {
                    ToSendDataBuffer.HSEQ    = ReceivedDataBuffer.HSEQ;    // Numéro séquence PC
                    ToSendDataBuffer.DSEQ    = 0;                            // Numéro séquence PIC
                    ToSendDataBuffer.CMD    = CMD_RESPOND;                    // Type requête
                    ToSendDataBuffer.ERR    = 0;                            // Erreur
        
                    switch (ReceivedDataBuffer.DATA[0]) {
                        case AX12_PING:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            ToSendDataBuffer.DATA[0] = pingAX12(ReceivedDataBuffer.DATA[1], word1.Val);
                        break;
    
                        case AX12_READ8:
                            word1.byte.HB = ReceivedDataBuffer.DATA[3];
                            word1.byte.LB = ReceivedDataBuffer.DATA[4];
                            ToSendDataBuffer.DATA[0] = readValue8(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], word1.Val);
                        break;

                        case AX12_READ16:
                            word1.byte.HB = ReceivedDataBuffer.DATA[3];
                            word1.byte.LB = ReceivedDataBuffer.DATA[4];
                            word2.Val = readValue16(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], word1.Val);
                            ToSendDataBuffer.DATA[0] = word2.byte.HB;
                            ToSendDataBuffer.DATA[1] = word2.byte.LB;
                        break;
    
                        case AX12_WRITE8:
                            writeValue8(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], ReceivedDataBuffer.DATA[3]);
                        break;

                        case AX12_WRITE16:
                            word1.byte.HB = ReceivedDataBuffer.DATA[3];
                            word1.byte.LB = ReceivedDataBuffer.DATA[4];
                            writeValue16(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], word1.Val);
                        break;
    
                        case AX12_GOTO:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            word2.byte.HB = ReceivedDataBuffer.DATA[4];
                            word2.byte.LB = ReceivedDataBuffer.DATA[5];
                            goTo(ReceivedDataBuffer.DATA[1], word1.Val, word2.Val, ReceivedDataBuffer.DATA[6]);
                        break;
    
                        case AX12_GET_POS:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            word2.Val = getPosition(ReceivedDataBuffer.DATA[1], word1.Val);
                            ToSendDataBuffer.DATA[0] = word2.byte.HB;
                            ToSendDataBuffer.DATA[1] = word2.byte.LB;
                        break;
    
                        case AX12_GET_SPEED:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            word2.Val = getSpeed(ReceivedDataBuffer.DATA[1], word1.Val);
                            ToSendDataBuffer.DATA[0] = word2.byte.HB;
                            ToSendDataBuffer.DATA[1] = word2.byte.LB;
                        break;
    
                        case AX12_GET_LOAD:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            word2.Val = readValue16(ReceivedDataBuffer.DATA[1], P_PRESENT_LOAD, word1.Val);
                            ToSendDataBuffer.DATA[0] = word2.byte.HB;
                            ToSendDataBuffer.DATA[1] = word2.byte.LB;
                        break;
    
                        case AX12_GET_STATS:
                            word1.byte.HB = ReceivedDataBuffer.DATA[2];
                            word1.byte.LB = ReceivedDataBuffer.DATA[3];
                            // Position
                            word2.Val = getPosition(ReceivedDataBuffer.DATA[1], word1.Val);
                            ToSendDataBuffer.DATA[0] = word2.byte.HB;
                            ToSendDataBuffer.DATA[1] = word2.byte.LB;
                            // Speed
                            word2.Val = getSpeed(ReceivedDataBuffer.DATA[1], word1.Val);
                            ToSendDataBuffer.DATA[2] = word2.byte.HB;
                            ToSendDataBuffer.DATA[3] = word2.byte.LB;
                            // Torque
                            word2.Val = readValue16(ReceivedDataBuffer.DATA[1], P_PRESENT_LOAD, word1.Val);
                            ToSendDataBuffer.DATA[4] = word2.byte.HB;
                            ToSendDataBuffer.DATA[5] = word2.byte.LB;
                            // Voltage
                            ToSendDataBuffer.DATA[6] = readValue8(ReceivedDataBuffer.DATA[1], P_PRESENT_VOLTAGE, word1.Val);
                            // Temperature
                            ToSendDataBuffer.DATA[7] = readValue8(ReceivedDataBuffer.DATA[1], P_PRESENT_TEMPERATURE, word1.Val);
                            // CW Angle Limit
                            word2.Val = readValue16(ReceivedDataBuffer.DATA[1], P_CW_ANGLE_LIMIT, word1.Val);
                            ToSendDataBuffer.DATA[8] = word2.byte.HB;
                            ToSendDataBuffer.DATA[9] = word2.byte.LB;
                            // CCW Angle Limit
                            word2.Val = readValue16(ReceivedDataBuffer.DATA[1], P_CCW_ANGLE_LIMIT, word1.Val);
                            ToSendDataBuffer.DATA[10] = word2.byte.HB;
                            ToSendDataBuffer.DATA[11] = word2.byte.LB;
                        break;
    
                        case AX12_WRITE_REG8:
                            regWrite8(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], ReceivedDataBuffer.DATA[3]);
                        break;

                        case AX12_WRITE_REG16:
                            word1.byte.HB = ReceivedDataBuffer.DATA[3];
                            word1.byte.LB = ReceivedDataBuffer.DATA[4];
                            regWrite16(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], word1.Val);
                        break;
    
                        case AX12_ACTION:
                            actionAX12(ReceivedDataBuffer.DATA[1]);
                        break;
    
                        case AX12_RESET:
                            resetAX12(ReceivedDataBuffer.DATA[1]);
                        break;

                        case AX12_CONFIG:
                            configAX12(ReceivedDataBuffer.DATA[1]);
                        break;

                        default:
                            ToSendDataBuffer.ERR = ERR_UNKNOWN_CMD;
                    }

                    USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
                    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
                }        
            break;

            case CMD_LCD:
                switch (ReceivedDataBuffer.DATA[0]) {
                    case LCD_CLEAR:
                        lcd_clear(ADD_LCD);
                    break;
                    
                    case LCD_CURSOR_ON:
                        lcd_set_cursor(ADD_LCD, TRUE);
                    break;
                    
                    case LCD_CURSOR_OFF:
                        lcd_set_cursor(ADD_LCD, FALSE);
                    break;
                    
                    case LCD_BACKLIGHT_ON:
                        lcd_set_backlight(ADD_LCD, TRUE);
                    break;
                    
                    case LCD_BACKLIGHT_OFF:
                        lcd_set_backlight(ADD_LCD, FALSE);
                    break;
                    
                    case LCD_GOTO_POS:
                        lcd_goto(ADD_LCD, ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2]);
                    break;
                    
                    case LCD_WRITE:
                        lcd_write(ADD_LCD, &ReceivedDataBuffer.DATA[1]);
                    break;
                    
                    case LCD_WRITE_LINE:
                        lcd_write_line(ADD_LCD, ReceivedDataBuffer.DATA[1], &ReceivedDataBuffer.DATA[2]);
                    break;

                    default:
                        error(ERR_UNKNOWN_CMD);
                }

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
            break;

            case CMD_MOTOR:
                switch (ReceivedDataBuffer.DATA[0]) {
                    case MOTOR_ENABLE:
                        enableMotor(ReceivedDataBuffer.DATA[1]);
                    break;

                    case MOTOR_DISABLE:
                        disableMotor(ReceivedDataBuffer.DATA[1]);
                    break;

                    case MOTOR_MOVE:
                        dword.byte.MB = ReceivedDataBuffer.DATA[4];
                        dword.byte.UB = ReceivedDataBuffer.DATA[5];
                        dword.byte.HB = ReceivedDataBuffer.DATA[6];
                        dword.byte.LB = ReceivedDataBuffer.DATA[7];
                        move(ReceivedDataBuffer.DATA[1], ReceivedDataBuffer.DATA[2], ReceivedDataBuffer.DATA[3], dword.Val);

                        traj_engaged = TRUE;
                    break;

                    default:
                        error(ERR_UNKNOWN_CMD);
                }

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet   
            break;

            case CMD_SEND:
                OpenI2C(MASTER, SLEW_OFF);
                StartI2C();
                IdleI2C();        // Absolument nécessaire
                WriteI2C(0x00);
                putsI2C(&ReceivedDataBuffer.DATA[0]);
                StopI2C();
                CloseI2C();

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
            break;

            case CMD_TEST:
                // MOTORS TEST
                ////////////////////////////////////////////////////////////////////////
/*
                enableMotor(MOTOR_RIGHT);
                move(MOTOR_RIGHT, 0, 255, 0);
*/
                // ADJD-S371 TEST
                ////////////////////////////////////////////////////////////////////////
/*
                // Sensor digital values can be acquired by writing 01H to CTRL register (address 00H).
                // Read CTRL register. When the value in CTRL register is 00H, sensor digital values are acquired in sensor sample data registers.
                adjdWriteRegister(CTRL, 0x01);

                timeOut = TRUE;

                for (count = 0; count < 1000; count++) {
                    if (adjdReadRegister(CTRL) == 0) {
                        timeOut = FALSE;
                        break;
                    }
                }

                ToSendDataBuffer.HSEQ    = 0;                        // Numéro séquence PC
                ToSendDataBuffer.DSEQ    = (UPClientSeq++);            // Numéro séquence PIC
                ToSendDataBuffer.CMD    = CMD_SEND;                    // Type requête
                ToSendDataBuffer.ERR    = 0;                        // Erreur

                if (timeOut)
                    ToSendDataBuffer.ERR    = ERR_ADJD_S371_NOT_RESPONDING;
                else {
                    colRed.byte.LB = adjdReadRegister(DATA_RED_LO);
                    colRed.byte.HB = adjdReadRegister(DATA_RED_HI);
                    colGreen.byte.LB = adjdReadRegister(DATA_GREEN_LO);
                    colGreen.byte.HB = adjdReadRegister(DATA_GREEN_HI);
                    colBlue.byte.LB = adjdReadRegister(DATA_BLUE_LO);
                    colBlue.byte.HB = adjdReadRegister(DATA_BLUE_HI);
                    colClear.byte.LB = adjdReadRegister(DATA_CLEAR_LO);
                    colClear.byte.HB = adjdReadRegister(DATA_CLEAR_HI);
            
                    ToSendDataBuffer.DATA[0] = '\0';
                    sprintf((char *) &ToSendDataBuffer.DATA[0], "Red: %04x\nGreen: %04x\nBlue: %04x\nClear: %04x", colRed.Val, colGreen.Val, colBlue.Val, colClear.Val);
                }
        
                USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
*/
                // AX12 TEST INITIALIZATION
                ////////////////////////////////////////////////////////////////////////

                CloseUSART();
                OpenUSART(USART_TX_INT_OFF
                    & USART_RX_INT_ON
                    & USART_ASYNCH_MODE
                    & USART_EIGHT_BIT
                    & USART_BRGH_HIGH,
                    2); // FOSC/[16 (n + 1)] = 1000000 bauds

                setParam(0, 0x04);
                setParam(1, 0x10);
                sendInstPacket(1, INST_WRITE, 2);

                setParam(0, 0x04);
                setParam(1, 0x10);
                sendInstPacket(1, INST_WRITE, 2);

                initAX12();


                // AX12 TEST
                ////////////////////////////////////////////////////////////////////////

/*
                setParam(0, 0x1E);
                setParam(1, 0xFF);
                sendInstPacket(1, INST_WRITE, 2);

                setParam(0, 0x1E);
                setParam(1, 0x1F);
                sendInstPacket(1, INST_WRITE, 2);
*/
/*
                setParam(0, 0x04);
                setParam(1, 0x10);
                sendInstPacket(1, INST_WRITE, 2);
*/
                
//                writeValue8(ID_BROADCAST, P_BAUD_RATE, 0x10);
                //resetAX12(1);

                // FF FF 01 04 03 04 10 CS

//                    waitStatusPacket();

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
            break;

            default:
                // Commande non reconnue
                if (!HIDTxHandleBusy(USBInHandle)) {
                    ToSendDataBuffer.HSEQ = ReceivedDataBuffer.HSEQ;    // Numéro séquence PC
                    ToSendDataBuffer.DSEQ = 0;                                // Numéro séquence PIC
                    ToSendDataBuffer.CMD = CMD_RESPOND;                        // Type requête
                    ToSendDataBuffer.ERR = ERR_UNKNOWN_CMD;                    // Erreur

                    USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
                    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);        // Re-arm the OUT endpoint for the next packet
                }
        }
    }
}//end ProcessIO

/********************************************************************
 * Function:        void BlinkUSBStatus(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        BlinkUSBStatus turns on and off LEDs 
 *                  corresponding to the USB device state.
 *
 * Note:            mLED macros can be found in HardwareProfile.h
 *                  USBDeviceState is declared and updated in
 *                  usb_device.c.
 *******************************************************************/
void BlinkUSBStatus(void)
{
    static WORD led_count=0;
    
    if(led_count == 0)led_count = 10000U;
    led_count--;

    #define mLED_Both_Off()         {mLED_1_Off();mLED_2_Off();}
    #define mLED_Both_On()          {mLED_1_On();mLED_2_On();}
    #define mLED_Only_1_On()        {mLED_1_On();mLED_2_Off();}
    #define mLED_Only_2_On()        {mLED_1_Off();mLED_2_On();}

    if(USBSuspendControl == 1)
    {
        if(led_count==0)
        {
            mLED_1_Toggle();
            if(mGetLED_1())
            {
                mLED_2_On();
            }
            else
            {
                mLED_2_Off();
            }
        }//end if
    }
    else
    {
        if(USBDeviceState == DETACHED_STATE)
        {
            mLED_Both_Off();
        }
        else if(USBDeviceState == ATTACHED_STATE)
        {
            mLED_Both_On();
        }
        else if(USBDeviceState == POWERED_STATE)
        {
            mLED_Only_1_On();
        }
        else if(USBDeviceState == DEFAULT_STATE)
        {
            mLED_Only_2_On();
        }
        else if(USBDeviceState == ADDRESS_STATE)
        {
            if(led_count == 0)
            {
                mLED_1_Toggle();
                mLED_2_Off();
            }//end if
        }
        else if(USBDeviceState == CONFIGURED_STATE)
        {
            if(led_count==0)
            {
                mLED_1_Toggle();
                if(mGetLED_1())
                {
                    mLED_2_Off();
                }
                else
                {
                    mLED_2_On();
                }
            }//end if
        }//end if(...)
    }//end if(UCONbits.SUSPND...)

}//end BlinkUSBStatus




// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions USBCBxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the USBCBSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the USBCBSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the USBCBWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the USBCBSuspend() function.

// The USBCBSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        void USBCBSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
void USBCBSuspend(void)
{
    //Example power saving code.  Insert appropriate code here for the desired
    //application behavior.  If the microcontroller will be put to sleep, a
    //process similar to that shown below may be used:
    
    //ConfigureIOPinsForLowPower();
    //SaveStateOfAllInterruptEnableBits();
    //DisableAllInterruptEnableBits();
    //EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();    //should enable at least USBActivityIF as a wake source
    //Sleep();
    //RestoreStateOfAllPreviouslySavedInterruptEnableBits();    //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.
    //RestoreIOPinsToNormal();                                    //Preferrably, this should be done in the USBCBWakeFromSuspend() function instead.

    //IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is 
    //cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause 
    //things to not work as intended.    

    unsigned char msg1[] = "[Kro]bot       Wally";
    unsigned char msg2[] = "!!!!!!!!!!!!!!!!!!!!";
    unsigned char msg3[] = "Perte connectivite  ";
    unsigned char msg4[] = "          carte mere";
    
    // Turn off LCD backlight and display status
    //lcd_clear(ADD_LCD);
    lcd_set_cursor(ADD_LCD, FALSE);
    lcd_set_backlight(ADD_LCD, FALSE);
    lcd_write_line(ADD_LCD, 1, msg1);
    lcd_write_line(ADD_LCD, 2, msg2);
    lcd_write_line(ADD_LCD, 3, msg3);
    lcd_write_line(ADD_LCD, 4, msg4);

    #if defined(__C30__)
    #if 0
        U1EIR = 0xFFFF;
        U1IR = 0xFFFF;
        U1OTGIR = 0xFFFF;
        IFS5bits.USB1IF = 0;
        IEC5bits.USB1IE = 1;
        U1OTGIEbits.ACTVIE = 1;
        U1OTGIRbits.ACTVIF = 1;
        Sleep();
    #endif
    #endif
}


/******************************************************************************
 * Function:        void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *                    In this example the interrupt is only used when the device
 *                    goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;
        
            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        void USBCBWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *                    suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *                    mode, the host may wake the device back up by sending non-
 *                    idle state signalling.
 *                    
 *                    This call back is invoked when a wakeup from USB suspend 
 *                    is detected.
 *
 * Note:            None
 *****************************************************************************/
void USBCBWakeFromSuspend(void)
{
    // If clock switching or other power savings measures were taken when
    // executing the USBCBSuspend() function, now would be a good time to
    // switch back to normal full power run mode conditions.  The host allows
    // a few milliseconds of wakeup time, after which the device must be 
    // fully back to normal, and capable of receiving and processing USB
    // packets.  In order to do this, the USB module must receive proper
    // clocking (IE: 48MHz clock must be available to SIE for full speed USB
    // operation).

    unsigned char msg1[] = "[Kro]bot       Wally";
    unsigned char msg2[] = "--------------------";
    unsigned char msg3[] = "Connexion etablie   ";
    unsigned char msg4[] = "  avec la carte mere";
    

    // Turn on LCD backlight and display status
    lcd_clear(ADD_LCD);
    lcd_set_cursor(ADD_LCD, FALSE);
    lcd_set_backlight(ADD_LCD, TRUE);
    lcd_write_line(ADD_LCD, 1, msg1);
    lcd_write_line(ADD_LCD, 2, msg2);
    lcd_write_line(ADD_LCD, 3, msg3);
    lcd_write_line(ADD_LCD, 4, msg4);
}

/********************************************************************
 * Function:        void USBCB_SOF_Handler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
void USBCB_SOF_Handler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        void USBCBErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
void USBCBErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

    // Typically, user firmware does not need to do anything special
    // if a USB error occurs.  For example, if the host sends an OUT
    // packet to your device, but the packet gets corrupted (ex:
    // because of a bad connection, or the user unplugs the
    // USB cable during the transmission) this will typically set
    // one or more USB error interrupt flags.  Nothing specific
    // needs to be done however, since the SIE will automatically
    // send a "NAK" packet to the host.  In response to this, the
    // host will normally retry to send the packet again, and no
    // data loss occurs.  The system will typically recover
    // automatically, without the need for application firmware
    // intervention.
    
    // Nevertheless, this callback function is provided, such as
    // for debugging purposes.
}


/*******************************************************************
 * Function:        void USBCBCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 *                     firmware must process the request and respond
 *                    appropriately to fulfill the request.  Some of
 *                    the SETUP packets will be for standard
 *                    USB "chapter 9" (as in, fulfilling chapter 9 of
 *                    the official USB specifications) requests, while
 *                    others may be specific to the USB device class
 *                    that is being implemented.  For example, a HID
 *                    class device needs to be able to respond to
 *                    "GET REPORT" type of requests.  This
 *                    is not a standard USB chapter 9 request, and 
 *                    therefore not handled by usb_device.c.  Instead
 *                    this request should be handled by class specific 
 *                    firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *******************************************************************/
void USBCBCheckOtherReq(void)
{
    USBCheckHIDRequest();
}//end


/*******************************************************************
 * Function:        void USBCBStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USBCBStdSetDscHandler() callback function is
 *                    called when a SETUP, bRequest: SET_DESCRIPTOR request
 *                    arrives.  Typically SET_DESCRIPTOR requests are
 *                    not used in most applications, and it is
 *                    optional to support this type of request.
 *
 * Note:            None
 *******************************************************************/
void USBCBStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/*******************************************************************
 * Function:        void USBCBInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 *                     SET_CONFIGURATION (wValue not = 0) request.  This 
 *                    callback function should initialize the endpoints 
 *                    for the device's usage according to the current 
 *                    configuration.
 *
 * Note:            None
 *******************************************************************/
void USBCBInitEP(void)
{
    //enable the HID endpoint
    USBEnableEndpoint(HID_EP,USB_IN_ENABLED|USB_OUT_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    //Re-arm the OUT endpoint for the next packet
    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer,64);
}

/********************************************************************
 * Function:        void USBCBSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 *                     peripheral devices to wake up a host PC (such
 *                    as if it is in a low power suspend to RAM state).
 *                    This can be a very useful feature in some
 *                    USB applications, such as an Infrared remote
 *                    control    receiver.  If a user presses the "power"
 *                    button on a remote control, it is nice that the
 *                    IR receiver can detect this signalling, and then
 *                    send a USB "command" to the PC to wake up.
 *                    
 *                    The USBCBSendResume() "callback" function is used
 *                    to send this special USB signalling which wakes 
 *                    up the PC.  This function may be called by
 *                    application firmware to wake up the PC.  This
 *                    function should only be called when:
 *                    
 *                    1.  The USB driver used on the host PC supports
 *                        the remote wakeup capability.
 *                    2.  The USB configuration descriptor indicates
 *                        the device is remote wakeup capable in the
 *                        bmAttributes field.
 *                    3.  The USB host PC is currently sleeping,
 *                        and has previously sent your device a SET 
 *                        FEATURE setup packet which "armed" the
 *                        remote wakeup capability.   
 *
 *                    This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            Interrupt vs. Polling
 *                  -Primary clock
 *                  -Secondary clock ***** MAKE NOTES ABOUT THIS *******
 *                   > Can switch to primary first by calling USBCBWakeFromSuspend()
 
 *                  The modifiable section in this routine should be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of 1-13 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at lest 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
void USBCBSendResume(void)
{
    static WORD delay_count;
    
    USBResumeControl = 1;                // Start RESUME signaling
    
    delay_count = 1800U;                // Set RESUME line for 1-13 ms
    do
    {
        delay_count--;
    }while(delay_count);
    USBResumeControl = 0;
}


/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, void *pdata, WORD size)
{
    switch(event)
    {
        case EVENT_CONFIGURED: 
            USBCBInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            USBCBStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            USBCBCheckOtherReq();
            break;
        case EVENT_SOF:
            USBCB_SOF_Handler();
            break;
        case EVENT_SUSPEND:
            USBCBSuspend();
            break;
        case EVENT_RESUME:
            USBCBWakeFromSuspend();
            break;
        case EVENT_BUS_ERROR:
            USBCBErrorHandler();
            break;
        case EVENT_TRANSFER:
            Nop();
            break;
        default:
            break;
    }      
    return TRUE; 
}

/**
 * ResetSource() permet de connaitre l'origine du dernier reset tout en réinitialisant correctement
 * les registres pour le prochain reset. Si elle n'est pas appelée à chaque démarrage du PIC, les
 * registres ne seront pas correctement réinitialisées et l'origine du reset renvoyée par cette
 * fonction pourra être erronée.
 *
 * @return        source        source du dernier RESET, peut valoir une
 *                            des constantes RESET_*.
*/
char ResetSource(void) {
    char src;

    if (!RCONbits.POR)
        // Power-on Reset
        src = RESET_SOURCE_POR;
    else if (!RCONbits.RI)
        // RESET Instruction
        src = RESET_SOURCE_RI;
    else if (!RCONbits.BOR) // && RCONbits.POR
        // Brown-out Reset
        src = RESET_SOURCE_BOR;
    else if (!RCONbits.TO)
        // Watchdog Time-out Reset
        src = RESET_SOURCE_WDT;
    else if (STKPTRbits.STKFUL)
        // Stack Full Reset (STVREN = 1)
        src = RESET_SOURCE_STKFUL;
    else if (STKPTRbits.STKUNF)
        // Stack Underflow Reset (STVREN = 1)
        src = RESET_SOURCE_STKUNF;
    else
        // Master Clear Reset
        src = RESET_SOURCE_MCLR;

    // Réinitialisation de tous les bits de status
    RCONbits.RI = 1;
    RCONbits.TO = 1;
    RCONbits.PD = 1;
    RCONbits.POR = 1;
    RCONbits.BOR = 1;
    STKPTRbits.STKFUL = 0;
    STKPTRbits.STKUNF = 0;

    return src;
}

/** EOF main.c *************************************************/
#endif
