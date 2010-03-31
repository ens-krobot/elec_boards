/********************************************************************
 FileName:        main.c
 Dependencies:    See INCLUDES section
 Processor:       PIC18, PIC24, PIC32 USB Microcontrollers
 Hardware:        This demo is natively intended to be used on Microchip USB demo
                  boards supported by the MCHPFSUSB stack.  See release notes for
                  support matrix.  This demo can be modified for use on other hardware
                  platforms.
 Complier:        Microchip C18 (for PIC18) or C30 (for PIC24) or C32 (for PIC32)
 Company:         Microchip Technology, Inc.

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
********************************************************************/

#ifndef MAIN_C
#define MAIN_C

/** INCLUDES *******************************************************/
#include <timers.h>     // fonctions pour les timers
#include <usart.h>
#include <i2c.h>
#include <delays.h>
#include <string.h>        // absolument n�cessaire pour utiliser strcmp()

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
#include "rs232.h"

/** VARIABLES ******************************************************/
#pragma udata
BYTE old_sw1,old_sw2;

USB_HANDLE USBOutHandle = 0;
USB_HANDLE USBInHandle = 0;
BOOL blinkStatusValid = TRUE;
UINT8 UPClientSeq = 0;
char resetSource;

volatile unsigned int ms = 0;
volatile unsigned int s = 0;

extern volatile BYTE err[32];
extern volatile BYTE errno;

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

/** PRIVATE PROTOTYPES *********************************************/
void BlinkUSBStatus(void);
static void InitializeSystem(void);
void ProcessIO(void);
void UserInit(void);
void YourHighPriorityISRCode();
void YourLowPriorityISRCode();
char ResetSource(void);

#if defined(REV_2_0)
    BOOL Switch1IsPressed(void);
    BOOL Switch2IsPressed(void);
#endif

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
    
        // Interruption par le timer1
        if (PIR1bits.TMR1IF) {        
            WriteTimer1(57536); // 2^16 - 8 000 : la prochaine interruption a lieu dans 8 000 cycles = 1 ms
            ms++;
    
            if (ms >= 1000) {
                ms = 0;
                s++;
            }
    
            // En fonctionnement : la LED 3 clignote
    //        if (s > 1 && ms == 500)
    //            mLED_3_Toggle();
    
            // On r�autorise l'interruption
            PIR1bits.TMR1IF = 0;
        }

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
/*
        if (PIR1bits.RCIF) {
            RS232Receive();
            mLED_3_On();
    
            // On r�autorise l'interruption
             PIR1bits.RCIF = 0;
        }
*/

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

    // ResetSource() permet de connaitre l'origine du dernier reset tout en r�initialisant correctement
    // les registres pour le prochain reset. Si elle n'est pas appel�e � chaque d�marrage du PIC, les
    // registres ne seront pas correctement r�initialis�es et l'origine du reset renvoy�e par cette
    // fonction pourra �tre erron�e.
    resetSource = ResetSource();

    // Fail-safe to be able to enter the bootloader
    if (resetSource == RESET_SOURCE_MCLR) {
        // We check the reset source to avoid entering the bootloader mode on
        // events such as a Power-on reset followed by a Brown-out reset
        eeprom = ReadEEPROM(0x00);
        eeprom|= 0b1;
        WriteEEPROM(0x00, eeprom);
        Delay10KTCYx(0);
        eeprom^= 0b1;
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

    // Configuration du timer1
    PIE1bits.TMR1IE = 1;    // Autorise les interruption par d�passement du Timer (Timer overflow)
    OpenTimer1(TIMER_INT_ON        // active le timer1
         & T1_16BIT_RW            // compte sur 16 bits
         & T1_SOURCE_INT        // utilise l'horloge interne
         & T1_PS_1_1            // incr�mente le compteur � chaque cycle (1:1)
         & T1_OSC1EN_OFF        // pas d'oscillateur sur le timer1
         & T1_SYNC_EXT_OFF        // ne pas se synchroniser sur une horloge externe
    );
    WriteTimer1(57536); // 2^16 - 8 000 : la prochaine interruption a lieu dans 8 000 cycles = 1 ms

    // Configuration de l'USART
    OpenUSART(USART_TX_INT_OFF
        & USART_RX_INT_OFF
        & USART_ASYNCH_MODE
        & USART_EIGHT_BIT
        & USART_CONT_RX
        & USART_BRGH_HIGH,
    25);                    // FOSC/[16 (n + 1)] = 115200 bauds
//    IPR1bits.RCIP = 0; /* Make receive interrupt low priority */

    // Configuration de l'I2C
    SSPADD = 119;    // Vitesse de l'I2C = 100 KHz ( SSPADD = FOSC/[4*(SSPADD+1)] )

    // Initialisation et effacement de l'afficheur LCD
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument n�cessaire
    WriteI2C(0x00);
    WriteI2C(0);    // Les 3 write suivants servent � initialiser correctement l'afficheur
    WriteI2C(0);
    WriteI2C(0);
    WriteI2C(0x1B);
    WriteI2C(0x43);
    StopI2C();
    CloseI2C();

    /**
     * Initialisation des composants externes
    */

    //Initialize all of the LED pins
    mInitAllLEDs();
    
    #if defined(REV_2_0)
        //Initialize all of the push buttons
        mInitAllSwitches();
        old_sw1 = sw1;
        old_sw2 = sw2;
    #endif

    //initialize the variable holding the handle for the last
    // transmission
    USBOutHandle = 0;
    USBInHandle = 0;

    blinkStatusValid = TRUE;
}//end UserInit

/********************************************************************
 * Function:        void ProcessIO(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is a place holder for other user
 *                  routines. It is a mixture of both USB and
 *                  non-USB tasks.
 *
 * Note:            None
 *******************************************************************/
void ProcessIO(void)
{   
    static WORD led_count=0;
    BYTE eeprom;
    UINT i;
    UINT count;
    BOOL timeOut;

    if (DataRdyUSART()) {
        mLED_3_On();
        RS232Receive();
        led_count = 1000U;
    }
    else if (led_count > 0) {
        led_count--;

        if (led_count == 0)
            mLED_3_Off();
    }

    //Blink the LEDs according to the USB device status
    if(blinkStatusValid) {
        BlinkUSBStatus();
    }

    // User Application USB tasks
    if((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1)) return;

    if (!HIDTxHandleBusy(USBInHandle) && err[errno] > 0) {
        ToSendDataBuffer.HSEQ    = 0;                        // Num�ro s�quence PC
        ToSendDataBuffer.DSEQ    = (UPClientSeq++);            // Num�ro s�quence PIC
        ToSendDataBuffer.CMD    = CMD_ERR;                    // Type requ�te
        ToSendDataBuffer.ERR    = err[errno];                // Erreur
        USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);

        err[errno] = 0;

        if (errno > 0)
            errno--;
        else if (err[31] > 0)
            errno = 31;
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
                    ToSendDataBuffer.HSEQ    = ReceivedDataBuffer.HSEQ;    // Num�ro s�quence PC
                    ToSendDataBuffer.DSEQ    = 0;                              // Num�ro s�quence PIC
                    ToSendDataBuffer.CMD    = CMD_RESPOND;                     // Type requ�te
                    ToSendDataBuffer.ERR    = 0;                               // Erreur
    
                    switch (ReceivedDataBuffer.DATA[0]) {
                        case GET_RESET_SOURCE:
                            ToSendDataBuffer.DATA[0] = resetSource;
                        break;
    
                        case GET_BOARD_INFO:
                            ToSendDataBuffer.DATA[0] = '\0';

                            #if defined(REV_1_0)
                                strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) "Carte d'essais 1.0\nOlivier BICHLER");
                            #elif defined(REV_2_0)
                                strcatpgm2ram((char *) &ToSendDataBuffer.DATA[0], (const rom far char *) "Carte d'essais 2.0\nOlivier BICHLER");
                            #else
                                #error Unknown board revision
                            #endif
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

                            StartI2C();                             // send start bit
                            IdleI2C();                              // and wait for it to clear
                            WriteI2C(0xC0);                         // 11000000 - write command
                            WriteI2C(0);                            // read from address 0 

                            RestartI2C();
                            IdleI2C();
                            WriteI2C(0xC1);                         // 11000001 - read command

                            // R�cup�re N� de r�vision du module CMP03
                            // R�cup�re la valeur de l'angle sur un octet (0 - 255)
                            // R�cup�re l'octet de poids fort de l'angle (0 - 3599)
                            // R�cup�re l'octet de poids faible de l'angle (0 - 3599)
                            for (i = 0; i < 4; i++) {
                                timeOut = TRUE;

                                SSPCON2bits.RCEN = 1;               // start receiving

                                for (count = 0; count < 1000; count++) {
                                    if (DataRdyI2C()) {
                                        ToSendDataBuffer.DATA[i] = SSPBUF;
                                        timeOut = FALSE;
                                        break;
                                    }
                                }

                                if (timeOut) {
                                    ToSendDataBuffer.ERR = ERR_CMP03_NOT_RESPONDING;
                                    NotAckI2C();                    // not acknowledge for last byte
                                    IdleI2C();                      // wait for ack. sequence to end
                                    break;
                                }
    
                                // Il n'y a pas d'acknowledge si l'octet que l'on lit est le dernier
                                // Sinon il est n�cessaire d'en faire un, car si l'esclave en attend un
                                // et que l'on ne l'envoie pas, il peut rester plant� � l'attendre ind�finiment...
                                if (i < 3)
                                    AckI2C();                       // start acknowledge sequence
                                else
                                    NotAckI2C();                    // not acknowledge for last byte

                                IdleI2C();                          // wait for ack. sequence to end
                            }

                            StopI2C();
                            IdleI2C();
                            CloseI2C();

/*
                            OpenI2C(MASTER, SLEW_OFF);

                            SSPCON2bits.SEN = 1;                    // send start bit
                            while(SSPCON2bits.SEN);                 // and wait for it to clear
                            SSPCON2bits.ACKDT = 0;                  // acknowledge bit
                        
                            PIR1bits.SSPIF = 0;
                            SSPBUF = 0xC0;                          // 11000000 - write command
                            while(!PIR1bits.SSPIF);                 // wait for interrupt
                            PIR1bits.SSPIF = 0;                     // then clear it.
                        
                            SSPBUF = 0;                             // read from address 0 
                            while(!PIR1bits.SSPIF);                 // 
                            PIR1bits.SSPIF = 0;                     // 
                        
                            SSPCON2bits.RSEN = 1;                   // send repeated start bit
                            while(SSPCON2bits.RSEN);                // and wait for it to clear
                        
                            PIR1bits.SSPIF = 0;
                            SSPBUF = 0xC1;                          // 11000001 - read command
                            while(!PIR1bits.SSPIF);                 // wait for interrupt
                            PIR1bits.SSPIF = 0;                     // then clear it.
                                
                            SSPCON2bits.RCEN = 1;                   // start receiving
                            while(!SSPSTATbits.BF);                 // wait for data
                            ToSendDataBuffer.DATA[0] = SSPBUF;     // and get it
                            SSPCON2bits.ACKEN = 1;                  // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);               // wait for ack. sequence to end
                            
                            SSPCON2bits.RCEN = 1;                   // start receiving
                            while(!SSPSTATbits.BF);                 // wait for data
                            ToSendDataBuffer.DATA[1] = SSPBUF;   // and get it
                            SSPCON2bits.ACKEN = 1;                  // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);               // wait for ack. sequence to end
                        
                            SSPCON2bits.RCEN = 1;                   // start receiving
                            while(!SSPSTATbits.BF);                 // wait for data
                            ToSendDataBuffer.DATA[2] = SSPBUF;   // and get it
                            SSPCON2bits.ACKEN = 1;                  // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);               // wait for ack. sequence to end
                        
                            SSPCON2bits.RCEN = 1;                   // start receiving
                            while(!SSPSTATbits.BF);                 // wait for data
                            ToSendDataBuffer.DATA[3] = SSPBUF;   // and get it
                            SSPCON2bits.ACKDT = 1;                  // not acknowledge for last byte
                            SSPCON2bits.ACKEN = 1;                  // start acknowledge sequence
                            while(SSPCON2bits.ACKEN);               // wait for ack. sequence to end
                        
                            SSPCON2bits.PEN = 1;                    // send stop bit
                            while(SSPCON2bits.PEN);                 //

                            CloseI2C();
*/
                        break;

                        default:
                            ToSendDataBuffer.ERR = ERR_UNKNOWN_GET;
                    }

                    USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
                    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer, 64);        // Re-arm the OUT endpoint for the next packet
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

                    default:
                        error(ERR_UNKNOWN_SET);
                }

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer, 64);        // Re-arm the OUT endpoint for the next packet
            break;

            case CMD_SEND:
                OpenI2C(MASTER, SLEW_OFF);
                StartI2C();
                IdleI2C();        // Absolument n�cessaire
                WriteI2C(0x00);
                putsI2C(&ReceivedDataBuffer.DATA[0]);
                StopI2C();
                CloseI2C();

                USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer, 64);        // Re-arm the OUT endpoint for the next packet
            break;

            default:
                // Commande non reconnue
                if (!HIDTxHandleBusy(USBInHandle)) {
                    ToSendDataBuffer.HSEQ = ReceivedDataBuffer.HSEQ;       // Num�ro s�quence PC
                    ToSendDataBuffer.DSEQ = 0;                                 // Num�ro s�quence PIC
                    ToSendDataBuffer.CMD = CMD_RESPOND;                        // Type requ�te
                    ToSendDataBuffer.ERR = ERR_UNKNOWN_CMD;                    // Erreur

                    USBInHandle = HIDTxPacket(HID_EP, (BYTE*) &ToSendDataBuffer, 64);
                    USBOutHandle = HIDRxPacket(HID_EP,(BYTE*)&ReceivedDataBuffer, 64);        // Re-arm the OUT endpoint for the next packet
                }
        }
    }
}//end ProcessIO

#if defined(REV_2_0)

/******************************************************************************
 * Function:        BOOL Switch1IsPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - pressed, FALSE - not pressed
 *
 * Side Effects:    None
 *
 * Overview:        Indicates if the switch is pressed.  
 *
 * Note:            
 *
 *****************************************************************************/
BOOL Switch1IsPressed(void)
{
    if(sw1 != old_sw1)
    {
        old_sw1 = sw1;                  // Save new value
        if(sw1 == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}//end Switch1IsPressed

/******************************************************************************
 * Function:        BOOL Switch2IsPressed(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          TRUE - pressed, FALSE - not pressed
 *
 * Side Effects:    None
 *
 * Overview:        Indicates if the switch is pressed.  
 *
 * Note:            
 *
 *****************************************************************************/
BOOL Switch2IsPressed(void)
{
    if(sw2 != old_sw2)
    {
        old_sw2 = sw2;                  // Save new value
        if(sw2 == 0)                    // If pressed
            return TRUE;                // Was pressed
    }//end if
    return FALSE;                       // Was not pressed
}//end Switch2IsPressed

#endif


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
    
    // Turn off LCD backlight
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument n�cessaire
    WriteI2C(0x00);
    WriteI2C(0x1B);
    WriteI2C(0x62);
    StopI2C();
    CloseI2C();

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

    // Turn on LCD backlight
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument n�cessaire
    WriteI2C(0x00);
    WriteI2C(0x1B);
    WriteI2C(0x42);
    StopI2C();
    CloseI2C();
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
 * ResetSource() permet de connaitre l'origine du dernier reset tout en r�initialisant correctement
 * les registres pour le prochain reset. Si elle n'est pas appel�e � chaque d�marrage du PIC, les
 * registres ne seront pas correctement r�initialis�es et l'origine du reset renvoy�e par cette
 * fonction pourra �tre erron�e.
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

    // R�initialisation de tous les bits de status
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
