/**
 * @file eeprom.c
 * Fonctions de lecture et d'�criture avec l'EEPROM du PIC.
*/

#ifndef EEPROM_C
#define EEPROM_C

#include "eeprom.h"

/**
 * Ecrit dans l'EEPROM du PIC.
 * La taille totale de la m�moire est de 256 * 8 bits = 256 octets.
 *
 * @param        addr        emplacement de l'octet � �crire (de 0 � 255)
 * @param        data        donn�es � �crire, sur 8 bits
*/
void WriteEEPROM(BYTE addr, BYTE dat) {
    static unsigned char GIE_Status;
    EEADR = addr;                    // EEPROM memory location
    EEDATA = dat;                    // Data to be writen 
    EECON1bits.EEPGD = 0;            // Enable EEPROM write
    EECON1bits.CFGS = 0;             // Enable EEPROM write
    EECON1bits.WREN = 1;             // Enable EEPROM write
    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts
    EECON2 = 0x55;                   // Required sequence to start write cycle
    EECON2 = 0xAA;                   // Required sequence to start write cycle
    EECON1bits.WR = 1;               // Required sequence to start write cycle
    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
    while(EECON1bits.WR);            // Wait for completion of write sequence
    PIR2bits.EEIF = 0;               // Disable EEPROM write
    EECON1bits.WREN = 0;             // Disable EEPROM write
}

/**
 * Lit dans l'EEPROM du PIC.
 * La taille totale de la m�moire est de 256 * 8 bits = 256 octets.
 *
 * @param        addr        emplacement de l'octet � lire (de 0 � 255)
 * @return       data        donn�es lues, sur 8 bits
*/
BYTE ReadEEPROM(BYTE addr) {
    EEADR = addr;                    // EEPROM memory location
    EECON1bits.EEPGD = 0;            // Enable read sequence
    EECON1bits.CFGS = 0;             // Enable read sequence
    EECON1bits.RD = 1;               // Enable read sequence
    Delay10TCYx(2);                  // Delay to ensure read is completed
    return EEDATA;
}

#endif
