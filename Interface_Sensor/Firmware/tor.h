/**
 * @file tor.h
 * Gère les capteurs de contact TOR.
*/

#ifndef TOR_H
#define TOR_H

#include "Compiler.h"
#include "HardwareProfile.h"

// Entrées / sorties
#define TOR1                PORTAbits.RA0        ///< Capteur TOR N°1
#define TOR2                PORTAbits.RA1        ///< Capteur TOR N°2
#define TOR3                PORTAbits.RA2        ///< Capteur TOR N°3
#define TOR4                PORTAbits.RA3        ///< Capteur TOR N°4
#define TOR5                PORTAbits.RA4        ///< Capteur TOR N°5
#define TOR6                PORTAbits.RA5        ///< Capteur TOR N°6
#define TOR7                PORTEbits.RE0        ///< Capteur TOR N°7
#define TOR8                PORTEbits.RE1        ///< Capteur TOR N°8
#define TOR9                PORTEbits.RE2        ///< Capteur TOR N°9
#define TOR10               PORTDbits.RD0        ///< Capteur TOR N°10
#define TOR11               PORTDbits.RD1        ///< Capteur TOR N°11
#define TOR12               PORTDbits.RD2        ///< Capteur TOR N°12
#define TOR13               PORTDbits.RD4        ///< Capteur TOR N°13
#define TOR14               PORTDbits.RD5        ///< Capteur TOR N°14
#define TOR15               PORTDbits.RD6        ///< Capteur TOR N°15
#define TOR16               PORTDbits.RD7        ///< Capteur TOR N°16

#endif
