/**
 * @file tor.h
 * Gère les capteurs de contact TOR.
*/

#ifndef TOR_H
#define TOR_H

#include "Compiler.h"
#include "HardwareProfile.h"

// Entrées / sorties
#define TOR1                PORTCbits.RC7        ///< Capteur TOR N°1
#define TOR2                PORTDbits.RD4        ///< Capteur TOR N°2
#define TOR3                PORTDbits.RD1        ///< Capteur TOR N°3
#define TOR4                PORTDbits.RD0        ///< Capteur TOR N°4
#define TOR5                PORTCbits.RC2        ///< Capteur TOR N°5
#define TOR6                PORTCbits.RC1        ///< Capteur TOR N°6
#define TOR7                PORTCbits.RC0        ///< Capteur TOR N°7
#define TOR8                PORTEbits.RE2        ///< Capteur TOR N°8
#define TOR9                PORTEbits.RE1        ///< Capteur TOR N°9
#define TOR10               PORTEbits.RE0        ///< Capteur TOR N°10
#define TOR11               PORTAbits.RA5        ///< Capteur TOR N°11
#define TOR12               PORTAbits.RA4        ///< Capteur TOR N°12
#define TOR13               PORTBbits.RB4        ///< Capteur TOR N°13
#define TOR14               PORTBbits.RB5        ///< Capteur TOR N°14
#define TOR15               PORTBbits.RB6        ///< Capteur TOR N°15
#define TOR16               PORTBbits.RB7        ///< Capteur TOR N°16

#endif
