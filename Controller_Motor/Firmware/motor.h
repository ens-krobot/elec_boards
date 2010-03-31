/**
 * @file motor.h
 * G�re l'interface avec la carte de puissance du moteur.
*/

#ifndef MOTOR_H
#define MOTOR_H

#include <adc.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

// Param�tres g�n�raux
#define ISENS_R              0.02                    ///< Valeur de la r�sistance de mesure du courant (en ohm)
#define ISENS_GAIN            4.0                    ///< Gain du circuit de mesure (en V/V)
#define ISENS_AVR0             10                    ///< Nombre de mesure du courant pour la moyenne de 1er niveau
#define ISENS_AVR1             10                    /**< Nombre de mesure du courant pour la moyenne de 2nd niveau
    la valeur du courant qui sera renvoy�e par la fonction getIsens() sera en fait une moyenne sur
    (ISENS_AVR0 * ISENS_AVR1) �chantillons. */

// Constantes calcul�es
#define ISENS_COEF           (1000.0 / (ISENS_R * ISENS_GAIN))

// Entr�es / sorties
#define EN_MOTOR1            PORTAbits.RA2        ///< Signal ENABLE du moteur de droite (moteur 1)

#if defined(REV_1_0)
    // Board revision 1.0
    #define EN_MOTOR2            PORTAbits.RA1        ///< Signal ENABLE du moteur de gauche (moteur 2)

    #define initMotorsIO() {                                                                               \
        TRISA&= 0b11111001;                                                                                \
        PORTA&= 0b11111001;                                                                                \
    }
#elif defined(REV_1_1)
    // Board revision 1.1
    #define EN_MOTOR2            PORTBbits.RB4        ///< Signal ENABLE du moteur de gauche (moteur 2)

    #define initMotorsIO() {                                                                               \
        TRISA&= 0b11111011;                                                                                \
        PORTA&= 0b11111011;                                                                                \
        TRISB&= 0b11101111;                                                                                \
        PORTB&= 0b11101111;                                                                                \
    }
#else
    #error Unknown board revision
#endif

// Constantes programme
#define MOTOR_RIGHT            1            ///< S�lection du moteur de droite (moteur 1)
#define MOTOR_LEFT             2            ///< S�lection du moteur de gauche (moteur 2)
#define MOTOR_BOTH             3            ///< S�lection des 2 moteurs simultan�ment

#if defined(REV_1_0)
    // Board revision 1.0
    /// S�lection de la PIN AN0.
    /// @bug Sur la r�vision 1.0 de la carte, il n'est pas possible de mettre AN0 et AN3 en analogique
    /// sans mettre �galement AN1 et AN2 en analogique, alors que ces deux sorties sont pr�vues pour �tre
    /// des sorties num�riques sur cette version de la carte.
    /// Ce bug a �t� corrig� dans la r�vision 1.1.
    #define ADC_ANA            ADC_1ANA    // analog: AN0      digital: AN1->15
#elif defined(REV_1_1)
    // Board revision 1.1
    #define ADC_ANA            ADC_2ANA    // analog: AN0->1   digital: AN2->15
#else
    #error Unknown board revision
#endif

/**
 * Initialise l'interface de commande des cartes de puissance.
 * Cette commande ne doit �tre appel�e qu'une seule fois au d�but du programme.
*/
#define initMotors() {                                                                                     \
    initMotorsIO();                                                                                        \
                                                                                                           \
    OpenADC(ADC_FOSC_64         /* A/D clock source (forc�ment Fosc / 64 car PIC � 48 MHz) */              \
        & ADC_RIGHT_JUST        /* A/D result justification */                                             \
        & ADC_0_TAD,            /* A/D acquisition time select (pas la peine d'attendre avant le */        \
                                /* d�but de la conversion car l'acquisition a largement eu le */           \
                                /* temps de se faire entre chaque interruption, toutes les 1 ms) */        \
        ADC_INT_OFF             /* A/D Interrupts */                                                       \
        & ADC_REF_VDD_VSS,      /* A/D voltage configuration */                                            \
        ADC_ANA                                                                                            \
    );                                                                                                     \
}

void interruptMotor(void);
long getIsens(char axis);
void enableMotor(char axis);
void disableMotor(char axis);

#endif
