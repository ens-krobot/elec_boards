/**
 * @file mcc.c
*/

#ifndef MCC_C
#define MCC_C

#include "mcc.h"

volatile long glbEnc1Pos = 0;
volatile long glbEnc2Pos = 0;

volatile unsigned long durationMotor1 = 0;
volatile unsigned long durationMotor2 = 0;

void initMCC(char withEncoder) {
    /* Initialisation des PINs */
    #ifndef KROBOT_2010
        TRISA&= 0b11111110;        /* M1_SENS */
        TRISB&= 0b11000011;        /* M1_EN, M2_PWM, M2_SENS, M2_EN */
        TRISC&= 0b11111011;        /* M1_PWM */
    #else
        TRISB&= 0b11001011;        /* M1_EN, M1_INB, M1_INA */
        TRISC&= 0b11111011;        /* M1_PWM */
    #endif

    /* Configuration du timer2 */
    OpenTimer2(TIMER_INT_OFF   /* pas d'interruption sur le timer2 */
         & T2_PS_1_16          /* incrémente le compteur tous les 16 cycles (1:16) */
         & T2_POST_1_1         /* postscale (1:1) */
    );

    // Configuration du timer0
    if (withEncoder & MOTOR_RIGHT) {
        OpenTimer0(TIMER_INT_ON        // active le timer0
             & T0_16BIT                // compte sur 16 bits
             & T0_SOURCE_EXT           // utilise le signal de l'encodeur 1 comme horloge
             & T0_EDGE_RISE            // incrémentation sur front montant
             & T0_PS_1_1               // incrémente le compteur à chaque cycle (1:1)
        );

        WriteTimer0(0);
    }
    else {
        OpenTimer0(TIMER_INT_ON        // active le timer0
             & T0_16BIT                // compte sur 16 bits
             & T0_SOURCE_INT           // utilise l'horloge interne
             & T0_EDGE_RISE            // incrémentation sur front montant
             & T0_PS_1_8               // incrémente le compteur tout les 8 cycles (1:8)
        );
    }

    #ifndef KROBOT_2010
        INTCON2bits.TMR0IP = 0;  // Low priority interrupt for timer0

        // Configuration du timer1
        if (withEncoder & MOTOR_LEFT) {
            OpenTimer1(TIMER_INT_ON        // active le timer1
                 & T1_16BIT_RW             // compte sur 16 bits
                 & T1_SOURCE_EXT           // utilise le signal de l'encodeur 2 comme horloge
                 & T1_PS_1_1               // incrémente le compteur à chaque cycle (1:1)
                 & T1_OSC1EN_OFF           // pas d'oscillateur sur le timer1
                 & T1_SYNC_EXT_OFF         // ne pas se synchroniser sur une horloge externe
            );
    
            WriteTimer1(0);
        }
        else {
            OpenTimer1(TIMER_INT_ON        // active le timer
                 & T1_16BIT_RW             // compte sur 16 bits 
                 & T1_SOURCE_INT           // utilise l'horloge interne
                 & T1_PS_1_8               // incrémente le compteur tout les 8 cycles (1:8)
                 & T1_OSC1EN_OFF           // pas d'oscillateur sur le timer1
                 & T1_SYNC_EXT_OFF         // ne pas se synchroniser sur une horloge externe
            );
        }
    #endif

    /* Initialisation de l'état des PINs */
    #ifndef KROBOT_2010
        M1_EN = 0;
        M2_EN = 0;
        M1_PWM = 0;
        M2_PWM = 0;
        M1_SENS = 0;
        M2_SENS = 0;
    #else
        M1_EN = 0;
        M1_PWM = 0;
        M1_INA = 0;
        M1_INB = 0;
    #endif
}

void interruptMotor1() {
    WriteTimer0(5536);     // 2^16 - 60 000 : la prochaine interruption a lieu dans 60 000 cycles * 8 = 40 ms

    if (durationMotor1 > 0) {
        durationMotor1--;

        if (durationMotor1 == 0) {
            //ClosePWM1();
            CCP1CON = 0;
            M1_PWM = 0;

            #ifdef KROBOT_2010
                M1_INA = 0;
                M1_INB = 0;
            #endif
        }
    }
}

#ifndef KROBOT_2010
    void interruptMotor2() {
        WriteTimer1(5536);     // 2^16 - 60 000 : la prochaine interruption a lieu dans 60 000 cycles * 8 = 40 ms
    
        if (durationMotor2 > 0) {
            durationMotor2--;
    
            if (durationMotor2 == 0) {
                //ClosePWM2();
                CCP2CON = 0;
                M2_PWM = 0;
            }
        }
    }
#endif

/**
 * Active un ou plusieurs axe(s) moteur(s).
 * Cette fonction permet d'activer un axe moteur en vue d'une commande.
 * L'activation d'un moteur rend la carte de puissance correspondante réceptive au
 * signal PWM.
 *
 * @param        axis        l'axe moteur à activer, peut valoir : @n
 *                             #MOTOR_RIGHT    le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void enableMotor(char axis) {
    #ifndef KROBOT_2010
        if (axis & MOTOR_RIGHT)
            M1_EN = 1;
    
        if (axis & MOTOR_LEFT)
            M2_EN = 1;
    #else
        M1_EN = 1;
    #endif
}

/**
 * Désactive un ou plusieurs axe(s) moteur(s).
 * Cette fonction désactive un axe moteur et le rend inerte à toute commande.
 * Désactiver un moteur revient à désactiver la carte de puissance associée.
 *
 * @param        axis        l'axe moteur à activer, peut valoir : @n
 *                             #MOTOR_RIGHT    le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void disableMotor(char axis) {
    #ifndef KROBOT_2010
        if (axis & MOTOR_RIGHT)
            M1_EN = 0;
    
        if (axis & MOTOR_LEFT)
            M2_EN = 0;
    #else
        M1_EN = 0;
    #endif
}

/**
 * Fait avancer un moteur ou les deux moteurs.
 *
 * @param        axis        l'axe moteur à activer, peut valoir : @n
 *                             #MOTOR_RIGHT    le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * @param       sens        sens de rotation des moteurs.
 * @param       speed       vitesse, en pourcentage (0 = vitesse nulle, 255 = 100%, vitesse max.)
 * @param       duration    durée d'activation, en ms (0 = rotation continue)
*/
void move(char axis, char sens, BYTE speed, unsigned long duration) {
    #ifndef KROBOT_2010
        if (axis & MOTOR_RIGHT) {
            durationMotor1 = duration / 40;
            WriteTimer0(5536);
    
            M1_SENS    = sens;
    
            // PWM period =[(period ) + 1] x 4 x TOSC x TMR2 prescaler
            // TMR2 prescaler = 1
            OpenPWM1((char) ((float) CLOCK_FREQ / (4.0 * (float) PWM_FREQ * 16.0) - 1.0));
        
            // PWM x Duty cycle = (DCx<9:0>) x TOSC
            SetDCPWM1((unsigned int) ((float) speed / 255.0 * (MOTOR1_VOLTAGE / H_BRIDGE_VOLTAGE) * (float) CLOCK_FREQ / (float) PWM_FREQ) / 16.0);
        }

        if (axis & MOTOR_LEFT) {
            durationMotor2 = duration / 40;
            WriteTimer1(5536);
    
            M2_SENS    = sens;
    
            // PWM period =[(period ) + 1] x 4 x TOSC x TMR2 prescaler
            // TMR2 prescaler = 1
            OpenPWM2((char) ((float) CLOCK_FREQ / (4.0 * (float) PWM_FREQ * 16.0) - 1.0));
        
            // PWM x Duty cycle = (DCx<9:0>) x TOSC
            SetDCPWM2((unsigned int) ((float) speed / 255.0 * (MOTOR1_VOLTAGE / H_BRIDGE_VOLTAGE) * (float) CLOCK_FREQ / (float) PWM_FREQ) / 16.0);
        }
    #else
        durationMotor1 = duration / 40;
        WriteTimer0(5536);

        M1_INA     = sens;
        M1_INB     = ~sens;

        // PWM period =[(period ) + 1] x 4 x TOSC x TMR2 prescaler
        // TMR2 prescaler = 1
        OpenPWM1((char) ((float) CLOCK_FREQ / (4.0 * (float) PWM_FREQ * 16.0) - 1.0));
    
        // PWM x Duty cycle = (DCx<9:0>) x TOSC
        SetDCPWM1((unsigned int) ((float) speed / 255.0 * (MOTOR1_VOLTAGE / H_BRIDGE_VOLTAGE) * (float) CLOCK_FREQ / (float) PWM_FREQ) / 16.0);
    #endif
}

BOOL checkTOR(void) {
    // M1_SENS == 0 : up (IN3 = RD7 = TOR2)
    // M1_SENS == 1 : down (IN2 = RD6 = TOR1)

    #ifndef KROBOT_2010
        if ((M1_SENS == 1 && !TOR1) || (M1_SENS == 0 && !TOR2)) {
            //ClosePWM1();
            CCP1CON = 0;
            M1_PWM = 0;
            durationMotor1 = 0;
            return TRUE;
        }
    #else
        if ((M1_INA == 1 && M1_INB == 0 && !TOR1) || (M1_INA == 0 && M1_INB == 1 && !TOR2)) {
            CCP1CON = 0;
            M1_PWM = 0;

            #ifdef MODE_INAB
                M1_INA = 0;
                M1_INB = 0;
            #endif

            durationMotor1 = 0;
            return TRUE;
        }
    #endif

    return FALSE;
}

#endif
