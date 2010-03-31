/**
 * @file rangefinder.c
 * Gère la télémétrie.
*/

#ifndef RANGEFINDER_C
#define RANGEFINDER_C

#include "rangefinder.h"

volatile char State = 0;
volatile char Rangefinder;
volatile long ElapsedTime = 0;
volatile long CurrentMeas;
volatile long MeasuredDist[8] = {0};        // en mm
volatile unsigned char CalibrationData[64];

#ifdef EN_DYN_CALIBRATION
volatile BOOL EnDynCalibration = TRUE;
#else
volatile BOOL EnDynCalibration = FALSE;
#endif

volatile char MeasDiff[8] = {0};
volatile char CalibrationStatus = 0;

/**
 * Fonction d'interruption pour le télémètre.
 * Cette fonction ne doit être appelée que dans une interruption.
*/
void interruptRangefinder(void) {
    static char refVoltage;
    WORD_VAL delayPulse;

    switch (State) {
        case 1:
            refVoltage = START_THRESHOLD;

            // Première période d'attente
            ElapsedTime = 0;                        // Mise à 0 du temps de comptage
            WriteTimer1(65535 - (unsigned int) (CYCLE_FREQ * RCPT_TIMER_PER / 1e6));

            PIE2bits.CMIE = 1;                        // Activation des interruptions du comparateur
            State = 2;
        break;

        case 2:
            ElapsedTime++;

#ifdef EN_DYN_THRESHOLD
            if (refVoltage > THRESHOLD_STEP)
                refVoltage-= THRESHOLD_STEP;

            if (refVoltage >= MIN_THRESHOLD)
                CVRCON = (CVRCON & 0b11110000) | (refVoltage & 0b1111);
#endif

            if (ElapsedTime >= (long) (RCPT_MAX_DELAY / RCPT_TIMER_PER)) {
                CurrentMeas = (long) (SOUND_SPEED / 2.0 * (ElapsedTime * RCPT_TIMER_PER) / 1000.0) + MEAS_OFFSET;
                PIE2bits.CMIE = 0;                        // Désactivation des interruptions du comparateur
                RCPT_EN = 0;                            // Désactivation du circuit de réception

                if (EnDynCalibration) {
                    // On diminue le seuil bas et on recommence !
                    if (MIN_THRESHOLD > 1) {
                        CalibrationData[8*Rangefinder + 1] = MIN_THRESHOLD-1;
                        State = 0;
                        return;
                    }
                }

                // Délais avant de passer au télémètre suivant
                WriteTimer1(65535 - (unsigned int) (CYCLE_FREQ * DELAY_RCPT / 1e6));
/*
                if (CalibrationStatus == 0) {
                    if (CurrentMeas - MeasuredDist[Rangefinder] > 100 && MeasDiff[Rangefinder] < 2)
                        MeasDiff[Rangefinder]++;
                    else {
                        MeasDiff[Rangefinder] = 0;
                        MeasuredDist[Rangefinder] = CurrentMeas;
                    }
                }
                else
*/
                    MeasuredDist[Rangefinder] = CurrentMeas;

                State = 3;
            }
            else
                WriteTimer1(65535 - (unsigned int) (CYCLE_FREQ * RCPT_TIMER_PER / 1e6));
        break;

        case 3:
            PIE1bits.TMR1IE = 0;                    // Désactivation des interruption par dépassement du Timer (Timer overflow)
            State = 4;
        break;

        default:
        break;
    }
}

/**
 * Fonction d'interruption pour le télémètre.
 * Cette fonction ne doit être appelée que dans une interruption.
*/
void interruptRangefinder2(void) {
    unsigned int time = ReadTimer1();        // On lit tout de suite le timer1 pour éviter les erreurs
                                            // et un overflow dans le cas limite !

    if (CMCONbits.C1OUT == 1 && State == 2) {
        time-= (65535 - (unsigned int) (CYCLE_FREQ * RCPT_TIMER_PER / 1e6));

        CurrentMeas = (long) (SOUND_SPEED / 2.0 * (ElapsedTime * RCPT_TIMER_PER) / 1000.0) + MEAS_OFFSET;
        CurrentMeas+= (long) (SOUND_SPEED / 2.0 * (time / CYCLE_FREQ * 1000.0));
        PIE2bits.CMIE = 0;                      // Désactivation des interruptions du comparateur
        RCPT_EN = 0;                            // Désactivation du circuit de réception

        if (EnDynCalibration) {
            // On augmente le seuil bas si possible
            if (MIN_THRESHOLD < START_THRESHOLD)
                CalibrationData[8*Rangefinder + 1] = MIN_THRESHOLD+1;
    
            if (CurrentMeas <= MEAS_OFFSET) {
                if (START_THRESHOLD < 15) {
                    // On augmente le seuil haut et on recommence !
                    CalibrationData[8*Rangefinder] = START_THRESHOLD+1;
                    State = 0;
                    return;
                }
            }
            else {
                if (START_THRESHOLD > 1) {
                    // On diminue le seuil haut
                    CalibrationData[8*Rangefinder] = START_THRESHOLD-1;
                }
            }
        }

        // Délais avant de passer au télémètre suivant
        WriteTimer1(65535 - (unsigned int) (CYCLE_FREQ * DELAY_RCPT / 1e6));
/*
        if (CalibrationStatus == 0) {
            if (CurrentMeas - MeasuredDist[Rangefinder] > 100 && MeasDiff[Rangefinder] < 2)
                MeasDiff[Rangefinder]++;
            else {
                MeasDiff[Rangefinder] = 0;
                MeasuredDist[Rangefinder] = CurrentMeas;
            }
        }
        else
*/
            MeasuredDist[Rangefinder] = CurrentMeas;

        State = 3;
    }
}

/**
 * Mesure une distance sur un télémètre.
 *
 * @param        addr        adresse du télémètre (de 0 à 7)
 * @return       status      renvoie #TRUE si la mesure est terminée et #FALSE sinon
*/
BOOL measureDistance(char addr) {
    switch (State) {
        case 0:
            // Sélection du télémètre et émission de la trame
            Rangefinder = addr;
            selectRangefinder(addr);            // Sélection du télémètre

            if (CalibrationStatus < 2)
                sendPulse();                        // Envoie de l'impulsion

            RCPT_EN = 1;                            // Activation du circuit de réception

            // Réglage du seuil de réception initial
            CVRCON = (CVRCON & 0b11110000) | (START_THRESHOLD & 0b1111);

            // Délais avant la prise en compte du signal à la réception
            WriteTimer1(65535 - (unsigned int) (CYCLE_FREQ * DELAY_PULSE / 1e6));
            PIE1bits.TMR1IE = 1;    // Autorise les interruption par dépassement du Timer (Timer overflow)
            State = 1;
        break;

        case 1:
            // Activation de la réception
        case 2:
            // Attente réception
        case 3:
            // Réception ou délais de réception dépassé
        break;

        case 4:
            // Mesure télémètre suivant
            State = 0;
            return TRUE;
        break;

        default:
            State = 0;
    }

    return FALSE;
}

/**
 * Sélection d'un télémètre.
 * Cette fonction permet de sélectionner un télémètre pour l'émission et la réception.
 *
 * @param        addr        adresse du télémètre (de 0 à 7)
*/
void selectRangefinder(char addr) {
    ADDR0 = addr & 1;
    ADDR1 = (addr >> 1) & 1;
    ADDR2 = (addr >> 2) & 1;
}

/**
 * Envoie une impulsion ultrason.
 * @note Cette fonction bloque le programme durant l'envoie de l'impulsion.
 * Cela est nécessaire pour que la fréquence de la trame émise soit à peu prêt maitrisée.
*/
void sendPulse(void) {
    unsigned char i;

    EM_EN = 1;

_asm
    movlw MAX_PULSE
    movwf i, 0

    loop:
        dcfsnz i, 1, 0
        goto done
_endasm;
        PULSE = 1;
_asm
        nop nop nop nop nop nop nop nop nop nop            // 10 NOP par ligne
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop                // 150 cycles - 1 pour "bsf PORTD, 5, 0"
_endasm;
        PULSE = 0;
_asm
        nop nop nop nop nop nop nop nop nop nop            // 10 NOP par ligne
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop nop nop nop
        nop nop nop nop nop nop nop                        // 150 cycles - 3 pour "bcf PORTD, 5, 0", "bra loop" et "dcfsnz i, 1, 0"
        bra loop

    done:
_endasm;

/*
    for (i = 0; i < MAX_PULSE; i++) {
        PULSE = 1;
        Delay10TCYx((CLOCK_FREQ / (PULSE_FREQ * 8.0)) / 10.0);          // attends 150 cycles à 40 KHz

        PULSE = 0;
        Delay10TCYx((CLOCK_FREQ / (PULSE_FREQ * 8.0)) / 10.0);          // attends 150 cycles à 40 KHz
    }
*/
    EM_EN = 0;
}

/**
 * Donne la distance mesurée pour un télémètre.
 *
 * @param        addr        adresse du télémètre (de 0 à 7)
 * @return       dist        distance mesurée (en mm)
*/
long getMeasuredDist(char addr) {
    return MeasuredDist[addr];
}

/**
 * Fonction de calibration semi-automatique d'un télémètre.
 *
 * @param        addr        adresse du télémètre (de 0 à 7)
 * @return       status      état de la calibration, peut valoir : @n
 *                            #CAL_ERROR            Erreur survenue durant la calibration (calibration impossible) @n
 *                            #CAL_CONTINUE        Demande de réappeler cette fonction pour poursuivre la calibration @n
 *                            #CAL_DONE            Calibration terminée @n
 *                            #CAL_PLACE_INF        Demande de placer le capteur loin de tout obstacle avant de réappeler cette fonction @n
 *                            #CAL_PLACE_30        Demande de placer le capteur à 30 cm d'un obstacle avant de réappeler cette fonction @n
 *                            #CAL_PLACE_100        Demande de placer le capteur à 1 m d'un obstacle avant de réappeler cette fonction
*/
BYTE calibrate(char addr, BOOL skipMeas) {
    static char state = 0;
    static char measure = 0;
    static long meas[5];
    static long meas30, meas100;
    static char dynCal;

    WORD_VAL delayPulse, slopeWord, originWord;
    BYTE threshold;
    double slope, origin;

    switch (state) {
        case 0:
            // 0. Initialisation
            CalibrationStatus = 1;

            dynCal = EnDynCalibration;
            EnDynCalibration = 0;

            // On ne met pas les seuils aux max. pour se laisser une marge de manoeuvre par la suite
            CalibrationData[8*addr] = 12;                            // On commence par régler le seuil haut assez haut
            CalibrationData[8*addr+1] = 12;                            // Idem pour le seuil bas (= pas de calibration dynamique)

            delayPulse.Val = 20;                                    // Pour commencer, on attend un temps minimal
            CalibrationData[8*addr+2] = delayPulse.byte.HB;
            CalibrationData[8*addr+3] = delayPulse.byte.LB;

            slopeWord.Val = 340;                                    // Pente initiale = vitesse du son
            CalibrationData[8*addr+4] = slopeWord.byte.HB;
            CalibrationData[8*addr+5] = slopeWord.byte.LB;

            originWord.Val = 0;                                        // Offset initial nul
            CalibrationData[8*addr+6] = originWord.byte.HB;
            CalibrationData[8*addr+7] = originWord.byte.LB;

            state = 1;
            return CAL_PLACE_INF;
        break;

        case 1:
            // 1. Réglage du temps d'attente juste après l'émission
            // Objectif : le diminuer au maximum pour pouvoir faire des mesures au plus prêt du capteur.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                delayPulse.byte.HB = CalibrationData[8*addr + 2];
                delayPulse.byte.LB = CalibrationData[8*addr + 3];

                if (meas[0] < 3000 || meas[1] < 3000 || meas[2] < 3000 || meas[3] < 3000 || meas[4] < 3000) {
                    // Pas OK
                    if (delayPulse.Val >= 2000) {        // <=> distance de 34 cm
                        // On a déjà beaucoup attendu et pourtant il semble qu'il y ait encore du signal sur le transducteur.
                        // => Peut-être faut-il augmenter le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        delayPulse.Val+= 20;
                        CalibrationData[8*addr+2] = delayPulse.byte.HB;
                        CalibrationData[8*addr+3] = delayPulse.byte.LB;
                    }
                }
                else {
                    // OK, mais on ajoute quand même une bonne marge de sécurité
                    delayPulse.Val+= 40;
                    CalibrationData[8*addr+2] = delayPulse.byte.HB;
                    CalibrationData[8*addr+3] = delayPulse.byte.LB;

                    CalibrationData[8*addr] = 15;                            // On met le seuil haut au maximum
                    CalibrationData[8*addr+1] = 15;                            // Idem pour le seuil bas (= pas de calibration dynamique)
                    state = 2;
                }
            }
        break;

        case 2:
            // 2. Réglage du seuil haut
            // Objectif : le diminuer au maximum pour la meilleure sensibilité.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                threshold = CalibrationData[8*addr];

                if (meas[0] < 3000 || meas[1] < 3000 || meas[2] < 3000 || meas[3] < 3000 || meas[4] < 3000) {
                    if (threshold >= 15) {
                        // Le seuil max. est atteint et on a encore un faux positif, on ne peut rien faire.
                        // => il faudrait augmenter manuellement le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        // On a trop baissé le seuil
                        threshold+= 3;            // On le remonte, avec une marge de sécurité

                        if (threshold > 15)
                            threshold = 15;

                        CalibrationData[8*addr] = threshold;
                    }
                }
                else if (threshold > 1) {
                    // On peut encore baisser le seuil
                    threshold--;
                    CalibrationData[8*addr] = threshold;
                    return CAL_CONTINUE;
                }

                state = 3;                // Passe à l'étape suivante
            }
        break;

        case 3:
            // 3. Réglage du seuil bas
            // Objectif : le diminuer au maximum pour la meilleure sensibilité.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                threshold = CalibrationData[8*addr+1];

                if (meas[0] < 3000 || meas[1] < 3000 || meas[2] < 3000 || meas[3] < 3000 || meas[4] < 3000) {
                    if (threshold >= 15) {
                        // Le seuil max. est atteint et on a encore un faux positif, on ne peut rien faire.
                        // => il faudrait augmenter manuellement le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        // On a trop baissé le seuil
                        threshold+= 3;            // On le remonte, avec une marge de sécurité

                        if (threshold > 15)
                            threshold = 15;

                        CalibrationData[8*addr+1] = threshold;
                    }
                }
                else if (threshold > 1) {
                    // On peut encore baisser le seuil
                    threshold--;
                    CalibrationData[8*addr+1] = threshold;
                    return CAL_CONTINUE;
                }

                if (skipMeas) {
                    // Si on a pas envie de faire des mesures précises, on évalue au mieux l'offset et on termine la calibration
                    delayPulse.byte.HB = CalibrationData[8*addr + 2];
                    delayPulse.byte.LB = CalibrationData[8*addr + 3];

                    originWord.Val = (WORD) (340.0 / 2.0 * (1e6 * MAX_PULSE / PULSE_FREQ + delayPulse.Val) / 1000.0);
                    CalibrationData[8*addr+6] = originWord.byte.HB;
                    CalibrationData[8*addr+7] = originWord.byte.LB;

                    state = 6;
                }
                else {
                    state = 4;
                    return CAL_PLACE_30;
                }
            }
        break;

        case 4:
            // 4. Calibration des mesures : mesure de près (30 cm)
            if (measureDistance(addr)) {
                meas30 = getMeasuredDist(addr);

                state = 5;
                return CAL_PLACE_100;
            }
        break;

        case 5:
            // 5. Calibration des mesures : mesure de loin (1 m)
            if (measureDistance(addr)) {
                meas100 = getMeasuredDist(addr);

                if (meas30 >= meas100) {
                    // La mesure à 30 cm est identique ou plus grande que celle à 1 m, il y a vraissemblablement un problème !
                    CalibrationStatus = 0;

                    state = 0;
                    EnDynCalibration = dynCal;
                    return CAL_ERROR;
                }
                else {
                    // Nouvelles pente et origine
                    slope = (1000.0 - 300.0) / ((meas100 - meas30) / 340.0);
                    origin = ((1000.0 + 300.0) - slope * ((meas100 + meas30) / 340.0)) / 2.0;

                    slopeWord.Val = (WORD) slope;
                    CalibrationData[8*addr+4] = slopeWord.byte.HB;
                    CalibrationData[8*addr+5] = slopeWord.byte.LB;

                    originWord.Val = (WORD) origin;
                    CalibrationData[8*addr+6] = originWord.byte.HB;
                    CalibrationData[8*addr+7] = originWord.byte.LB;

                    state = 6;
                }
            }
        break;

        case 6:
            WriteEEPROM(8*addr + 1, CalibrationData[8*addr]);
            WriteEEPROM(8*addr + 2, CalibrationData[8*addr+1]);
            WriteEEPROM(8*addr + 3, CalibrationData[8*addr+2]);
            WriteEEPROM(8*addr + 4, CalibrationData[8*addr+3]);
            WriteEEPROM(8*addr + 5, CalibrationData[8*addr+4]);
            WriteEEPROM(8*addr + 6, CalibrationData[8*addr+5]);
            WriteEEPROM(8*addr + 7, CalibrationData[8*addr+6]);
            WriteEEPROM(8*addr + 8, CalibrationData[8*addr+7]);

            CalibrationStatus = 0;

            state = 0;
            EnDynCalibration = dynCal;
            return CAL_DONE;
        break;

        default:
            state = 0;
    }

    return CAL_CONTINUE;
}

/**
 * Fonction de calibration automatique d'un télémètre.
 *
 * @param        addr        adresse du télémètre (de 0 à 7)
 * @return       status      état de la calibration, peut valoir : @n
 *                            #CAL_ERROR            Erreur survenue durant la calibration (calibration impossible) @n
 *                            #CAL_CONTINUE        Demande de réappeler cette fonction pour poursuivre la calibration @n
 *                            #CAL_DONE            Calibration terminée
*/
BYTE calibrateAuto(char addr) {
    static char state = 0;
    static char measure = 0;
    static long meas[5];
    static char dynCal;

    WORD_VAL delayPulse, slopeWord, originWord;
    BYTE threshold;

    switch (state) {
        case 0:
            // 0. Initialisation
            CalibrationStatus = 1;

            dynCal = EnDynCalibration;
            EnDynCalibration = 0;

            // On ne met pas les seuils aux max. pour se laisser une marge de manoeuvre par la suite
            CalibrationData[8*addr] = 12;                            // On commence par régler le seuil haut assez haut
            CalibrationData[8*addr+1] = 12;                            // Idem pour le seuil bas (= pas de calibration dynamique)

            delayPulse.Val = 200;                                    // Pour commencer, on attend un temps minimal
            CalibrationData[8*addr+2] = delayPulse.byte.HB;
            CalibrationData[8*addr+3] = delayPulse.byte.LB;

            slopeWord.Val = 340;                                    // Pente initiale = vitesse du son
            CalibrationData[8*addr+4] = slopeWord.byte.HB;
            CalibrationData[8*addr+5] = slopeWord.byte.LB;

            // Offset initial
            originWord.Val = (WORD) (340.0 / 2.0 * (1e6 * MAX_PULSE / PULSE_FREQ + delayPulse.Val) / 1000.0);
            CalibrationData[8*addr+6] = originWord.byte.HB;
            CalibrationData[8*addr+7] = originWord.byte.LB;

            state = 1;
        break;

        case 1:
            // 1. Réglage du temps d'attente juste après l'émission
            // Objectif : le diminuer au maximum pour pouvoir faire des mesures au plus prêt du capteur.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                delayPulse.byte.HB = CalibrationData[8*addr + 2];
                delayPulse.byte.LB = CalibrationData[8*addr + 3];
                originWord.Val = (WORD) (340.0 / 2.0 * (1e6 * MAX_PULSE / PULSE_FREQ + delayPulse.Val) / 1000.0);

                if (meas[0] <= originWord.Val || meas[1] <= originWord.Val || meas[2] <= originWord.Val || meas[3] <= originWord.Val || meas[4] <= originWord.Val) {
                    // Pas OK
                    if (delayPulse.Val >= 2000) {        // <=> distance de 34 cm
                        // On a déjà beaucoup attendu et pourtant il semble qu'il y ait encore du signal sur le transducteur.
                        // => Peut-être faut-il augmenter le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        delayPulse.Val+= 30;
                        CalibrationData[8*addr+2] = delayPulse.byte.HB;
                        CalibrationData[8*addr+3] = delayPulse.byte.LB;
                    }
                }
                else {
                    // OK, mais on ajoute quand même une bonne marge de sécurité
                    delayPulse.Val+= 60;
                    CalibrationData[8*addr+2] = delayPulse.byte.HB;
                    CalibrationData[8*addr+3] = delayPulse.byte.LB;

                    originWord.Val = (WORD) (340.0 / 2.0 * (1e6 * MAX_PULSE / PULSE_FREQ + delayPulse.Val) / 1000.0);
                    CalibrationData[8*addr+6] = originWord.byte.HB;
                    CalibrationData[8*addr+7] = originWord.byte.LB;

                    CalibrationData[8*addr] = 15;                            // On met le seuil haut au maximum
                    CalibrationData[8*addr+1] = 15;                            // Idem pour le seuil bas (= pas de calibration dynamique)
                    state = 2;
                }
            }
        break;

        case 2:
            // 2. Réglage du seuil haut
            // Objectif : le diminuer au maximum pour la meilleure sensibilité.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                threshold = CalibrationData[8*addr];

                delayPulse.byte.HB = CalibrationData[8*addr + 2];
                delayPulse.byte.LB = CalibrationData[8*addr + 3];
                originWord.Val = (WORD) (340.0 / 2.0 * (1e6 * MAX_PULSE / PULSE_FREQ + delayPulse.Val) / 1000.0);

                if (meas[0] <= originWord.Val || meas[1] <= originWord.Val || meas[2] <= originWord.Val || meas[3] <= originWord.Val || meas[4] <= originWord.Val) {
                    if (threshold >= 15) {
                        // Le seuil max. est atteint et on a encore un faux positif, on ne peut rien faire.
                        // => il faudrait augmenter manuellement le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        // On a trop baissé le seuil
                        threshold+= 2;            // On le remonte, avec une marge de sécurité

                        if (threshold > 15)
                            threshold = 15;

                        CalibrationData[8*addr] = threshold;
                    }
                }
                else if (threshold > 1) {
                    // On peut encore baisser le seuil
                    threshold--;
                    CalibrationData[8*addr] = threshold;
                    return CAL_CONTINUE;
                }

                state = 3;                // Passe à l'étape suivante
            }
        break;

        case 3:
            // 3. Réglage du seuil bas
            // Objectif : le diminuer au maximum pour la meilleure sensibilité.
            // Condition d'arrêt : détection erronée d'un obstacle proche.
            CalibrationStatus = 2;

            if (measureDistance(addr)) {
                meas[measure] = getMeasuredDist(addr);
                measure++;
            }

            if (measure >= 5) {
                measure = 0;
                threshold = CalibrationData[8*addr+1];

                if (meas[0] < 3000 || meas[1] < 3000 || meas[2] < 3000 || meas[3] < 3000 || meas[4] < 3000) {
                    if (threshold >= 15) {
                        // Le seuil max. est atteint et on a encore un faux positif, on ne peut rien faire.
                        // => il faudrait augmenter manuellement le seuil via le potentiomètre sur la plaquette.
                        CalibrationStatus = 0;

                        state = 0;
                        EnDynCalibration = dynCal;
                        return CAL_ERROR;
                    }
                    else {
                        // On a trop baissé le seuil
                        threshold+= 2;            // On le remonte, avec une marge de sécurité

                        if (threshold > 15)
                            threshold = 15;

                        CalibrationData[8*addr+1] = threshold;
                    }
                }
                else if (threshold > 1) {
                    // On peut encore baisser le seuil
                    threshold--;
                    CalibrationData[8*addr+1] = threshold;
                    return CAL_CONTINUE;
                }

                state = 4;
            }
        break;

        case 4:
            CalibrationStatus = 0;

            state = 0;
            EnDynCalibration = dynCal;
            return CAL_DONE;
        break;

        default:
            state = 0;
    }

    return CAL_CONTINUE;
}

/**
 * This Quickselect routine is based on the algorithm described in
 * "Numerical recipes in C", Second Edition,
 * Cambridge University Press, 1992, Section 8.5, ISBN 0-521-43108-5
 * This code by Nicolas Devillard - 1998. Public domain.
*/
#define ELEM_SWAP(a,b) { register long t=(a);(a)=(b);(b)=t; }
long quickSelect(long arr[], int n) {
    int low, high ;
    int median;
    int middle, ll, hh;
    low = 0 ; high = n-1 ; median = (low + high) / 2;

    for (;;) {
        if (high <= low) /* One element only */
            return arr[median] ;

        if (high == low + 1) { /* Two elements only */
            if (arr[low] > arr[high])
                ELEM_SWAP(arr[low], arr[high]) ;

            return arr[median] ;
        }

        /* Find median of low, middle and high items; swap into position low */
        middle = (low + high) / 2;
        if (arr[middle] > arr[high]) ELEM_SWAP(arr[middle], arr[high]) ;
        if (arr[low] > arr[high]) ELEM_SWAP(arr[low], arr[high]) ;
        if (arr[middle] > arr[low]) ELEM_SWAP(arr[middle], arr[low]) ;

        /* Swap low item (now in position middle) into position (low+1) */
        ELEM_SWAP(arr[middle], arr[low+1]) ;

        /* Nibble from each end towards middle, swapping items when stuck */
        ll = low + 1;
        hh = high;

        for (;;) {
            do ll++; while (arr[low] > arr[ll]) ;
            do hh--; while (arr[hh] > arr[low]) ;

            if (hh < ll)
                break;

            ELEM_SWAP(arr[ll], arr[hh]) ;
        }

        /* Swap middle item (in position low) back into correct position */
        ELEM_SWAP(arr[low], arr[hh]) ;

        /* Re-set active partition */
        if (hh <= median)
            low = ll;

        if (hh >= median)
            high = hh - 1;
    }
}
#undef ELEM_SWAP

#endif
