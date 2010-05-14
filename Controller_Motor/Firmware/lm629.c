/**
 * @file lm629.c
 * Gère l'interface avec les circuits intégrés LM629.
*/

#ifndef LM629_C
#define LM629_C

#include "lm629.h"

volatile BOOL trajLoaded = 0;
volatile BOOL trajEngaged = 0;
volatile BYTE trajCompleted = 0;

volatile DWORD posRight = 0;
volatile DWORD posLeft = 0;

void checkLM629Interrupt(void) {
    // Ne jamais oublier de resetter l'interruption correspondante sous peine de flooder l'USB dans le cas de la méthode LM_POLLING
    // et de ne pas détecter les interruptions suivantes dans le cas de la méthode LM_INTERRUPT !
    // Toutes les interruptions non masquées doivent être traités ici dans le cas de la méthode LM_INTERRUPT.
    BYTE statusRight = readStatus(MOTOR_RIGHT);
    BYTE statusLeft = readStatus(MOTOR_LEFT);
    char axis;

    // Bit 1: Command-Error Interrupt
    if ((statusRight & LM_COMMAND_ERROR) || (statusLeft & LM_COMMAND_ERROR)) {
        error(ERR_LM_COMMAND_ERROR);
        resetInterrupt(MOTOR_BOTH, LM_COMMAND_ERROR);
    }

    // Bit 2: Trajectory-Complete Interrupt
    axis = 0;

    if (statusRight & LM_TRAJECTORY_COMPLETE)
        axis|= MOTOR_RIGHT;

    if (statusLeft & LM_TRAJECTORY_COMPLETE)
        axis|= MOTOR_LEFT;

    if (axis > 0) {
        trajCompleted|= axis;
        resetInterrupt(axis, LM_TRAJECTORY_COMPLETE);
    }

    // Bit 3: Index-Pulse Interrupt -- Not implemented

    // Bit 4: Wrap-Around Interrupt -- Not implemented

    // Bit 5: Position-Error Interrupt
    if ((statusRight & LM_POSITION_ERROR) || (statusLeft & LM_POSITION_ERROR)) {
        error(ERR_LM_POSITION_ERROR);
        resetInterrupt(MOTOR_BOTH, LM_POSITION_ERROR);
    }

    // Bit 6: Breakpoint Interrupt
    axis = 0;

    if (statusRight & LM_BREAKPOINT)
        axis|= MOTOR_RIGHT;

    if (statusLeft & LM_BREAKPOINT)
        axis|= MOTOR_LEFT;
    
    if (axis > 0) {
        if (trajLoaded)
            start(axis);
        
        resetInterrupt(axis, LM_BREAKPOINT);
    }
}

/**
 * Change la direction du BUS de donnée.
 * Cette fonction est utilisée en interne et ne doit pas être appelée sinon.
 * 
 * @param        dir        la direction du BUS, peut valoir : @n
 *                         #DATA_OUT        transfert du PIC vers le LM @n
 *                         #DATA_IN        transfert du LM vers le PIC
*/
void dataBusDirection(char dir) {
    if (dir == DATA_OUT) {
        TRISE&= 0b11111000;
        TRISC&= 0b00111100;
        TRISD&= 0b11101111;
    }
    else if (dir == DATA_IN) {
        TRISE|= 0b00000111;
        TRISC|= 0b11000011;
        TRISD|= 0b00010000;
    }
}

/**
 * Attends que le ou les LM(s) ne soient plus occupés.
 * Cette fonction est appelée en interne dans writeCommand(), writeDataWord(), readDataWord()
 * et readStatus(). Il n'est donc pas nécessaire de l'appeler avant d'utiliser l'une de ces
 * fonctions et vous ne devriez en fait jamais devoir l'appeler.
 *
 * @param        axis    l'axe moteur correspondant au LM que l'on veut attendre, peut valoir : @n
 *                         #MOTOR_RIGHT    le moteur de droite uniquement @n
 *                         #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                         #MOTOR_BOTH        les 2 moteurs, on attends donc que les 2 LMs ne soient plus occupés
*/
void waitBusyLM(char axis) {
    char busy_bit;

    dataBusDirection(DATA_IN);
    LM_PS = PS_COMMAND;                    // attendre au moins 30 ns avant LM_READ = 0

    // La durée d'attente ne devrait pas excéder 100 us
    if (axis & MOTOR_RIGHT) {
        LM_CS1 = 0;
        LM_CS2 = 1;                        // attendre au moins 30 ns avant LM_READ = 0
        Nop();                             // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)

        do {
            LM_READ = 0;                   // attendre au moins 180 ns avant de lire le bus
            Nop(); Nop(); Nop();           // attends 3 cycles, à 48 MHz / 4 = 250 ns (> 180 ns)

            busy_bit = LM_D0;

            LM_READ = 1;                   // attendre au moins 120 ns avant LM_READ = 0
            Nop(); Nop();                  // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 120 ns)
        }
        while (busy_bit == 1);
    }

    // La durée d'attente ne devrait pas excéder 100 us
    if (axis & MOTOR_LEFT) {
        LM_CS1 = 1;
        LM_CS2 = 0;                        // attendre au moins 30 ns avant LM_READ = 0
        Nop();                             // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)

        do {
            LM_READ = 0;                   // attendre au moins 180 ns avant de lire le bus
            Nop(); Nop(); Nop();           // attends 3 cycles, à 48 MHz / 4 = 250 ns (> 180 ns)

            busy_bit = LM_D0;

            LM_READ = 1;                   // attendre au moins 120 ns avant LM_READ = 0
            Nop(); Nop();                  // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 120 ns)
        }
        while (busy_bit == 1);
    }
}

/**
 * Envoie une commande à/aux LM(s).
 * Il est possible d'envoyer une même commande aux 2 LMs simultanément.
 * 
 * @param        axis        l'axe moteur correspondant au LM à atteindre, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 *
 * @param        command        la commande à envoyer, peut valoir : @n
 *                             LM_CMD_*        une des constante commande du LM
*/
void writeCommand(char axis, BYTE command) {
    BYTE_VAL cmd = {command};
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    waitBusyLM(axis);
    dataBusDirection(DATA_OUT);

    LM_CS1 = ((axis & MOTOR_RIGHT) == 0);
    LM_CS2 = ((axis & MOTOR_LEFT) == 0);

    LM_PS = PS_COMMAND;                  // attendre au moins 30 ns avant LM_WRITE = 0
    Nop();                               // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)
    LM_WRITE = 0;                        // doit rester à 0 pendant au moins 100 ns

    LM_D0 = cmd.bits.b0;
    LM_D1 = cmd.bits.b1;
    LM_D2 = cmd.bits.b2;
    LM_D3 = cmd.bits.b3;
    LM_D4 = cmd.bits.b4;
    LM_D5 = cmd.bits.b5;
    LM_D6 = cmd.bits.b6;
    LM_D7 = cmd.bits.b7;                 // attendre au moins 50 ns avant LM_WRITE = 1

    Nop(); Nop();                        // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 100 ns + 50 ns)
    LM_WRITE = 1;                        // attendre au moins 30 ns avant LM_PS = PS_DATA + attendre au moins 120 ns avant de changer les données

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Envoie un mot au LM (un mot = 2 octets = 16 bits).
 * Il est possible d'envoyer un même mot aux 2 LMs simultanément.
 * 
 * @param        axis        l'axe moteur correspondant au LM à atteindre, peut valoir : @n
 *                             #MOTOR_RIGHT      le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 *
 * @param        word        le mot à envoyer, sur 16 bits
*/
void writeDataWord(char axis, WORD data) {
    WORD_VAL word = {data};
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    waitBusyLM(axis);
    dataBusDirection(DATA_OUT);

    LM_CS1 = ((axis & MOTOR_RIGHT) == 0);
    LM_CS2 = ((axis & MOTOR_LEFT) == 0);

    LM_PS = PS_DATA;                     // attendre au moins 30 ns avant LM_WRITE = 0
    Nop();                               // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)
    LM_WRITE = 0;                        // doit rester à 0 pendant au moins 100 ns

    // On transfert l'octet de poids fort en premier
    LM_D0 = word.bits.b8;
    LM_D1 = word.bits.b9;
    LM_D2 = word.bits.b10;
    LM_D3 = word.bits.b11;
    LM_D4 = word.bits.b12;
    LM_D5 = word.bits.b13;
    LM_D6 = word.bits.b14;
    LM_D7 = word.bits.b15;               // attendre au moins 50 ns avant LM_WRITE = 1

    Nop(); Nop();                        // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 100 ns + 50 ns)
    LM_WRITE = 1;                        // attendre au moins 120 ns avant LM_WRITE = 0
    Nop(); Nop();                        // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 120 ns)
    LM_WRITE = 0;                        // doit rester à 0 pendant au moins 100 ns

    // On transfert l'octet de poids faible en second
    LM_D0 = word.bits.b0;
    LM_D1 = word.bits.b1;
    LM_D2 = word.bits.b2;
    LM_D3 = word.bits.b3;
    LM_D4 = word.bits.b4;
    LM_D5 = word.bits.b5;
    LM_D6 = word.bits.b6;
    LM_D7 = word.bits.b7;                // attendre au moins 50 ns avant LM_WRITE = 1

    Nop(); Nop();                        // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 100 ns + 50 ns)
    LM_WRITE = 1;                        // attendre au moins 120 ns avant de changer les données

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Lire un mot envoyé par le LM (un mot = 2 octets = 16 bits).
 * @note Il n'est évidemment pas possible de lire 2 LMs simultanément, puisqu'ils partagent
 * le même bus de donnée.
 * 
 * @param        axis        l'axe moteur correspondant au LM à lire, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite @n
 *                             #MOTOR_LEFT        le moteur de gauche
 *
 * @return        word        le mot reçu, sur 16 bits
*/
WORD readDataWord(char axis) {
    WORD_VAL word;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

        waitBusyLM(axis);
        dataBusDirection(DATA_IN);
    
        // Si axis = MOTOR_BOTH, LM_CS1 = LM_CS2 = 1 : on ne sélectionne aucun LM, pour éviter un conflit sur le bus
        LM_CS1 = ((axis & MOTOR_LEFT) != 0);
        LM_CS2 = ((axis & MOTOR_RIGHT) != 0);
    
        LM_PS = PS_DATA;                     // attendre au moins 30 ns avant LM_READ = 0
        Nop();                               // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)
        LM_READ = 0;                         // attendre au moins 180 ns avant de lire le bus
        Nop(); Nop(); Nop();                 // attends 3 cycles, à 48 MHz / 4 = 250 ns (> 180 ns)
    
        word.byte.HB = LM_DATA_BUS;
    
        LM_READ = 1;                         // attendre au moins 120 ns avant LM_READ = 0
        Nop(); Nop();                        // attends 2 cycles, à 48 MHz / 4 = 166 ns (> 120 ns)
        LM_READ = 0;                         // attendre au moins 180 ns avant de lire le bus
        Nop(); Nop(); Nop();                 // attends 3 cycles, à 48 MHz / 4 = 250 ns (> 180 ns)
    
        word.byte.LB = LM_DATA_BUS;
    
        LM_READ = 1;                         // attendre au moins 30 ns avant une autre action
    
        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
        return word.Val;
    }   
}

/**
 * Lire l'état d'un LM.
 * @note Il n'est évidemment pas possible de lire l'état de 2 LMs simultanément, puisqu'ils
 * partagent le même bus de donnée.
 *
 * @param        axis        l'axe moteur correspondant au LM à lire, peut valoir : @n
 *                             #MOTOR_RIGHT    le moteur de droite @n
 *                             #MOTOR_LEFT        le moteur de gauche
 *
 * @return        status        l'état du LM, sur 8 bits
*/
BYTE readStatus(char axis) {
    BYTE status = 0;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

        waitBusyLM(axis);
        dataBusDirection(DATA_IN);

        // Si axis = MOTOR_BOTH, LM_CS1 = LM_CS2 = 1 : on ne sélectionne aucun LM, pour éviter un conflit sur le bus
        LM_CS1 = ((axis & MOTOR_LEFT) != 0);
        LM_CS2 = ((axis & MOTOR_RIGHT) != 0);
    
        LM_PS = PS_COMMAND;                  // attendre au moins 30 ns avant LM_READ = 0
        Nop();                               // attends 1 cycle, à 48 MHz / 4 = 83.3 ns (> 30 ns)
        LM_READ = 0;                         // attendre au moins 180 ns avant de lire le bus
        Nop(); Nop(); Nop();                 // attends 3 cycles, à 48 MHz / 4 = 250 ns (> 180 ns)
    
        status = LM_DATA_BUS;
    
        LM_READ = 1;                         // attendre au moins 30 ns avant une autre action

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
        return status;
    }   
}

/**
 * Lire un bit d'état du LM.
 * Cette fonction appelle simplement readStatus() et applique le masque correspondant au
 * bit à lire.
 * 
 * @param        axis        l'axe moteur correspondant au LM à lire, peut valoir : @n
 *                             #MOTOR_RIGH        le moteur de droite @n
 *                             #MOTOR_LEFT        le moteur de gauche
 *
 * @param        bit            le bit à lire, peut valoir : @n
 *                             #LM_BUSY_BIT                Busy bit @n
 *                             #LM_COMMAND_ERROR           Command Error [Interrupt] @n
 *                             #LM_TRAJECTORY_COMPLETE     Trajectory Complete [Interrupt] @n
 *                             #LM_INDEX_PULSE             Index Pulse Observed [Interrupt] @n
 *                             #LM_WRAP_AROUND             Wraparound Occurred [Interrupt] @n
 *                             #LM_POSITION_ERROR          Excessive Position Error [Interrupt] @n
 *                             #LM_BREAKPOINT              Breakpoint Reached [Interrupt] @n
 *                             #LM_MOTOR_OFF               Motor Off
 * 
 * @return        value        la valeur du bit, 0 (#FALSE) ou 1 (#TRUE).
*/
BOOL readStatusBit(char axis, BYTE type) {
	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else
        return ((readStatus(axis) & type) != 0);
}

/**
 * Reset certains bits d'interruption du/des LM(s).
 * 
 * @param        axis        l'axe moteur correspondant au LM à programmer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * @param        type        type d'interruption à resetter
*/
void resetInterrupt(char axis, WORD type) {
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    writeCommand(axis, LM_CMD_RSTI);
    writeDataWord(axis, ~type);

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Reset tous les bits d'interruption du/des LM(s).
 * 
 * @param        axis        l'axe moteur correspondant au LM à programmer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void resetAllInterrupt(char axis) {
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    writeCommand(axis, LM_CMD_RSTI);
    writeDataWord(axis, 0x0000);

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Initialise un LM, c'est-à-dire :
 * - Contrôle que le RESET s'est fait correctement ;
 * - Active ou non certaines interruptions ;
 * - Reset les interruptions ;
 * - Charge et applique les paramètres du correcteur.
 * 
 * @note Cette fonction s'assure que le LM est correctement initialisé. Si ce n'est pas le
 * cas, le LM est resetté jusqu'à ce que son état soit correct. Dans le cas d'un LM
 * défaillant, il est possible que cette fonction bloque le programme indéfiniment, si
 * aucun RESET correct n'est obtenu.
 * @note Cette fonction effectue un RESET en envoyant au LM la commande #LM_CMD_RESET. Elle
 * n'effectue pas de RESET hardware avec #LM_RST, car cela affecterait les 2 LMs 
 * simultanément, ce qui invaliderait l'initialisation de l'autre LM !
 * @note Une cause possible du blocage de cette fonction est que l'état de la sortie #LM_RST
 * ait été altéré durant l'exécution du programme, empêchant le LM de s'initialiser
 * (RESET hardware permanent si #LM_RST est à 0).
 *
 * @param        axis        l'axe moteur correspondant au LM à initialiser, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite @n
 *                             #MOTOR_LEFT        le moteur de gauche
 *
 * @param        kp            la constante proportionnelle pour le correcteur, sur 16 bits
 * @param        ki            la constante intégrale pour le correcteur, sur 16 bits
 * @param        kd            la constante dérivée pour le correcteur, sur 16 bits
 * @param        il            limite d'intégration, sur 16 bits
*/
void initLM(char axis, WORD kp, WORD ki, WORD kd, WORD il) {
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    // Reset du LM (si nécessaire)
    while (readStatus(axis) != 0x84 && readStatus(axis) != 0xC4) {
initLM_RESET:
        writeCommand(axis, LM_CMD_RESET);    // attendre au moins 1.5 ms
        Delay10KTCYx(2);                    // attends 20000 cycles, à 48 MHz / 4 = 1.67 ms (> 1.5 ms)
    }

    // Activation de certaines interruptions
    writeCommand(axis, LM_CMD_MSKI);
    writeDataWord(axis, LM_COMMAND_ERROR | LM_TRAJECTORY_COMPLETE | LM_POSITION_ERROR | LM_BREAKPOINT);

    // Reset des interruptions
    resetAllInterrupt(axis);

    if (readStatus(axis) != 0x80 && readStatus(axis) != 0xC0)
        goto initLM_RESET;

    if (readStatus(axis) == 0xC0)
        resetAllInterrupt(axis);

    // Chargement des paramètres du filtre
    writeCommand(axis, LM_CMD_LFIL);
    writeDataWord(axis, 0x0000 | LM_LFIL_KP | LM_LFIL_KI | LM_LFIL_KD | LM_LFIL_IL);
    writeDataWord(axis, kp);
    writeDataWord(axis, ki);
    writeDataWord(axis, kd);
    writeDataWord(axis, il);

    // Application du filtre
    writeCommand(axis, LM_CMD_UDF);

    // Sécurité
    writeCommand(axis, LM_CMD_LPES);
    writeDataWord(axis, LM_PES_LIMIT);

    trajLoaded = 0;
    trajEngaged = 0;
    trajCompleted = 0;

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Initialise les 2 LMs avec les paramètres par défaut.
 * Cette fonction doit être appelée au moins une fois avant le chargement de la première trajectoire.
 * 
 * @return        success        renvoie #TRUE en cas de succès et #FALSE si une erreur est survenue
*/
BOOL initLMs(void) {
    WORD_VAL kp, ki, kd, il;

    // KP
    kp.byte.HB = ReadEEPROM(0x01);       // HB
    kp.byte.LB = ReadEEPROM(0x02);       // LB
    // KI
    ki.byte.HB = ReadEEPROM(0x03);       // HB
    ki.byte.LB = ReadEEPROM(0x04);       // LB
    // KD
    kd.byte.HB = ReadEEPROM(0x05);       // HB
    kd.byte.LB = ReadEEPROM(0x06);       // LB
    // LI
    il.byte.HB = ReadEEPROM(0x07);       // HB
    il.byte.LB = ReadEEPROM(0x08);       // LB

    initLM(MOTOR_RIGHT, kp.Val, ki.Val, kd.Val, il.Val);

    // KP
    kp.byte.HB = ReadEEPROM(0x09);       // HB
    kp.byte.LB = ReadEEPROM(0x0A);       // LB
    // KI
    ki.byte.HB = ReadEEPROM(0x0B);       // HB
    ki.byte.LB = ReadEEPROM(0x0C);       // LB
    // KD
    kd.byte.HB = ReadEEPROM(0x0D);       // HB
    kd.byte.LB = ReadEEPROM(0x0E);       // LB
    // LI
    il.byte.HB = ReadEEPROM(0x0F);       // HB
    il.byte.LB = ReadEEPROM(0x10);       // LB

    initLM(MOTOR_LEFT, kp.Val, ki.Val, kd.Val, il.Val);

    writeCommand(MOTOR_BOTH, LM_CMD_DFH);

    return (!readStatusBit(MOTOR_RIGHT, LM_COMMAND_ERROR) && !readStatusBit(MOTOR_LEFT, LM_COMMAND_ERROR));
}

/**
 * Programme une nouvelle position à atteindre.
 * 
 * @param        axis        l'axe moteur correspondant au LM à programmer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * 
 * @param        pos            position à atteindre, sur 32 bits
 * @param        vel            vitesse max., sur 32 bits
 * @param        acc            accélération, sur 32 bits @n
 *                             /!\ uniquement si l'on part d'une position à l'arrêt
 *                             l'accélération ne peut pas être changée en court de trajectoire
*/
void newPosition(char axis, DWORD pos, DWORD vel, DWORD acc) {
    DWORD_VAL posData;
    DWORD_VAL velData = {vel};
    DWORD_VAL accData = {acc};
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    writeCommand(axis, LM_CMD_LTRJ);

    if (accData.Val != NULL) {
        writeDataWord(axis, LM_LTRJ_LOAD_POS | LM_LTRJ_POS_REL | LM_LTRJ_LOAD_VEL | LM_LTRJ_LOAD_ACC);
        writeDataWord(axis, accData.word.HW);
        writeDataWord(axis, accData.word.LW);
    }
    else
        writeDataWord(axis, LM_LTRJ_LOAD_POS | LM_LTRJ_POS_REL | LM_LTRJ_LOAD_VEL);

    writeDataWord(axis, velData.word.HW);
    writeDataWord(axis, velData.word.LW);

    if (CON_MOTOR_RIGHT == CON_MOTOR_LEFT) {
        // Dans ce cas, on peut écrire simultanément sur les 2 LM pour gagner du temps
        posData.Val = CON_MOTOR_RIGHT * pos;
        writeDataWord(axis, posData.word.HW);
        writeDataWord(axis, posData.word.LW);
    }
    else {
        if (axis & MOTOR_LEFT) {
            posData.Val = CON_MOTOR_LEFT * pos;
            writeDataWord(MOTOR_LEFT, posData.word.HW);
            writeDataWord(MOTOR_LEFT, posData.word.LW);
        }

        if (axis & MOTOR_RIGHT) {
            posData.Val = CON_MOTOR_RIGHT * pos;
            writeDataWord(MOTOR_RIGHT, posData.word.HW);
            writeDataWord(MOTOR_RIGHT, posData.word.LW);
        }
    }

    trajLoaded = 1;
    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Programme une nouvelle vitesse à atteindre.
 * 
 * @param        axis        l'axe moteur correspondant au LM à programmer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * 
 * @param        vel            vitesse, sur 32 bits
 * @param        acc            accélération, sur 32 bits @n
 *                             /!\ uniquement si l'on part d'une position à l'arrêt
 *                             l'accélération ne peut pas être changée en court de trajectoire
 * 
 * @param        dir          sens de rotation, peut valoir : @n
 *                              1       avance @n
 *                             -1       recule
*/
void newVelocity(char axis, DWORD vel, DWORD acc, char dir) {
    DWORD_VAL velData = {vel};
    DWORD_VAL accData = {acc};
    WORD arg = LM_LTRJ_LOAD_VEL | LM_LTRJ_VEL_MODE;
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    writeCommand(axis, LM_CMD_LTRJ);

    if (accData.Val != NULL)
        arg|= LM_LTRJ_LOAD_ACC;

    if (CON_MOTOR_RIGHT == CON_MOTOR_LEFT) {
        // Dans ce cas, on peut écrire simultanément sur les 2 LM pour gagner du temps
        if (dir == CON_MOTOR_RIGHT)
            arg|= LM_LTRJ_FORWARD_DIR;

        writeDataWord(axis, arg);
    }
    else {
        if (axis & MOTOR_LEFT) {
            if (dir == CON_MOTOR_LEFT)
                arg|= LM_LTRJ_FORWARD_DIR;

            writeDataWord(MOTOR_LEFT, arg);
        }

        if (axis & MOTOR_RIGHT) {
            arg = LM_LTRJ_LOAD_VEL | LM_LTRJ_VEL_MODE;      // Il faut réinitialiser arg

            if (dir == CON_MOTOR_RIGHT)
                arg|= LM_LTRJ_FORWARD_DIR;

            writeDataWord(MOTOR_RIGHT, arg);
        }
    }

    if (accData.Val != NULL) {
        writeDataWord(axis, accData.word.HW);
        writeDataWord(axis, accData.word.LW);
    }

    writeDataWord(axis, velData.word.HW);
    writeDataWord(axis, velData.word.LW);

    trajLoaded = 1;
    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Change la vitesse en court.
 * 
 * @param        axis        l'axe moteur correspondant au LM à programmer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * 
 * @param        vel          vitesse, sur 32 bits
 * @param        dir          sens de rotation, peut valoir : @n
 *                              1       avance @n
 *                             -1       recule
*/
void changeVelocity(char axis, DWORD vel, char dir) {
    newVelocity(axis, vel, NULL, dir);
}

/**
 * Démarre la trajectoire programmée.
 * 
 * @param        axis        l'axe moteur correspondant au LM à démarrer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
*/
void start(char axis) {
    if (axis & MOTOR_RIGHT)
        posRight = getRealPosition(MOTOR_RIGHT);

    if (axis & MOTOR_LEFT)
        posLeft = getRealPosition(MOTOR_LEFT);

    trajLoaded = 0;

    writeCommand(axis, LM_CMD_STT);
    trajEngaged = 1;
    trajCompleted = 0;
}

/**
 * Arrête le/les moteur(s).
 * 
 * @param        axis        l'axe moteur correspondant au LM à démarrer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 *
 * @param        type        type d'arrêt, peut valoir : @n
 *                             #LM_LTRJ_MOTOR_OFF      arrêt en roue libre @n
 *                             #LM_LTRJ_STOP_ABRUPT    arrêt brutale @n
 *                             #LM_LTRJ_STOP_SMOOTH    arrêt doux
*/
void stop(char axis, WORD type) {
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    writeCommand(axis, LM_CMD_LTRJ);
    writeDataWord(axis, type);

    //writeCommand(axis, LM_CMD_STT);
    trajEngaged = 1;
    trajCompleted = 0;

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Ajoute un breakpoint pour l'enchainement de trajectoires.
 *
 * @param        axis        l'axe moteur correspondant au LM à démarrer, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             #MOTOR_BOTH        les 2 moteurs
 * @param        pos         la position du breakpoint, depuis le début de la trajectoire (rel = 0) 
 *                           ou par rapport à la position finale désirée (rel = 1), sur 16 bits
 * @param        rel         indique si la position du breakpoint est exprimée en valeur relative (rel = 1) ou absolue (rel = 0)
*/
void setBreakpoint(char axis, DWORD pos, BOOL rel) {
    DWORD_VAL posData;
    static unsigned char GIE_Status;

    GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
    INTCONbits.GIE = 0;              // Disable global interrupts

    if (rel) {
        posData.Val = pos;
	    writeCommand(axis, LM_CMD_SBPR);
        writeDataWord(axis, posData.word.HW);
        writeDataWord(axis, posData.word.LW);
    }
    else {
	    writeCommand(axis, LM_CMD_SBPA);

        if (axis & MOTOR_LEFT) {
            posData.Val = posLeft + CON_MOTOR_LEFT * pos;
            writeDataWord(MOTOR_LEFT, posData.word.HW);
            writeDataWord(MOTOR_LEFT, posData.word.LW);
        }

        if (axis & MOTOR_RIGHT) {
            posData.Val = posRight + CON_MOTOR_RIGHT * pos;
            writeDataWord(MOTOR_RIGHT, posData.word.HW);
            writeDataWord(MOTOR_RIGHT, posData.word.LW);
        }
    }

    INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
}

/**
 * Récupère la position désirée (la consigne) des moteurs déterminée par les LMs.
 * Attention : cela ne correspond pas à la position finale à atteindre, mais bien
 * à la consigne à l'instant présent en entrée de la boucle d'asservissement.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return        DWORD      la position de consigne, sur 32 bits
*/
DWORD getDesiredPosition(char axis) {
	DWORD_VAL posData;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

		writeCommand(axis, LM_CMD_RDDP);
		posData.word.HW = readDataWord(axis);
		posData.word.LW = readDataWord(axis);

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
		return posData.Val;
	}
}

/**
 * Récupère la position véritable des moteurs déterminée par les LMs.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return        DWORD      la position réelle, sur 32 bits
*/
DWORD getRealPosition(char axis) {
	DWORD_VAL posData;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

		writeCommand(axis, LM_CMD_RDRP);
		posData.word.HW = readDataWord(axis);
		posData.word.LW = readDataWord(axis);

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
		return posData.Val;
	}
}

/**
 * Récupère la vitesse désirée (la consigne) des moteurs déterminée par les LMs.
 * Attention : cela ne correspond pas à la vitesse finale à atteindre, mais bien
 * à la consigne à l'instant présent en entrée de la boucle d'asservissement.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return        DWORD      la vitesse de consigne, sur 32 bits
*/
DWORD getDesiredVelocity(char axis) {
	DWORD_VAL velData;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

		writeCommand(axis, LM_CMD_RDDV);
		velData.word.HW = readDataWord(axis);
		velData.word.LW = readDataWord(axis);

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
		return velData.Val;
	}
}

/**
 * Récupère la vitesse véritable des moteurs déterminée par les LMs.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return        DWORD      la vitesse réelle, sur 32 bits
*/
DWORD getRealVelocity(char axis) {
	DWORD_VAL velData;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

		writeCommand(axis, LM_CMD_RDRV);
		velData.word.HW = readDataWord(axis);
		velData.word.LW = 0;

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
		return velData.Val;
	}
}

/**
 * Récupère la valeur du terme d'intégration.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return       WORD       la valeur de l'intégration, sur 16 bits
*/
WORD getIntegrationSum(char axis) {
	WORD data;
    static unsigned char GIE_Status;

	if (axis != MOTOR_RIGHT && axis != MOTOR_LEFT) {
        error(ERR_INVALID_AXIS);
		return 0;
    }   
	else {
        GIE_Status = INTCONbits.GIE;     // Save global interrupt enable bit
        INTCONbits.GIE = 0;              // Disable global interrupts

		writeCommand(axis, LM_CMD_RDSUM);
        data = readDataWord(axis);

        INTCONbits.GIE = GIE_Status;     // Restore the original global interrupt status
        return data;
	}
}

/**
 * Récupère la valeur de la distance actuelle relativement au début de la trajectoire.
 *
 * @param        axis        l'axe moteur correspondant au LM, peut valoir : @n
 *                             #MOTOR_RIGHT       le moteur de droite uniquement @n
 *                             #MOTOR_LEFT        le moteur de gauche uniquement @n
 *                             (spécifier les #MOTOR_BOTH est bien évidemment impossible)
 *
 * @return       short       la distance parcourue, en mm, sur 16 bits
*/
short getRelPos(char axis) {
    if (axis == MOTOR_RIGHT)
        return ((float) ((long) getRealPosition(MOTOR_RIGHT) - (long) posRight) / (float) COEF_RIGHT_WHEEL * CON_MOTOR_RIGHT);
    else if (axis == MOTOR_LEFT)
        return ((float) ((long) getRealPosition(MOTOR_LEFT) - (long) posLeft) / (float) COEF_LEFT_WHEEL * CON_MOTOR_LEFT);
    else {
        error(ERR_INVALID_AXIS);
        return 0;
    }
}

/**
 * Avance.
 * 
 * @param        pos            position à atteindre, en mm, sur 16 bits
 * @param        vel            vitesse max., en mm/s, sur 16 bits
 * @param        acc            accélération, en mm/s², sur 16 bits
*/
void moveForward(short pos, short vel, short acc) {
    newPosition(MOTOR_BOTH,
        (long) ((float) pos * COEF_RIGHT_WHEEL), 
        (long) ((float) vel * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_RIGHT_WHEEL * CONST_ACC));
    start(MOTOR_BOTH);
}

/**
 * Recule.
 * 
 * @param        pos            position à atteindre, en mm, sur 16 bits
 * @param        vel            vitesse max., en mm/s, sur 16 bits
 * @param        acc            accélération, en mm/s², sur 16 bits
*/
void moveBackward(short pos, short vel, short acc) {
    newPosition(MOTOR_BOTH,
        - (long) ((float) pos * COEF_RIGHT_WHEEL), 
        (long) ((float) vel * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_RIGHT_WHEEL * CONST_ACC));
    start(MOTOR_BOTH);
}

/**
 * Tourne vers la droite, le centre de rotation étant le milieu de l'axe des roues.
 * 
 * @param        angle        angle de rotation par rapport à la direction initiale du robot, en °, sur 16 bits
 * @param        vel          vitesse max., en mm/s, sur 16 bits
 * @param        acc          accélération, en mm/s², sur 16 bits
*/
void turnRight(short angle, short vel, short acc) {
    newPosition(MOTOR_RIGHT,
        - (long) ((float) angle * CONST_POS * WHEELS_DIST * COEF_RIGHT_WHEEL), 
        (long) ((float) vel * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_RIGHT_WHEEL * CONST_ACC));
    newPosition(MOTOR_LEFT,
        (long) ((float) angle * CONST_POS * WHEELS_DIST * COEF_LEFT_WHEEL),
        (long) ((float) vel * COEF_LEFT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_LEFT_WHEEL * CONST_ACC));
    start(MOTOR_BOTH);
}

/**
 * Tourne vers la gauche, le centre de rotation étant le milieu de l'axe des roues.
 * 
 * @param        angle        angle de rotation par rapport à la direction initiale du robot, en °, sur 16 bits
 * @param        vel          vitesse max., en mm/s, sur 16 bits
 * @param        acc          accélération, en mm/s², sur 16 bits
*/
void turnLeft(short angle, short vel, short acc) {
    newPosition(MOTOR_RIGHT,
        (long) ((float) angle * CONST_POS * WHEELS_DIST * COEF_RIGHT_WHEEL), 
        (long) ((float) vel * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_RIGHT_WHEEL * CONST_ACC));
    newPosition(MOTOR_LEFT,
        - (long) ((float) angle * CONST_POS * WHEELS_DIST * COEF_LEFT_WHEEL),
        (long) ((float) vel * COEF_LEFT_WHEEL * CONST_VEL),
        (long) ((float) acc * COEF_LEFT_WHEEL * CONST_ACC));
    start(MOTOR_BOTH);
}

/**
 * Tourne autour d'un centre quelconque situé sur l'axe des roues.
 * Ceci est une fonction généralisée des fonctions turnRight() et turnLeft().
 * 
 * @param        angle        angle de rotation par rapport à la direction initiale du robot, en °, sur 16 bits
 * @param        vel          vitesse max., en mm/s, sur 16 bits
 * @param        acc          accélération, en mm/s², sur 16 bits
 * @param        c            position du centre de rotation par rapport au milieu de l'axe des roues, en mm ou en %, sur 16 bits
 * @param        rel          indique si c est exprimé en mm (rel = 0) ou en % de la distance du milieu à la roue, #WHEELS_DIST/2 (rel = 1)
 *
 *                  -100       0        100
 * relative = 1:     ||--------.--------||
 *                   LW                 RW
 *
 * @note angle et c peuvent être positifs ou négatifs. @n
 * Lorsqu'exprimé en %, c peut être supérieur à 100 en valeur absolue.
*/
void turn(short angle, short vel, short acc, short c, BOOL rel) {
    float d1, d2, d1d2, vel1, vel2, acc1, acc2;

    if (rel)
        c = WHEELS_DIST / 2.0 * (c / 100.0);

    d1 = (float) angle * 2.0 * (WHEELS_DIST / 2.0 - c);
    d2 = - (float) angle * 2.0 * (WHEELS_DIST / 2.0 + c);

    d1d2 = (d1 / d2);
    d1d2 = (d1d2 >= 0) ? d1d2 : -d1d2;

    vel1 = (2.0 * d1d2 * (float) vel) / (1.0 + d1d2);
    vel2 = 2.0 * (float) vel - vel1;

    acc1 = (2.0 * d1d2 * (float) acc) / (1.0 + d1d2);
    acc2 = 2.0 * (float) acc - acc1;

    newPosition(MOTOR_RIGHT,
        (long) (d1 * CONST_POS * COEF_RIGHT_WHEEL), 
        (long) (vel1 * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) (acc1 * COEF_RIGHT_WHEEL * CONST_ACC));

    newPosition(MOTOR_LEFT,
        (long) (d2 * CONST_POS * COEF_RIGHT_WHEEL), 
        (long) (vel2 * COEF_RIGHT_WHEEL * CONST_VEL),
        (long) (acc2 * COEF_RIGHT_WHEEL * CONST_ACC));

    start(MOTOR_BOTH);
}

/**
 * Aller à un emplacement, relativement à la position actuelle.
 *
 *        ^
 *        |          (x,y)
 *      y +        . 
 *        |
 *        |
 *        |
 *        |
 *        |--------+----->
 *      (0,0)      x
 *        
 *
 * @param        x            abscisse, en mm, sur 16 bits
 * @param        y            ordonnée, en mm, sur 16 bits
 * @param        vel          vitesse max., en mm/s, sur 16 bits
 * @param        acc          accélération, en mm/s², sur 16 bits
 * @param        mode         mode de la trajectoire, peut valoir : @n
 *                             #GOTO_STRAIGHT          va tout droit au point désiré @n
 *                             #GOTO_CURVE_RIGHT       va en contournant par la droite @n
 *                             #GOTO_CURVE_LEFT        va en contournant par la gauche
 * @param        d            distance de contournement
*/
void goTo(short x, short y, short vel, short acc, BYTE mode, short d) {
    short posRight, posLeft, velRight, velLeft;
    float angle1, angle2, angle3;
    float r, ux, uy, vx, vy, Ox, Oy, u2x, u2y;
    float a;

    a = (((float) x)*((float) x) + ((float) y)*((float) y))/4.0;
    angle1 = atan2((float)y, (float)x);

    if (mode == GOTO_STRAIGHT || d == 0) {
        turnRight(90.0 - angle1*180.0/PI, vel, acc);
        waitTrajComplete();
        moveForward(sqrt(4.0 * a), vel, acc);
    }
    else {
		// Trajectory radius
		r = a/(2.0*d)+d/2.0;

		// Director vector of start to end
		ux = 1.0/(2.0*sqrt(a))*x;
		uy = 1.0/(2.0*sqrt(a))*y;

		// Rotate last vector by 90°
		vx = - uy;
		vy = ux;

		// Position of the circle's center
		Ox = x/2.0 - (r-d)*vx;
		Oy = y/2.0 - (r-d)*vy;

		// Director vector of start to center rotated by 90°
		u2x = - Oy / r;
		u2y =   Ox / r;

		// Tangent angle from the X axis
		angle2 = atan2(u2y, u2x);

		// Angle to travel on the circle
		angle3 = 2.0*atan2(sqrt(a), r-d);
    
        if (mode == GOTO_CURVE_LEFT) {
          // angle2 = angle2 if we pass left
    
          turnRight(90.0-angle2*180.0/PI, vel, acc);
    
          posRight = (r - (float) WHEELS_DIST/2.0)*angle3;
          posLeft  = (r + (float) WHEELS_DIST/2.0)*angle3;
          velLeft  = vel;
          velRight = ((float) posRight / ((float) posLeft)) * vel;
        }
        else {
          // If we pass right, we need to adjust the calculated angle
          angle2 = 2.0*angle1 - angle2;
    
          turnRight(90.0-angle2*180.0/PI, vel, acc);
    
          posRight = (r + (float) WHEELS_DIST/2.0)*angle3;
          posLeft  = (r - (float) WHEELS_DIST/2.0)*angle3;
          velRight = vel;
          velLeft = ((float) posLeft / ((float) posRight)) * vel;
        }
    
        waitTrajComplete();
 
        newPosition(MOTOR_RIGHT,
            (long) ((float) posRight * COEF_RIGHT_WHEEL), 
            (long) ((float) velRight * COEF_RIGHT_WHEEL * CONST_VEL),
            (long) ((float) acc * COEF_RIGHT_WHEEL * CONST_ACC));
        newPosition(MOTOR_LEFT,
            (long) ((float) posLeft * COEF_LEFT_WHEEL),
            (long) ((float) velLeft * COEF_LEFT_WHEEL * CONST_VEL),
            (long) ((float) acc * COEF_LEFT_WHEEL * CONST_ACC));

        start(MOTOR_BOTH);
    }
}

/**
 * Indique si la dernière trajectoire est terminée.
 * Attention : cette fonction renvoie 1 dès que la trajectoire programmée
 * est atteinte, ce qui signifie qu'en cas de dépassement (ce qui peut se
 * produire si le correcteur est mal réglé par exemple), cette fonction
 * peut renvoyer 1 alors même que le moteur continue à tourner.
 * Ainsi, dans le cas d'un système pseudo-armorti, il n'est pas possible
 * d'enchainer des trajectoires avec cette fonction et sans délais
 * supplémentaire sans perdre drastiquement en précision.
 *
 * @return        done        renvoie #TRUE en si la trajectoire est terminée et #FALSE sinon
*/
BOOL isTrajComplete(void) {
//    Delay10KTCYx(12);                    // attends 120000 cycles, à 48 MHz / 4 = 10 ms

    if ((trajCompleted & MOTOR_RIGHT) && (trajCompleted & MOTOR_LEFT)) {
        trajEngaged = 0;
        return TRUE;
    }
    else
        return FALSE;
}

/**
 * Attend la fin de la trajectoire en court.
 * Même remarque que pour la fonction isTrajComplete().
 * @attention Cette fonction bloque entièrement le PIC durant toute la durée d'attente de fin
 * de trajectoire. Il est préférable d'utiliser la fonction isTrajComplete() pour venir
 * vérifier à intervalles réguliers si la trajectoire est complétée ou non, tout en permettant
 * au programme de tourner le reste du temps.
*/
void waitTrajComplete(void) {
    while (!isTrajComplete());
}

/**
 * Indique si une trajectoire est en cours.
 * Si une trajectoire en cours est terminé, le résultat de cette fonction sera maintenu à TRUE
 * jusqu'à ce qu'une des fonctions isTrajComplete() ou waitTrajComplete() soit appelée.
 *
 * @return        done        renvoie #TRUE en si une trajectoire est en cours et #FALSE sinon
*/
BOOL isTrajEngaged(void) {
    return trajEngaged;
}

#endif
