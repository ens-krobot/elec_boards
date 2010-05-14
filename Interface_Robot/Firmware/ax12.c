/**
 * @file ax12.c
*/

#ifndef AX12_C
#define AX12_C

// Macro permettant d'attendre 1 useconde
#define DELAY_US() {Nop(); Nop(); Nop();    \\
                    Nop(); Nop(); Nop();    \\
                    Nop(); Nop(); Nop();    \\
                    Nop(); Nop(); Nop();}   \\

#include "ax12.h"

volatile BOOL glbReceived = 0;
volatile BYTE glbBuffer[32] = {0};
volatile BYTE glbParam[32];
volatile BOOL glbTimedOut = FALSE;

/**
 * Fonction permettant d'initialiser le bus de communication avec les AX-12
 */
void initAX12(void) {
    /* Initialisation des PINs */
    TRISD&= 0b11011111;        /* DIR_PORT */
    
    /* Initialisation de l'Ã©tat des PINs */
    DIR_PORT = 1;

    /* Configuration de l'USART */
    OpenUSART(USART_TX_INT_OFF
        & USART_RX_INT_ON
        & USART_ASYNCH_MODE
        & USART_EIGHT_BIT
        & USART_BRGH_HIGH,
        25); /* FOSC/[16 (n + 1)] = 115200 bauds */
}

/**
 * Fonction de gestion des interruptions sur l'UART reliÃ©e aux AX-12
 */
void interruptAX12(void) {
    static BYTE idx = 0;
    static BYTE len;
    static BYTE write = 0;
    BYTE buf;
    BYTE chksum;
    UINT8 i;
    
    // On attend qu'un octet soit disponible puis on le lit
    while (BusyUSART());
    buf = getcUSART();
    
    // On commence la reception d'un paquet si le buffer de reception est vide
    if (idx == 0 && !glbReceived)
        write = 1;

    // Si l'on est en train de recevoir, on traite l'octet en cours
    if (write == 1) {
        glbBuffer[idx] = buf;
    
        if (idx < 2 && buf != 0xFF) {
            // Ce n'est pas un paquet AX12
            error(ERR_AX12_WRONG_PACKET);
            write = 0;
            idx = 0;
        }
        
        if (idx == 3)
            len = buf+3;
    
        if (idx >= 3 && idx == len) {
            // Calcul du checksum
            // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
            chksum = 0;
    
            for (i = 2; i < len; i++)
                chksum+= glbBuffer[i];    

            if (glbBuffer[len] != (~chksum)) {
                error(ERR_AX12_CHKSUM);
                glbReceived = 0;
            }
            else {
                // Le paquet est valide, on peut tester le renvoi d'erreur par l'AX12
                if (glbBuffer[4] != 0)
                    // Il y a une erreur...                 
                    error(ERR_AX12_ERROR);
                    error(glbBuffer[4]);
                // Un paquet a été reçu
                glbReceived = 1;
            }
            
            // On reinitialise les varaibles statiques pour le prochain paquet
            write = 0;
            idx = 0;
        }
        else
            idx++;
    }
}

/**
 * Envoie un paquet d'instruction sur le bus dynamixel
 *
 * @param   id            Identifiant du destinataire au ID_BROADCAST
 *
 * @param   inst          Type d'instruction
 *
 * @param   paramLength   Nombre de parametres
 */
void sendInstPacket(BYTE id, BYTE inst, BYTE paramLength) {
    UINT8 i;
    BYTE buf[32];
    BYTE chksum;

    buf[0] = 0xFF;
    buf[1] = 0xFF;
    buf[2] = id;
    buf[3] = 2+paramLength;
    buf[4] = inst;

    for (i = 0; i < paramLength; i++)
        buf[5+i] = glbParam[i];
    
    // Calcul du checksum
    // Check Sum = ~ (ID + Length + Instruction + Parameter1 + ... Parameter N)
    chksum = 0;
    
    for (i = 2; i < paramLength+5; i++)
        chksum+= buf[i];
    
    buf[5+paramLength] = ~chksum;
    
    // Envoi
    DIR_PORT = 0; // On place le bus en mode carte -> dynamixel
    
    for (i = 0; i < paramLength+6; i++) {
        while (BusyUSART());
        WriteUSART(buf[i]);
    }
    
    //while (BusyUSART());
    
    DIR_PORT = 1; // On replace le bus en mode dynamixel -> carte
    
    // On autorise l'ecriture du paquet de status que l'on va recevoir en reponse
    glbReceived = 0;
}

/**
 * Indique si un paquet de status est en attente de traitement
 *
 * @return 1 si un paquet est en attente, 0 sinon
 */
BOOL isStatusPacketReceived(void) {
    return glbReceived;
}

/**
 * Indique si la derniÃ¨re opÃ©ration a fait un time-out
 *
 * @return #TRUE si la derniere operation a fait un time-out, #FALSE sinon
 */
BOOL hasTimedOut(void) {
    return glbTimedOut;
}

/**
 * Rend la main quand un paquet est disponible
 */
void waitStatusPacket(void) {
    while (!glbReceived);
}

/**
 * Definit la valeur d'un parametre de l'instruction a envoyer
 *
 * @param   index   Numero du parametre a modifier
 *
 * @param   value   Valeur du parametre
 */
void setParam(BYTE index, BYTE value) {
    glbParam[index] = value;
}

/**
 * Configuration initiale d'un AX12.
 * Cette fonction effectue les opérations suivantes :
 * - Changement du baud-rate
 * - Attribution de l'ID désirée
 * Attention, cette fonction reconfigurera tous les AX12 
 * actuellement connectés (utilisation de l'ID de broadcast).
 *
 * @param   newId   Nouvel identifiant de l'AX12
 */
void configAX12(BYTE newId) {
    // Les commandes sont doublées pour limiter les risques d'erreurs de transmission

    // Changement du baud-rate
    OpenUSART(USART_TX_INT_OFF
        & USART_RX_INT_ON
        & USART_ASYNCH_MODE
        & USART_EIGHT_BIT
        & USART_BRGH_HIGH,
        2); // FOSC/[16 (n + 1)] = 1000000 bauds
    
    writeValue8(ID_BROADCAST, P_BAUD_RATE, 0x10);
    writeValue8(ID_BROADCAST, P_BAUD_RATE, 0x10);

    // Configuration de l'ID
    writeValue8(ID_BROADCAST, P_ID, newId);
    writeValue8(ID_BROADCAST, P_ID, newId);
    
    // Réinitialisation du baud-rate pour la suite des opérations
    OpenUSART(USART_TX_INT_OFF
        & USART_RX_INT_ON
        & USART_ASYNCH_MODE
        & USART_EIGHT_BIT
        & USART_BRGH_HIGH,
        25); /* FOSC/[16 (n + 1)] = 115200 bauds */

    // Dans le cas où le baud-rate était déjà configuré, on refait la configuration de l'ID,
    // car il y a des chances que cela ne se soit pas passé correctement avec le mauvais baud-rate
    writeValue8(ID_BROADCAST, P_ID, newId);
    writeValue8(ID_BROADCAST, P_ID, newId);
}

/**
 * Determine l'erreur du dernier paquet recu
 *
 * @return Code de la derniere erreur recue
 */
BYTE lastErrorAX12(void) {
    if (glbReceived)
        return glbBuffer[4];
    else
        return 0;
}

/**
 * Ping un AX12 et attend la reponse un certain temps.
 *
 * @param   id        Identifiant de l'AX12
 *
 * @param   timeOut   Temps d'attente maximum en micro-seconde
 *
 * @return #TRUE si un AX12 a repondu, #FALSE sinon
 */
BOOL pingAX12(BYTE id, UINT timeOut) {
    UINT count;

    // Envoi de la requete
    sendInstPacket(id, INST_PING, 0);

    for (count = 0; count < timeOut; count++) {
        if (glbReceived) {
            glbTimedOut = FALSE;
            return TRUE;
        }

        DELAY_US();
    }

    // Time Out !
    glbTimedOut = TRUE; 
    return FALSE;
}

/**
 * Reset un AX12
 *
 * @param   id   Identifiant de l'AX12
 */
void resetAX12(BYTE id) {
    sendInstPacket(id, INST_RESET, 0);
}

/**
 * Ecrit une valeur dans un registre 8 bits d'un AX12
 *
 * @param   id      Identifiant de l'AX12
 *
 * @param   reg     Registre a ecrire
 *
 * @param   value   Valeur a ecrire
 */
void writeValue8(BYTE id, BYTE reg, BYTE value) {
    setParam(0, reg);
    setParam(1, value);
    sendInstPacket(id, INST_WRITE, 2);
}

/**
 * Ecrit une valeur dans un registre 16 bits d'un AX12
 *
 * @param   id      Identifiant de l'AX12
 *
 * @param   reg     Registre a ecrire (adresse du poids faible)
 *
 * @param   value   Valeur a ecrire
 */
void writeValue16(BYTE id, BYTE reg, WORD value) {
    WORD_VAL word;
    word.Val = value;

    setParam(0, reg);
    setParam(1, word.byte.LB); // octet de poids faible
    setParam(2, word.byte.HB); // octet de poids fort
    sendInstPacket(id, INST_WRITE, 3);
}

/**
 * Lit la valeur d'un registre 8 bits d'un AX12
 *
 * @param   id        Identifiant de l'AX12
 *
 * @param   reg       Registre a lire
 *
 * @param   timeOut   Temps d'attente maximum de la reponse en micro-secondes
 *
 * @return #TRUE si la reponse a ete recue, #FALSE en cas de Time Out
 */
BYTE readValue8(BYTE id, BYTE reg, UINT timeOut) {
    UINT count;

    // Envoi de la requete
    setParam(0, reg);
    setParam(1, 1); // On lit 1 octet
    sendInstPacket(id, INST_READ, 2);

    for (count = 0; count < 10*timeOut; count++) {
        if (glbReceived) {
            glbTimedOut = FALSE;
            return glbBuffer[5];
        }

        DELAY_US();
    }

    // Time Out !
    glbTimedOut = TRUE;
    return 0;
}

/**
 * Lit la valeur d'un registre 16 bits d'un AX12
 *
 * @param   id        Identifiant de l'AX12
 *
 * @param   reg       Registre a lire
 *
 * @param   timeOut   Temps d'attente maximum de la reponse en micro-secondes
 *
 * @return #TRUE si la reponse a ete recue, #FALSE en cas de Time Out
 */
WORD readValue16(BYTE id, BYTE reg, UINT timeOut) {
    UINT count;
    WORD_VAL word;

    // Envoi de la requete
    setParam(0, reg);
    setParam(1, 2); // On lit 2 octets
    sendInstPacket(id, INST_READ, 2);

    for (count = 0; count < 10*timeOut; count++) {
        if (glbReceived) {
            glbTimedOut = FALSE;
            word.byte.HB = glbBuffer[6];
            word.byte.LB = glbBuffer[5];
            return word.Val;
        }

        DELAY_US();
    }

    // Time Out !
    glbTimedOut = TRUE;
    return 0;
}

/**
 * Meme fonction que @writeValue8 mais la modification sera effective apres reception
 * de la commande action.@n
 * Ceci permet de synchroniser plusieurs AX12.
 *
 * @param   id      Identifiant de l'AX12
 *
 * @param   reg     Registre a ecrire
 *
 * @param   value   Valeur a ecrire
 */
void regWrite8(BYTE id, BYTE reg, BYTE value) {
    setParam(0, reg);
    setParam(1, value);
    sendInstPacket(id, INST_REG_WRITE, 2);
}

/**
 * Meme fonction que @writeValue16 mais la modification sera effective apres reception
 * de la commande action.@n
 * Ceci permet de synchroniser plusieurs AX12.
 *
 * @param   id      Identifiant de l'AX12
 *
 * @param   reg     Registre a ecrire
 *
 * @param   value   Valeur a ecrire
 */
void regWrite16(BYTE id, BYTE reg, WORD value) {
    WORD_VAL word;
    word.Val = value;

    setParam(0, reg);
    setParam(1, word.byte.LB); // octet de poids faible
    setParam(2, word.byte.HB); // octet de poids fort
    sendInstPacket(id, INST_REG_WRITE, 3);
}

/**
 * Declenche les actions enregistrees par regWrite*
 *
 * @param   id   Identifiant de l'AX12
 */
void actionAX12(BYTE id) {
    sendInstPacket(id, INST_ACTION, 0);
}

/**
 * Deplace un AX12
 *
 * @param   id         Identifiant de l'AX12
 *
 * @param   position   Position a atteindre
 *
 * @param   speed      Vitesse de deplacement max ( 0 = maximum possible)
 * 
 * @param   mode       Mode d'envoi des données à l'AX12@n
 *                    #AX12_EXEC_NOW# va executer l'action demandee de suite@n
 *                    #AX12_EXEC_ACTION# n'executera l'action qu'une fois l'instruction ACTION envoyee
 */
void goTo(BYTE id, WORD position, WORD speed, BYTE mode) {
    WORD_VAL word;

    setParam(0, P_GOAL_POSITION);

    word.Val = position;
    setParam(1, word.byte.LB); // octet de poids faible
    setParam(2, word.byte.HB); // octet de poids fort

    word.Val = speed;
    setParam(3, word.byte.LB); // octet de poids faible
    setParam(4, word.byte.HB); // octet de poids fort
    if (mode == AX12_EXEC_NOW)
        sendInstPacket(id, INST_WRITE, 5);
    else
        sendInstPacket(id, INST_REG_WRITE, 5);
}

/**
 * Recupere la position courante d'un AX12
 *
 * @param   id        Identifiant de l'AX12
 *
 * @param   timeOut   Temps d'attente maximum de la reponse en micro-secondes
 *
 * @return #TRUE si la reponse a ete recue, #FALSE en cas de Time Out
 */
int getPosition(BYTE id, UINT timeOut) {
    return readValue16(id, P_PRESENT_POSITION, timeOut);
}

/**
 * Recupere la vitesse courante d'un AX12
 *
 * @param   id        Identifiant de l'AX12
 *
 * @param   value     Adresse ou stocker le resultat
 *
 * @param   timeOut   Temps d'attente maximum de la reponse en micro-secondes
 *
 * @return #TRUE si la reponse a ete recue, #FALSE en cas de Time Out
 */
int getSpeed(BYTE id, UINT timeOut) {
    return readValue16(id, P_PRESENT_SPEED, timeOut);
}

#endif
