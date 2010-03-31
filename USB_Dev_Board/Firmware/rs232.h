#ifndef RS232_H
#define RS232_H

#include <usart.h>         // fonctions pour l'USART (port série)
#include <string.h>        // absolument nécessaire pour utiliser strcmp()
#include <stdlib.h>        // data conversion functions
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"

// Taille maximale d'une commande (en octets)
#define RS232_TAILLE_COMMANDE            32
// Octet de fin de commande
#define RS232_FIN_COMMANDE                0x00

void RS232Receive(void);

#endif
