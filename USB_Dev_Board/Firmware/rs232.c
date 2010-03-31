#ifndef RS232_C
#define RS232_C

#include "rs232.h"

volatile char rs232_buffer[RS232_TAILLE_COMMANDE];
volatile char rs232_index = 0;

void RS232Receive(void) {
    char b;
    char buf[11], str[5];

    while (BusyUSART());
    b = getcUSART();

    if (b != RS232_FIN_COMMANDE) {
        if (rs232_index < RS232_TAILLE_COMMANDE) {
            rs232_buffer[rs232_index] = b;
            rs232_index++;
//            putcUSART(b);        // On renvoie systématiquement ce que l'on reçoit pour confirmation
        }
        else
            putrsUSART((const rom far char *) "ERREUR : Longueur maximale de la commande dépassée");
    }
    else {
        // Ajoute le caractère de fin de chaine pour strcmp()
        rs232_buffer[rs232_index] = '\0';
//        putcUSART('\0');        // Termine la confirmation de la commande

        rs232_index = 0;        // Réinitialise l'index à 0 pour la prochaine commande

        /*
            Une variable déclarée en char est stockée dans la mémoire RAM (data memory),
            tandis qu'une chaine de caractère codée "en dur" est stockée dans la mémoire ROM
            (program memory). Pour utiliser les fonctions sur la comparaison de chaines, il
            faut donc utiliser les fonctions ayant le suffixe "pgm2ram".
            (voir aussi "FAQ-5 When I dereference a pointer to a string, 
            the result is not the first character of that string. Why?")
        */

        if (strcmppgm2ram(&rs232_buffer[0], (const rom far char *) "RESET") == 0) {
            Reset();
        }
        else if (strcmppgm2ram(&rs232_buffer[0], (const rom far char *) "PING") == 0) {
            while (BusyUSART());
            putrsUSART((const rom far char *) "PONG");
        }
        else if (memcmppgm2ram((char *) &rs232_buffer[0], (const rom far char *) "RT:", 3) == 0) {
            // Lit le TRIS d'un port (exemple : "RT:A")
            buf[0] = 'R';
            buf[1] = 'R';
            buf[2] = 'T';
            buf[3] = ':';
            buf[4] = rs232_buffer[3];
            buf[5] = ':';
            buf[6] = '\0';

            while (BusyUSART());

            switch (rs232_buffer[3]) {
                case 'A':
                    strcat(&buf[0], btoa(TRISA, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'B':
                    strcat(&buf[0], btoa(TRISB, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'C':
                    strcat(&buf[0], btoa(TRISC, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'D':
                    strcat(&buf[0], btoa(TRISD, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'E':
                    strcat(&buf[0], btoa(TRISE, &str[0]));
                    putsUSART(&buf[0]); break;

                default:
                    putrsUSART((const rom far char *) "ERREUR : Mauvais argument");
            }
        }
        else if (memcmppgm2ram((char *) &rs232_buffer[0], (const rom far char *) "WT:", 3) == 0) {
            // Définit le TRIS d'une entrée/sortie (exemple : "WT:RA0:1")
            b = (0b1 << (rs232_buffer[5] - 48));

            switch (rs232_buffer[4]) {
                case 'A':
                    if (rs232_buffer[7] == '1')
                        TRISA|= b;
                    else
                        TRISA&= (0b11111111 ^ b);
                break;

                case 'B':
                    if (rs232_buffer[7] == '1')
                        TRISB|= b;
                    else
                        TRISB&= (0b11111111 ^ b);
                break;

                case 'C':
                    if (rs232_buffer[7] == '1')
                        TRISC|= b;
                    else
                        TRISC&= (0b11111111 ^ b);
                break;

                case 'D':
                    if (rs232_buffer[7] == '1')
                        TRISD|= b;
                    else
                        TRISD&= (0b11111111 ^ b);
                break;

                case 'E':
                    // Toutes les IOs du port E sont toujours des sorties (LEDs)
                    putrsUSART((const rom far char *) "ERREUR : Le TRIS du port E ne peut pas être changé");
                break;

                default:
                    putrsUSART((const rom far char *) "ERREUR : Mauvais argument");
            }
        }
        else if (memcmppgm2ram((char *) &rs232_buffer[0], (const rom far char *) "R:", 2) == 0) {
            // Lit un port (exemple : "R:A")
            buf[0] = 'R';
            buf[1] = 'R';
            buf[2] = ':';
            buf[3] = rs232_buffer[2];
            buf[4] = ':';
            buf[5] = '\0';

            while (BusyUSART());

            switch (rs232_buffer[2]) {
                case 'A':
                    strcat(&buf[0], btoa(PORTA, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'B':
                    strcat(&buf[0], btoa(PORTB, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'C':
                    strcat(&buf[0], btoa(PORTC, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'D':
                    strcat(&buf[0], btoa(PORTD, &str[0]));
                    putsUSART(&buf[0]); break;

                case 'E':
                    strcat(&buf[0], btoa(PORTE, &str[0]));
                    putsUSART(&buf[0]); break;

                default:
                    putrsUSART((const rom far char *) "ERREUR : Mauvais argument");
            }
        }
        else if (memcmppgm2ram((char *) &rs232_buffer[0], (const rom far char *) "W:", 2) == 0) {
            // Définit l'état d'une entrée/sortie (exemple : "W:RA0:1")
            b = (0b1 << (rs232_buffer[4] - 48));

            switch (rs232_buffer[3]) {
                case 'A':
                    if (rs232_buffer[6] == '1')
                        LATA|= b;
                    else
                        LATA&= (0b11111111 ^ b);
                break;

                case 'B':
                    if (rs232_buffer[6] == '1')
                        LATB|= b;
                    else
                        LATB&= (0b11111111 ^ b);
                break;

                case 'C':
                    if (rs232_buffer[6] == '1')
                        LATC|= b;
                    else
                        LATC&= (0b11111111 ^ b);
                break;

                case 'D':
                    if (rs232_buffer[6] == '1')
                        LATD|= b;
                    else
                        LATD&= (0b11111111 ^ b);
                break;

                case 'E':
                    if (rs232_buffer[6] == '1')
                        LATE|= b;
                    else
                        LATE&= (0b11111111 ^ b);
                break;

                default:
                    putrsUSART((const rom far char *) "ERREUR : Mauvais argument");
            }
        }
        else
            putrsUSART((const rom far char *) "ERREUR : Commande non reconnue");
    }
}

#endif
