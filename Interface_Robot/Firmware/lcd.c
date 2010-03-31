/**
 * @file lcd.c
 * Bibliothèque de gestion de l'afficheur LCD CLDC204
*/

#include "lcd.h"

/**
 * Initialise et efface l'écran LCD
 *
 * @param        address        adresse de l'écran LCD sur le bus I2C
*/
void lcd_init(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();           // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(0);         // Les 3 write suivants servent à initialiser correctement l'afficheur
    WriteI2C(0);
    WriteI2C(0);
    WriteI2C(CMD_HEAD);  
    WriteI2C(CMD_CLEAR);
    StopI2C();
    CloseI2C();
}    

/**
 * Efface l'écran LCD
 *
 * @param        address        adresse de l'écran LCD sur le bus I2C
*/
void lcd_clear(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);  
    WriteI2C(CMD_CLEAR);
    StopI2C();
    CloseI2C();
}

/**
 * Règle l'affichage du curseur sur l'écran LCD
 *
 * @param        address           adresse de l'écran LCD sur le bus I2C
 * @param        activate_cursor   mode du curseur :@n
 *                                  @TRUE  : active le curseur@n
 *                                  @FALSE : désactive le curseur
*/    
void lcd_set_cursor(BYTE address, BOOL activate_cursor)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    if (activate_cursor)  
        WriteI2C(CMD_CURSOR_ON);
    else
        WriteI2C(CMD_CURSOR_OFF);
    StopI2C();
    CloseI2C();
}

/**
 * Règle l'activation du rétro-éclairage sur l'écran LCD
 *
 * @param        address              adresse de l'écran LCD sur le bus I2C
 * @param        activate_backlight   mode du curseur :@n
 *                                     @TRUE  : active le rétro-éclairage@n
 *                                     @FALSE : désactive le rétro-éclairage
*/       
void lcd_set_backlight(BYTE address, BOOL activate_backlight)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    if (activate_backlight)  
        WriteI2C(CMD_BACKLIGHT_ON);
    else
        WriteI2C(CMD_BACKLIGHT_OFF);
    StopI2C();
    CloseI2C();
}

/**
 * Déplace le curseur vers l'origine de l'écran LCD
 *
 * @param        address           adresse de l'écran LCD sur le bus I2C
*/       
void lcd_goto_origin(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    WriteI2C(CMD_GOTO_ORIGIN);
    StopI2C();
    CloseI2C();
}
    
/**
 * Positionne le curseur à une position donnée sur l'écran LCD
 *
 * @param        address           adresse de l'écran LCD sur le bus I2C
 * @param        xx                coordonnée x du curseur
 * @param        yy                coordonnée y du curseur
*/   
void lcd_goto(BYTE address, BYTE xx, BYTE yy)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    WriteI2C(CMD_GOTO_POS);
    WriteI2C(xx);
    WriteI2C(yy);
    StopI2C();
    CloseI2C();
}

/**
 * Ecrit une suite de caractère sur l'écran LCD à la position courante du curseur
 *
 * @param        address           adresse de l'écran LCD sur le bus I2C
 * @param        msg               chaîne de caractères à écrire (se termine par NULL)
*/       
void lcd_write(BYTE address, unsigned char *msg)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument nécessaire
    WriteI2C(address);
    putsI2C(msg);
    StopI2C();
    CloseI2C();
}

/**
 * Ecrit une suite de caractère sur l'écran LCD à la ligne indiquée
 *  Attention : le dépassement ne sera pas testé
 *
 * @param        address           adresse de l'écran LCD sur le bus I2C
 * @param        line              numéro de la ligne sur laquelle écrire
 * @param        msg               chaîne de caractères à écrire (se termine par NULL)
*/    
void lcd_write_line(BYTE address, BYTE line, unsigned char *msg)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument nécessaire
    WriteI2C(address);
    WriteI2C(line);
    putsI2C(msg);
    StopI2C();
    CloseI2C();
}
