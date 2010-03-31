/**
 * @file lcd.c
 * Biblioth�que de gestion de l'afficheur LCD CLDC204
*/

#include "lcd.h"

/**
 * Initialise et efface l'�cran LCD
 *
 * @param        address        adresse de l'�cran LCD sur le bus I2C
*/
void lcd_init(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();           // Absolument n�cessaire
    WriteI2C(address);
    WriteI2C(0);         // Les 3 write suivants servent � initialiser correctement l'afficheur
    WriteI2C(0);
    WriteI2C(0);
    WriteI2C(CMD_HEAD);  
    WriteI2C(CMD_CLEAR);
    StopI2C();
    CloseI2C();
}    

/**
 * Efface l'�cran LCD
 *
 * @param        address        adresse de l'�cran LCD sur le bus I2C
*/
void lcd_clear(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument n�cessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);  
    WriteI2C(CMD_CLEAR);
    StopI2C();
    CloseI2C();
}

/**
 * R�gle l'affichage du curseur sur l'�cran LCD
 *
 * @param        address           adresse de l'�cran LCD sur le bus I2C
 * @param        activate_cursor   mode du curseur :@n
 *                                  @TRUE  : active le curseur@n
 *                                  @FALSE : d�sactive le curseur
*/    
void lcd_set_cursor(BYTE address, BOOL activate_cursor)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument n�cessaire
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
 * R�gle l'activation du r�tro-�clairage sur l'�cran LCD
 *
 * @param        address              adresse de l'�cran LCD sur le bus I2C
 * @param        activate_backlight   mode du curseur :@n
 *                                     @TRUE  : active le r�tro-�clairage@n
 *                                     @FALSE : d�sactive le r�tro-�clairage
*/       
void lcd_set_backlight(BYTE address, BOOL activate_backlight)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument n�cessaire
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
 * D�place le curseur vers l'origine de l'�cran LCD
 *
 * @param        address           adresse de l'�cran LCD sur le bus I2C
*/       
void lcd_goto_origin(BYTE address)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument n�cessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    WriteI2C(CMD_GOTO_ORIGIN);
    StopI2C();
    CloseI2C();
}
    
/**
 * Positionne le curseur � une position donn�e sur l'�cran LCD
 *
 * @param        address           adresse de l'�cran LCD sur le bus I2C
 * @param        xx                coordonn�e x du curseur
 * @param        yy                coordonn�e y du curseur
*/   
void lcd_goto(BYTE address, BYTE xx, BYTE yy)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();             // Absolument n�cessaire
    WriteI2C(address);
    WriteI2C(CMD_HEAD);
    WriteI2C(CMD_GOTO_POS);
    WriteI2C(xx);
    WriteI2C(yy);
    StopI2C();
    CloseI2C();
}

/**
 * Ecrit une suite de caract�re sur l'�cran LCD � la position courante du curseur
 *
 * @param        address           adresse de l'�cran LCD sur le bus I2C
 * @param        msg               cha�ne de caract�res � �crire (se termine par NULL)
*/       
void lcd_write(BYTE address, unsigned char *msg)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument n�cessaire
    WriteI2C(address);
    putsI2C(msg);
    StopI2C();
    CloseI2C();
}

/**
 * Ecrit une suite de caract�re sur l'�cran LCD � la ligne indiqu�e
 *  Attention : le d�passement ne sera pas test�
 *
 * @param        address           adresse de l'�cran LCD sur le bus I2C
 * @param        line              num�ro de la ligne sur laquelle �crire
 * @param        msg               cha�ne de caract�res � �crire (se termine par NULL)
*/    
void lcd_write_line(BYTE address, BYTE line, unsigned char *msg)
{
    OpenI2C(MASTER, SLEW_OFF);
    StartI2C();
    IdleI2C();        // Absolument n�cessaire
    WriteI2C(address);
    WriteI2C(line);
    putsI2C(msg);
    StopI2C();
    CloseI2C();
}
