#ifndef HEADER__MOTOR
#define HEADER__MOTOR

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"

#include <math.h>

#define NUM_MOTORS 4

#define MOTEUR1 1 //Moteur à gauche de l'ascenseur vue de face
#define MOTEUR2 2 //Moteur suivant dans le sens horaire vue de dessus 
#define MOTEUR3 4 //Moteur à droite de l'ascenseur vue de dessus
#define MOTEUR4 8 //Moteur de l'ascenceur principal

#define FORWARD_DIRECTION 0 //Incrémentation du compteur
#define BACKWARD_DIRECTION 1 //Décrémentation du compteur

#define MAX_PWM 2300

//Def d'une structure moteur

typedef struct {
	uint16_t currentPWM; 
	float desiredPosition; 
	float currentPosition;
	uint8_t direction; //Indique le sens et la direction du moteur
	float speed;	//Indique la vitesse du moteur
	
} motor_status;

void motorsInit(void);

//Commande des moteurs 1..4
	void setMotors(unsigned short motors, unsigned short direction);
	void enableMotors(unsigned short motors);
	void disableMotors(unsigned short motors);
	void breakMotors(unsigned short motors);
	void freeMotors(unsigned short motors);
	void setVelocity(unsigned short motors, int32_t velocity);


#endif
