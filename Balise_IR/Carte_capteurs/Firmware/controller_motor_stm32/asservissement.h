#ifndef HEADER__ASSERVISSEMENT
#define HEADER__ASSERVISSEMENT

#include <drv/gpio_stm32.h>
#include <drv/clock_stm32.h>
#include "hw/hw_led.h"
#include <drv/timer.h>

#include <stdlib.h>

#define NUM_ENCODERS 4

#define MOTOR_CONTROL1 MOTEUR2
#define MOTOR_CONTROL2 MOTEUR3
#define MOTOR_CONTROL3 MOTEUR4

#define NUM_MOTORS_CONTROL 3

//Paramètre de l'asservissement

	#define control_refresh 5000

//Modèle du MOTOR1
	
	#define TAU 0
	#define KP1 0.22
	#define KP2 0.22
	#define KP3 0.22
	#define KP4 0.22
	
//Paramètres du PID du MOTOR1

	#define KP 100
	#define KI 0
	#define KD 0
	
//Paramètres des moteurs

	#define REDUCTION 51 //51:1
	
	#define GAINENCODER2 100
	#define GAINENCODER 360
	
//Paramètres du robot

	#define R 0.052
	#define L1 0.1774
	#define L2 0.1774
	#define L3 0.1774
	
	
	
//Def d'une structure encodeur

typedef struct {

	int64_t globalCounter; //Somme toutes les impulsions pour connaître la position de la roue
	uint8_t direction; //Indique le sens et la direction du moteur
	float speed;	//Indique la vitesse du moteur
	int32_t previousCounter; //Indique la valeur précédente du compteur issu de l'hardware
	int64_t desiredPosition; //Vector contenant la rampe suivie par la roue
	int64_t startPosition;
	int rampePosition;
	unsigned short busy;
	
} encoder_status;

typedef struct {

	float xmax;
	float vXmax;
	float ax;
	float ymax;
	float vYmax;
	float ay;
	float Tmax;
	float vTmax;
	float aT;
	float vitesse;
	float acceleration;
	float theta;
	int asserPosition;
	float Pprevious;
	
	//Position du robot sur la table
	float X;
	float Y;
	float T;
	float Tprevious[3];
	
	float Xstart;
	float Ystart;
	float Tstart;
	
	int busy;
	
} robot_control;

robot_control holonome;
encoder_status encoders_status[4];
void NORETURN control_process(void);

void controlInit(void);
void refreshEncoderStatus(unsigned short encoder);
void controlMotors(unsigned short motors);
int64_t convertRadInImpulsions( float desiredPosition, int i);
float convertImpulsionsInRad(int64_t actualImpulsions,int i );
void generateRampeMotor(unsigned short motors, float finalThetaDesired, float vmax, float a);
void generateRampeRobot(void);
void generateRampeRobotPlan(void);

#endif
