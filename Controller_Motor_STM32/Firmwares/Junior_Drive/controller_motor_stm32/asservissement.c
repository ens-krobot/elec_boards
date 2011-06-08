#include "asservissement.h"
#include "encoder.h"
#include "motor.h"
#include "odometry.h"

#include <cfg/macros.h>
#include <drv/gpio_stm32.h>
#include "stm32lib/stm32f10x_tim.h"
#include "stm32lib/stm32f10x_rcc.h"
#include "stm32lib/stm32f10x.h"
#include <math.h>






unsigned short def_encoders[4]={ENCODER1, ENCODER2, ENCODER3, ENCODER4};
float Kp[NUM_MOTORS]={KP1,KP2,KP3,KP4};

PROC_DEFINE_STACK(stack_control, KERN_MINSTACKSIZE * 4);


void controlInit(void)  {
	int i;
	
	
	for(i=0;i<NUM_ENCODERS;i++)
		{
			encoders_status[i].globalCounter=0;
			encoders_status[i].direction=0;
			encoders_status[i].speed=0;
			encoders_status[i].rampePosition=0;
			encoders_status[i].desiredPosition=0;
			encoders_status[i].startPosition=0;
			encoders_status[i].previousCounter=getEncoderCount(1<<i);
			encoders_status[i].busy=0;
		}
	/* Create a new child process */
        
        proc_new(control_process, NULL, sizeof(stack_control), stack_control);
        
        
	return;
}

void NORETURN control_process(void)
{
  Timer timer_control_process;
  int64_t desiredImpulsions[NUM_MOTORS];
  int i;
  float desiredPositions[NUM_MOTORS]={0,2*M_PI,2*M_PI,2*M_PI};
  timer_setDelay(&timer_control_process, us_to_ticks((utime_t)(control_refresh)));
  timer_setEvent(&timer_control_process);
  
  enableMotors( MOTEUR4|MOTEUR2|MOTEUR3);
  //convertRadInImpulsions(desiredPositions, desiredImpulsions);
  
  
  while(1) {
  	
  	timer_add(&timer_control_process);// Start process timer
  	/*for(i=0;i<NUM_MOTORS;i++)
  		{
  			desiredPositions[i]+=0.05;
  			if (desiredPositions[i]>(2*M_PI)) desiredPositions[i]=2*M_PI;
  		}
  	convertRadInImpulsions(desiredPositions, desiredImpulsions);*/
  	//generateRampeMotor(MOTEUR4|MOTEUR2|MOTEUR3,-20*2*M_PI,10,5);
  	generateRampeRobotPlan();
  	controlMotors(MOTEUR4|MOTEUR2|MOTEUR3);	

    timer_waitEvent(&timer_control_process); // Wait until the end of counting
  }
  
   
}

void refreshEncoderStatus(unsigned short encoder)  {
	int i;
	int64_t difference;
	int32_t currentCounter;
	for(i=0;i<NUM_ENCODERS;i++)  {
		if(encoder & (1<<i))  {
			/* Def direction*/
				encoders_status[i].direction = getEncoderDirection(def_encoders[i]);
			/* Calcul du compteur global*/
				currentCounter =(int32_t)(getEncoderCount(def_encoders[i]));
				difference = (int64_t)(currentCounter-encoders_status[i].previousCounter); 
				
				if( encoders_status[i].direction == FORWARD_DIRECTION)
					{
						if(difference < 0) 
							{
								difference += 65535;
							}
					}
				else if(difference > 0) difference -= 65535;
				encoders_status[i].globalCounter += difference;		
				encoders_status[i].previousCounter = currentCounter;
			/* Speed Calcul*/
		}
	}
	return;
}



void controlMotors(unsigned short motor)  {

	int64_t error;
	int32_t PWM;
	int i;
	
	//refreshEncoderStatus(ENCODER1 | ENCODER2 | ENCODER3 | ENCODER4);
	for(i=0;i<NUM_MOTORS;i++)
		{
			if(motor & (1<<i))  {  
				//Conversion en de desiredImpulsions en nombre d'impulsions
				
				
				error = encoders_status[i].desiredPosition - encoders_status[i].globalCounter; 
				PWM = (int32_t)(error*Kp[i]);
				
				setVelocity(1<<i, PWM);
				encoders_status[i].rampePosition+=1;
			}
		}
	holonome.asserPosition+=1;
	return;
}

int64_t convertRadInImpulsions(float actualPosition,int i )  {
	int64_t actualImpulsions;
	
		actualImpulsions = (int64_t)(actualPosition*REDUCTION*GAINENCODER*4/(2*M_PI));
	
	return actualImpulsions;
}

float convertImpulsionsInRad(int64_t actualImpulsions,int i )  {
	float actualPosition;
	
		actualPosition = (actualImpulsions/(REDUCTION*GAINENCODER*4/(2*M_PI)));
	
	return actualPosition;
}

void generateRampeMotor(unsigned short motor, float thetaMax, float vmax, float a)  {
	int i,k;
	//Def des variables nécessaires à la générations des rampes
		float tmax=0; //Défintion du temps final de fin de rampe
		float T = control_refresh/1000000.0; //Défintion de la période d'échantillonnage
		float t; //Défini le temps discret
		int N=0; //Nombre de points dans le vecteur de rampe
		float tvmax=vmax/a; //Temps où l'on atteint la vitesse max
		float thetaVmax=0;
		float theta;
		float t1, t2; //Temps pour les trapèzes
		float thetaDesired;
		unsigned short signe;
		
	int64_t position;
	int64_t actualPosition;
		
	for(i=0;i<NUM_MOTORS;i++)  {
		if(motor & 1<<i)  {	
			k=encoders_status[i].rampePosition;
			if(k==0) encoders_status[i].startPosition=encoders_status[i].globalCounter;
			actualPosition=encoders_status[i].startPosition;
			thetaDesired = thetaMax - convertImpulsionsInRad(actualPosition,i);
			if(thetaDesired<0)  {
				signe=1; //On désire aller à une position antérieure
				thetaDesired = - thetaDesired;
			}
			
			//Choix du profil : triangulaire ou trapézoïdal
				thetaVmax = a*tvmax*tvmax;
				if(thetaVmax>thetaDesired)  {
					//On génère un profil triangulaire
						tmax = 2*sqrt(thetaDesired/a);
						
						N=(int)(floorf(tmax/T))+1;
						
						if(N*T < tmax) N+=1; //On oublie forcément le point final
						//Définition des CI et CF
						t=(float)(k)*T;
						
						if(t<=tmax/2)  {
							//On est dans la première partie de la courbe
							theta=a/2*t*t;
						}
						else  {
							if(t<=tmax) {
								//On est dans la deuxième partie de la coube
								theta=-a/2*t*t+a*tmax*t+a*(tmax/2)*(tmax/2)-a/2*tmax*tmax;
							}
							else {
								theta = thetaDesired;
							}
						}
						
						position=convertRadInImpulsions(theta,i);
						
				}
				else  {
					//On génère un profil trapézoïdal
						t1=vmax/a;
						tmax = thetaDesired/vmax+t1;
						t2 = tmax - t1;
						N =(int)(floorf(tmax/T))+1;
						if(N*T < tmax) N+=1;
						
						t=k*T;
						
						if(t<=t1)  {
							//On est dans la première partie de la courbe
							theta=a/2*t*t;
						}
						else  {
							if(t<=t2)  {
								// On est dans la deuxième partie de la courbe
								theta=a/2*t1*t1+vmax*(t-t1);
							}
							else  {
								if(t<=tmax)  {
									//On est dans la troisième partie de la coube
									theta=-a/2*t*t+a*tmax*(t-t2)+a/2*(t1*t1+t2*t2)+vmax*(t2-t1);
								}
								else  {
									theta = thetaDesired;
								}
							}
							
						}
						position=convertRadInImpulsions(theta,i);
						
				}
				
			//On doit maintenant convertir et incrémenter vis-à-vis de la position actuelle
			
				if(signe==1) position*=-1;
				position+=actualPosition;
				encoders_status[i].desiredPosition = position;
				
		}
		
	}
	return;
}

void generateRampeRobot(void)  {
	int i,k;
	//Def des variables nécessaires à la générations des rampes
		float Pmax,vmax,a;
		float P,V;
		float Pvmax,tvmax,tmax;
		float T = control_refresh/1000000.0; //Défintion de la période d'échantillonnage
		float t; //Défini le temps discret
		int N=0; //Nombre de points dans le vecteur de rampe
		float t1, t2; //Temps pour les trapèzes
		float desiredP;
		float Pstart;
		float Ti[3]={0,0,0};
		float w[3]={0,0,0};
		unsigned short signe;
		
	int64_t position;
	int64_t actualPosition;
	
	k=holonome.asserPosition;
	
	
	if(k==0) {
		holonome.Xstart = holonome.X;
		holonome.Ystart = holonome.Y;
		holonome.Tstart = holonome.T;
		
		for(i=1;i<NUM_MOTORS;i++)
			{
				encoders_status[i].startPosition=encoders_status[i].globalCounter;
			}
	}
	
	for(i=0;i<3;i++)  {
		//Récolte des données
			switch (i) {
				case 0:
						// X
						Pmax = holonome.xmax;
						vmax = holonome.vXmax;
						a = holonome.ax;
						Pstart = holonome.Xstart;
						break;
				case 1:
						// Y
						Pmax = holonome.ymax;
						vmax = holonome.vYmax;
						a = holonome.ay;
						Pstart = holonome.Ystart;
						break;
				case 2:
						// T
						Pmax = holonome.Tmax;
						vmax = holonome.vTmax;
						a = holonome.aT;
						Pstart = holonome.Tstart;
						break;
			}
		desiredP=Pmax-Pstart;
		if(desiredP<0)  {
			signe = 1;
			desiredP = -desiredP;
		}	
		//Choix du profil : triangulaire ou trapézoïdal
		tvmax = vmax/a;
		Pvmax = a*tvmax*tvmax;	
		if(Pvmax>desiredP) {
			//On génère un profil triangulaire
			tmax = 2*sqrt(desiredP/a);
			N=(int)(floorf(tmax/T))+1;
			if(N*T < tmax) N+=1; //On oublie forcément le point final
			//Définition des CI et CF
			t=(float)(k)*T;
			
			if(t<=tmax/2)  {
				//On est dans la première partie de la courbe
				P=a/2*t*t;
				V=a*t;
			}
			else  {
				if(t<=tmax) {
					//On est dans la deuxième partie de la coube
					P=-a/2*t*t+a*tmax*t+a*(tmax/2)*(tmax/2)-a/2*tmax*tmax;
					V=-a*t+a*tmax;
				}
				else {
					P = desiredP;
					V=0;
				}
			}
		}
		else  {
			//On génère un profil trapézoïdal
			t1=vmax/a;
			tmax = desiredP/vmax+t1;
			t2 = tmax - t1;
			N =(int)(floorf(tmax/T))+1;
			if(N*T < tmax) N+=1;
			
			t=k*T;
			
			if(t<=t1)  {
				//On est dans la première partie de la courbe
				P=a/2*t*t;
				V=a*t;
			}
			else  {
				if(t<=t2)  {
					// On est dans la deuxième partie de la courbe
					P=a/2*t1*t1+vmax*(t-t1);
					V=vmax;
				}
				else  {
					if(t<=tmax)  {
						//On est dans la troisième partie de la coube
						P=-a/2*t*t+a*tmax*(t-t2)+a/2*(t1*t1+t2*t2)+vmax*(t2-t1);
						V=-a*t+a*tmax;
					}
					else  {
						P = desiredP;
						V=0;
					}
				}
				
			}
		}	
		if(signe==1) V=-V;		
		switch (i) {
			case 0:
					// X
					w[0] = 0;
					Ti[0]+=0;
					w[1]=(V*sqrt(3)/2)/R;
					Ti[1] += w[1]*T;
					w[2]=-w[1];
					Ti[2] += w[2]*T;
					break;
			case 1:
					// Y
					w[0] = -V/R; // -
					Ti[0] += w[0]*T;
					w[1]= +V/(2*R); //+
					Ti[1] += w[1]*T;
					w[2]=w[1];
					Ti[2] += w[2]*T;
					break;
			case 2:
					// T
					w[0] = -V*L1/R;
					Ti[0] += w[0]*T;
					w[1]= -V*L2/R;
					Ti[1] += w[1]*T;
					w[2]=-V*L3/R;;
					Ti[2] += w[2]*T;
					break;
		}				
		
			
		
	
			
	}
	for(i=0;i<NUM_MOTORS-1;i++)
		{
			Ti[i]+= holonome.Tprevious[i];
			holonome.Tprevious[i] = Ti[i];
		}
	
	for(i=1;i<NUM_MOTORS;i++)
		{
			//Moteur i	
				actualPosition = encoders_status[i].startPosition;
				position = convertRadInImpulsions(Ti[i-1],i);
				
				position+=actualPosition;
				
				encoders_status[i].desiredPosition = position;
		}
	return;
}

void generateRampeRobotPlan()  {
	int i,k;
	//Def des variables nécessaires à la générations des rampes
		float xmax, ymax,vmax,a;
		float P,V;
		float xVirtuel, yVirtuel;
		float Vx, Vy, theta;
		float Pvmax,tvmax,tmax;
		float T = control_refresh/1000000.0; //Défintion de la période d'échantillonnage
		float t; //Défini le temps discret
		int N=0; //Nombre de points dans le vecteur de rampe
		float t1, t2; //Temps pour les trapèzes
		float desiredP;
		float Xstart, Ystart, thetaRobot, Tstart;
		float Ti[3]={0,0,0};
		float w[3]={0,0,0};
		
	int64_t position;
	int64_t actualPosition;
	
	k=holonome.asserPosition;
	
	if(k!=-1) {
		if(k==0) {
			holonome.Xstart = holonome.X;
			holonome.Ystart = holonome.Y;
			holonome.Tstart = holonome.T;
			xmax = holonome.xmax;
			ymax = holonome.ymax;
		
			Xstart = holonome.Xstart;
			Ystart = holonome.Ystart;
		
			if((ymax-Ystart)==0) {
				if((xmax-Xstart)<0) holonome.theta=M_PI;
				else holonome.theta=0;
			}
			else holonome.theta = 2*atan((ymax-Ystart)/((xmax-Xstart)+sqrt((xmax-Xstart)*(xmax-Xstart)+(ymax-Ystart)*(ymax-Ystart))));
		
			for(i=0; i<3;i++)  {
				holonome.Tprevious[i]=0;
			}
		
			holonome.Pprevious = 0;
		
			for(i=1;i<NUM_MOTORS;i++)
				{
					encoders_status[i].startPosition=encoders_status[i].globalCounter;
				}
		}
	

		//Récolte des données
		xmax = holonome.xmax;
		ymax = holonome.ymax;
		vmax = holonome.vitesse;
		a = holonome.acceleration;
		Xstart = holonome.Xstart;
		Ystart = holonome.Ystart;
		theta = holonome.theta;
		thetaRobot = holonome_odometry.T;
	
		desiredP=sqrt((xmax-Xstart)*(xmax-Xstart)+(ymax-Ystart)*(ymax-Ystart));
	
		//Choix du profil : triangulaire ou trapézoïdal
		tvmax = vmax/a;
		Pvmax = a*tvmax*tvmax;	
		if(Pvmax>desiredP) {
			//On génère un profil triangulaire
			tmax = 2*sqrt(desiredP/a);
			N=(int)(floorf(tmax/T))+1;
			if(N*T < tmax) N+=1; //On oublie forcément le point final
			//Définition des CI et CF
			t=(float)(k)*T;
		
			if(t<=tmax/2)  {
				//On est dans la première partie de la courbe
				P=a/2*t*t;
				V=a*t;
			}
			else  {
				if(t<=tmax) {
					//On est dans la deuxième partie de la coube
					P=-a/2*t*t+a*tmax*t+a*(tmax/2)*(tmax/2)-a/2*tmax*tmax;
					V=-a*t+a*tmax;
				}
				else {
					P = desiredP;
					V=0;
					if(obstacle==0)  {
						holonome.busy=0;
					}
				}
			}
		}
		else  {
			//On génère un profil trapézoïdal
			t1=vmax/a;
			tmax = desiredP/vmax+t1;
			t2 = tmax - t1;
			N =(int)(floorf(tmax/T))+1;
			if(N*T < tmax) N+=1;
		
			t=k*T;
		
			if(t<=t1)  {
				//On est dans la première partie de la courbe
				P=a/2*t*t;
				V=a*t;
			}
			else  {
				if(t<=t2)  {
					// On est dans la deuxième partie de la courbe
					P=a/2*t1*t1+vmax*(t-t1);
					V=vmax;
				}
				else  {
					if(t<=tmax)  {
						//On est dans la troisième partie de la coube
						P=-a/2*t*t+a*tmax*(t-t2)+a/2*(t1*t1+t2*t2)+vmax*(t2-t1);
						V=-a*t+a*tmax;
					}
					else  {
						P = desiredP;
						V=0;
						if(obstacle==0)  {
							holonome.busy=0;
						}
					}
				}
			
			}
		}	
	
		Vx = V*cos(theta-thetaRobot);
		Vy = V*sin(theta-thetaRobot);
	
		//Ajout des composantes de déviations
	
			xVirtuel = cos(theta)*holonome.Pprevious;
			yVirtuel = sin(theta)*holonome.Pprevious; // dans la base de la table
		
			//holonome.Pprevious = P;
			//Vx += ((xVirtuel+Xstart-holonome_odometry.X)*cos(thetaRobot)-(yVirtuel+Ystart-holonome_odometry.Y)*sin(thetaRobot))/(5*T);
			//Vy += ((yVirtuel+Ystart-holonome_odometry.Y)*cos(thetaRobot)+(xVirtuel+Xstart-holonome_odometry.X)*sin(thetaRobot))/(5*T);
	
		w[0] = (-Vy)/R;
		w[1] = (Vx*sqrt(3)/2+Vy/2)/R;
		w[2] = (-Vx*sqrt(3)/2+Vy/2)/R;
	
	
		
		for(i=0;i<3;i++)  {
			Ti[i]=w[i]*T;
		}
			
			
		for(i=0;i<NUM_MOTORS-1;i++)
			{
				Ti[i]+= holonome.Tprevious[i];
				holonome.Tprevious[i] = Ti[i];
			}
	
		for(i=1;i<NUM_MOTORS;i++)
			{
				//Moteur i	
					actualPosition = encoders_status[i].startPosition;
					position = convertRadInImpulsions(Ti[i-1],i);
				
					position+=actualPosition;
				
					encoders_status[i].desiredPosition = position;
			}
	}
	else {
		for(i=0;i<NUM_MOTORS-1;i++)
			{
				Ti[i]= holonome.Tprevious[i];
				//holonome.Tprevious[i] = Ti[i];
			}
	
		for(i=1;i<NUM_MOTORS;i++)
			{
				//Moteur i	

					position = convertRadInImpulsions(Ti[i-1],i);
				
					encoders_status[i].desiredPosition = position;
			}
	}
	return;
}

/*
void generateRampeRobotPlan()  {
	int i,k;
	//Def des variables nécessaires à la générations des rampes
		float xmax, ymax,vmax,a;
		float P,V;
		float Vx, Vy, theta;
		float Pvmax,tvmax,tmax;
		float T = control_refresh/1000000.0; //Défintion de la période d'échantillonnage
		float t; //Défini le temps discret
		int N=0; //Nombre de points dans le vecteur de rampe
		float t1, t2; //Temps pour les trapèzes
		float desiredP;
		float Xstart, Ystart;
		float Ti[3]={0,0,0};
		float w[3]={0,0,0};
		
	int64_t position;
	int64_t actualPosition;
	
	k=holonome.asserPosition;
	
	
	if(k==0) {
		holonome.Xstart = holonome.X;
		holonome.Ystart = holonome.Y;
		holonome.Tstart = holonome.T;
		xmax = holonome.xmax;
		ymax = holonome.ymax;
		
		Xstart = holonome.Xstart;
		Ystart = holonome.Ystart;
		
		if((ymax-Ystart)==0) {
			if((xmax-Xstart)<0) holonome.theta=M_PI;
			else holonome.theta=0;
		}
		else holonome.theta = 2*atan((ymax-Ystart)/((xmax-Xstart)+sqrt((xmax-Xstart)*(xmax-Xstart)+(ymax-Ystart)*(ymax-Ystart))));
		
		for(i=0; i<3;i++)  {
			holonome.Tprevious[i]=0;
		}
		
		
		for(i=1;i<NUM_MOTORS;i++)
			{
				encoders_status[i].startPosition=encoders_status[i].globalCounter;
			}
	}
	

	//Récolte des données
	xmax = holonome.xmax;
	ymax = holonome.ymax;
	vmax = holonome.vitesse;
	a = holonome.acceleration;
	Xstart = holonome.Xstart;
	Ystart = holonome.Ystart;
	theta = holonome.theta;
	
	desiredP=sqrt((xmax-Xstart)*(xmax-Xstart)+(ymax-Ystart)*(ymax-Ystart));
	
	//Choix du profil : triangulaire ou trapézoïdal
	tvmax = vmax/a;
	Pvmax = a*tvmax*tvmax;	
	if(Pvmax>desiredP) {
		//On génère un profil triangulaire
		tmax = 2*sqrt(desiredP/a);
		N=(int)(floorf(tmax/T))+1;
		if(N*T < tmax) N+=1; //On oublie forcément le point final
		//Définition des CI et CF
		t=(float)(k)*T;
		
		if(t<=tmax/2)  {
			//On est dans la première partie de la courbe
			P=a/2*t*t;
			V=a*t;
		}
		else  {
			if(t<=tmax) {
				//On est dans la deuxième partie de la coube
				P=-a/2*t*t+a*tmax*t+a*(tmax/2)*(tmax/2)-a/2*tmax*tmax;
				V=-a*t+a*tmax;
			}
			else {
				P = desiredP;
				V=0;
				holonome.busy=0;
			}
		}
	}
	else  {
		//On génère un profil trapézoïdal
		t1=vmax/a;
		tmax = desiredP/vmax+t1;
		t2 = tmax - t1;
		N =(int)(floorf(tmax/T))+1;
		if(N*T < tmax) N+=1;
		
		t=k*T;
		
		if(t<=t1)  {
			//On est dans la première partie de la courbe
			P=a/2*t*t;
			V=a*t;
		}
		else  {
			if(t<=t2)  {
				// On est dans la deuxième partie de la courbe
				P=a/2*t1*t1+vmax*(t-t1);
				V=vmax;
			}
			else  {
				if(t<=tmax)  {
					//On est dans la troisième partie de la coube
					P=-a/2*t*t+a*tmax*(t-t2)+a/2*(t1*t1+t2*t2)+vmax*(t2-t1);
					V=-a*t+a*tmax;
				}
				else  {
					P = desiredP;
					V=0;
					holonome.busy=0;
				}
			}
			
		}
	}	
	
	Vx = V*cos(theta);
	Vy = V*sin(theta);
	
	
	w[0] = (-Vy)/R;
	w[1] = (Vx*sqrt(3)/2+Vy/2)/R;
	w[2] = (-Vx*sqrt(3)/2+Vy/2)/R;
	
	
		
	for(i=0;i<3;i++)  {
		Ti[i]=w[i]*T;
	}
			
			
	for(i=0;i<NUM_MOTORS-1;i++)
		{
			Ti[i]+= holonome.Tprevious[i];
			holonome.Tprevious[i] = Ti[i];
		}
	
	for(i=1;i<NUM_MOTORS;i++)
		{
			//Moteur i	
				actualPosition = encoders_status[i].startPosition;
				position = convertRadInImpulsions(Ti[i-1],i);
				
				position+=actualPosition;
				
				encoders_status[i].desiredPosition = position;
		}
	return;
}*/
