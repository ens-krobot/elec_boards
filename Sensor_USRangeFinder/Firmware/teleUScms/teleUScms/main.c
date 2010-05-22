//----------------------------------------------------------------------------
// C main line
//----------------------------------------------------------------------------

#pragma interrupt_handler porteuse_int
#pragma interrupt_handler declencheur_int
#pragma interrupt_handler tempsMort_int
#pragma interrupt_handler compteurCapture_int
#pragma interrupt_handler compteurFin_int
#pragma interrupt_handler bufComp_int

#include <m8c.h>        // part specific constants and macros
#include "PSoCAPI.h"    // PSoC API definitions for all User Modules
#include "delay.h"

#define NBR_SALVE 8
#define BUF_RD_SIZE 2
#define BUF_WR_SIZE 1

BYTE rdBuffer[BUF_RD_SIZE], wrBuffer[BUF_WR_SIZE];

void porteuse_int(void)
{
	static unsigned char salve_cnt = 0;
	
	salve_cnt ++;
	
	if (salve_cnt >= NBR_SALVE) {
		salve_cnt = 0;
		porteuse_Stop();
		tempsMort_Start();
		compteur_Start();
	}
}

void tempsMort_int(void)
{
	//bufComp_EnableInt();
	//comp_Start(comp_HIGHPOWER);//****************************
	tempsMort_Stop();
	tempsMort_WritePeriod(120);
}

void declencheur_int(void)
{
	//PRT2DR = PRT2DR & 0xF7;
	//porteuse_Start();
}

void compteurFin_int(void)
{
	// On a atteint la valeur de comparaison, on a donc pas détecté le retour de l'onde
	// On renvoie donc un résultat nul
	//comp_Stop();//***************************
	compteur_Stop();
	compteur_WritePeriod(60000);
	rdBuffer[0] = 0;
	rdBuffer[1] = 0;
	//PRT2DR = PRT2DR | 0x08;
}

void compteurCapture_int(void)
{
	// Le comparateur a commuté, on a donc détecté l'onde réfléchie
	WORD compt;
	compt = 60000 - (long)compteur_wReadCompareValue();
	if (compt <= 120)
		return;
	//comp_Stop();//****************************
	compteur_Stop();
	compteur_WritePeriod(60000);
	rdBuffer[0] = compt/256;
	rdBuffer[1] = (compt & 0x00FF);
	PRT2DR = PRT2DR | 0x08;
}

void bufComp_int(void)
{
	// Le comparateur a commuté, on a donc détecté l'onde réfléchie
	WORD compt;
	bufComp_DisableInt();
	compt = 60000 - (long)compteur_wReadTimer();
	rdBuffer[0] = compt/256;
	rdBuffer[1] = (compt & 0x00FF);
	
	
}

void init(void)
{
	BYTE i, j;
	
	// On active l'I2C en mode esclave
	I2CHW_Start();
	I2CHW_EnableSlave();
	I2CHW_EnableInt();
	
	// Définition des buffer de réception/émission de l'I2C
	I2CHW_InitRamRead(rdBuffer,BUF_RD_SIZE);
	I2CHW_InitWrite(wrBuffer,BUF_WR_SIZE);

	porteuse_EnableInt();
	inverseur_Start();
	declencheur_EnableInt();
	declencheur_Start();
	
	PGA_1_Start(PGA_1_HIGHPOWER);
	PGA_2_Start(PGA_2_HIGHPOWER);
	PGA_3_Start(PGA_3_HIGHPOWER);
	filtre1_Start(filtre1_HIGHPOWER);
	filtre2_Start(filtre2_HIGHPOWER);
	comp_Start(comp_HIGHPOWER);//*******
	
	bufComp_Start();
	compteur_EnableInt();
	tempsMort_EnableInt();
	
	// On active les interruptions globales
	M8C_EnableGInt;
	
	// On fait clignotter la LED pour indiquer qu'on a démarré en indiquant l'adresse
	for (j = 0; j < 5; j++) {
		PRT2DR = PRT2DR | 0x08;
		for (i=0; i< 10; i++)
			Delay50uTimes(200);
		PRT2DR = PRT2DR & 0xF7;
		for (i=0; i< 10; i++)
			Delay50uTimes(200);
	}
	PRT2DR = PRT2DR | 0x08;
	for (i=0; i< 10; i++)
		Delay50uTimes(200);	
	for (i=0; i< 10; i++)
		Delay50uTimes(200);
	PRT2DR = PRT2DR & 0xF7;
}

void main(void)
{
	BYTE status;
	BYTE ledStatus = 0;
	init();
	
    for (;;) {
		status = I2CHW_bReadI2CStatus();
		// On attend des données en provenance du maître
		if( status & I2CHW_WR_COMPLETE )
		{
			// traitement de la commande reçue
			switch(wrBuffer[0]) {
				case 'b':
					// blink
					/*if (ledStatus) {
						PRT2DR = PRT2DR & 0xF7;
						ledStatus = 0;
					} else {
						PRT2DR = PRT2DR | 0x08;
						ledStatus = 1;
					}*/
					PRT2DR = PRT2DR & 0xF7;
					porteuse_Start();
					break;
				case 'g':
					// Récupère d'une mesure
					//rdBuffer[0] = 'X';
					//rdBuffer[1] = 'L';
					break;
				default:
					break;
			}
			// On efface le drapeau d'écriture
			I2CHW_ClrWrStatus();
			// On réinitialise le buffer pour la prochaine écriture
			I2CHW_InitWrite(wrBuffer,BUF_WR_SIZE);
		}
		// Attend la fin de la lecture des données par le maître
		if( status & I2CHW_RD_COMPLETE )
		{
			// On efface le drapeau de lecture
			I2CHW_ClrRdStatus();
			// On réinitialise le buffer pour la prochaine lecture
			rdBuffer[0] = rdBuffer[1] = 0;
			PRT2DR = PRT2DR & 0xF7;
			I2CHW_InitRamRead(rdBuffer,BUF_RD_SIZE);
		}
	}
}
