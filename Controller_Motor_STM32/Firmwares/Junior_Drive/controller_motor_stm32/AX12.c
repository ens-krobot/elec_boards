#include "AX12.h"


void AX12Init()  {
        timer_delay(1000);
	ax1.address=(uint8_t) 1;
	ax1.position=(uint16_t) DEFAULT_AX1_POSITION;
	ax1.speed=(uint16_t) DEFAULT_SPEED;
	
	ax2.address=(uint8_t) 2;
	ax2.position=(uint16_t) DEFAULT_AX2_POSITION;
	ax2.speed=(uint16_t) DEFAULT_SPEED;
	return;
}

void ouvrir_pinces()  {

	ax1.position=(uint16_t) OPEN_AX1_POSITION;
	ax2.position=(uint16_t) OPEN_AX2_POSITION;
	timer_delay(500);
	return;
}

void fermer_pinces()  {

	ax1.position=(uint16_t) DEFAULT_AX1_POSITION;
	ax2.position=(uint16_t) DEFAULT_AX2_POSITION;
	timer_delay(500);
	return;
}

void serrer_palet()  {
	
	ax1.position=(uint16_t) CATCH_AX1_POSITION;
	ax2.position=(uint16_t) CATCH_AX2_POSITION;
	timer_delay(500);
	return;
}
