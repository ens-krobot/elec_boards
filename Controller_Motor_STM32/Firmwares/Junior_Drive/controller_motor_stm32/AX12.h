#ifndef HEADER__AX12
#define HEADER__AX12

#define DEFAULT_AX1_POSITION 120
#define DEFAULT_AX2_POSITION 853

#define OPEN_AX1_POSITION 500
#define OPEN_AX2_POSITION 453

#define CATCH_AX1_POSITION 335
#define CATCH_AX2_POSITION 648

#define DEFAULT_SPEED 300

#include <stdint.h>
#include "can_messages_sensors.h"
#include <drv/timer.h>

// Def of variables
typedef struct {
	uint8_t address;
	uint16_t position;
	uint16_t speed;
    
} ax12_def;	

ax12_def ax1, ax2;
	
void AX12Init(void);
void ouvrir_pinces(void);
void fermer_pinces(void);
void serrer_palet(void);

#endif
