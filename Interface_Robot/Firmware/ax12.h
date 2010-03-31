/**
 * @file ax12.h
*/

#ifndef AX12_H
#define AX12_H

#include <usart.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "error.h"

#define DIR_PORT                    PORTDbits.RD5

// --- Control Table Address ---
// EEPROM area
#define P_MODEL_NUMBER              0
#define P_MODEL_NUMBER_L            0
#define P_MODEL_NUMBER_H            1
#define P_VERSION                   2
#define P_ID                        3
#define P_BAUD_RATE                 4
#define P_RETURN_DELAY_TIME         5
#define P_CW_ANGLE_LIMIT            6
#define P_CW_ANGLE_LIMIT_L          6
#define P_CW_ANGLE_LIMIT_H          7
#define P_CCW_ANGLE_LIMIT           8
#define P_CCW_ANGLE_LIMIT_L         8
#define P_CCW_ANGLE_LIMIT_H         9
#define P_SYSTEM_DATA2              10
#define P_LIMIT_TEMPERATURE         11
#define P_DOWN_LIMIT_VOLTAGE        12
#define P_UP_LIMIT_VOLTAGE          13
#define P_MAX_TORQUE                14
#define P_MAX_TORQUE_L              14
#define P_MAX_TORQUE_H              15
#define P_RETURN_LEVEL              16
#define P_ALARM_LED                 17
#define P_ALARM_SHUTDOWN            18
#define P_OPERATING_MODE            19
#define P_DOWN_CALIBRATION          20
#define P_DOWN_CALIBRATION_L        20
#define P_DOWN_CALIBRATION_H        21
#define P_UP_CALIBRATION            22
#define P_UP_CALIBRATION_L          22
#define P_UP_CALIBRATION_H          23

// RAM area
#define P_TORQUE_ENABLE             24
#define P_LED                       25
#define P_CW_COMPLIANCE_MARGIN      26
#define P_CCW_COMPLIANCE_MARGIN     27
#define P_CW_COMPLIANCE_SLOPE       28
#define P_CCW_COMPLIANCE_SLOPE      29
#define P_GOAL_POSITION             30
#define P_GOAL_POSITION_L           30
#define P_GOAL_POSITION_H           31
#define P_GOAL_SPEED                32
#define P_GOAL_SPEED_L              32
#define P_GOAL_SPEED_H              33
#define P_TORQUE_LIMIT              34
#define P_TORQUE_LIMIT_L            34
#define P_TORQUE_LIMIT_H            35
#define P_PRESENT_POSITION          36
#define P_PRESENT_POSITION_L        36
#define P_PRESENT_POSITION_H        37
#define P_PRESENT_SPEED             38
#define P_PRESENT_SPEED_L           38
#define P_PRESENT_SPEED_H           39
#define P_PRESENT_LOAD              40
#define P_PRESENT_LOAD_L            40
#define P_PRESENT_LOAD_H            41
#define P_PRESENT_VOLTAGE           42
#define P_PRESENT_TEMPERATURE       43
#define P_REGISTERED_INSTRUCTION    44
#define P_PAUSE_TIME                45
#define P_MOVING                    46
#define P_LOCK                      47
#define P_PUNCH                     48
#define P_PUNCH_L                   48
#define P_PUNCH_H                   49

// --- Instruction ---
#define INST_PING                   0x01
#define INST_READ                   0x02
#define INST_WRITE                  0x03
#define INST_REG_WRITE              0x04
#define INST_ACTION                 0x05
#define INST_RESET                  0x06
#define INST_DIGITAL_RESET          0x07
#define INST_SYSTEM_READ            0x0C
#define INST_SYSTEM_WRITE           0x0D
#define INST_SYNC_WRITE             0x83
#define INST_SYNC_REG_WRITE         0x84

// --- Error ---
#define ERR_INPUT_VOLTAGE           1
#define ERR_ANGLE_LIMIT             2
#define ERR_OVERHEATING             4
#define ERR_RANGE                   8
#define ERR_CHECKSUM                16
#define ERR_OVERLOAD                32
#define ERR_INSTRUCTION             64

// --- Valeurs specifiques ---
#define ID_BROADCAST                0xFE

// Fonctions bas niveau
void initAX12(void);
void interruptAX12(void);
void sendInstPacket(BYTE id, BYTE inst, BYTE paramLength);
BOOL isStatusPacketReceived(void);
BOOL hasTimedOut(void);
void waitStatusPacket(void);
void setParam(BYTE index, BYTE value);
void configAX12(BYTE newId);

// Fonctions haut niveau
BYTE lastErrorAX12(void);

BOOL pingAX12(BYTE id, UINT timeOut);
void resetAX12(BYTE id);

void writeValue8(BYTE id, BYTE reg, BYTE value);
void writeValue16(BYTE id, BYTE reg, WORD value);

BYTE readValue8(BYTE id, BYTE reg, UINT timeOut);
WORD readValue16(BYTE id, BYTE reg, UINT timeOut);

void regWrite8(BYTE id, BYTE reg, BYTE value);
void regWrite16(BYTE id, BYTE reg, WORD value);
void actionAX12(BYTE id);

// Helper
void goTo(BYTE id, WORD position, WORD speed, BYTE mode);
int getPosition(BYTE id, UINT timeOut);
int getSpeed(BYTE id, UINT timeOut);

#endif
