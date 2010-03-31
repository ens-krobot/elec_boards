/**
 * @file lm629.h
 * Gère l'interface avec les circuits intégrés LM629.
*/

#ifndef LM629_H
#define LM629_H

#include <delays.h>
#include <math.h>
#include "GenericTypeDefs.h"
#include "Compiler.h"
#include "HardwareProfile.h"
#include "PcInterface.h"
#include "eeprom.h"
#include "error.h"
#include "motor.h"

// Robot utilisé
#define KROBOT_2010
// #define KROBOT_PROTO

// Paramètres généraux
#define LM_CLOCK               8e6        ///< Fréquence horloge LM (en Hz) -- forcément 6 ou 8 MHz selon la référence du composant
//#define LM_INTERRUPT      // Non-functional, bug in the card schematic (RB2 and RB3 pins are not interruptible on change...)
#define LM_POLLING

#if defined(KROBOT_2010)
    // Constantes pour [Kro]bot 2010
    #define ENCODER_RES         15*500        ///< Impulsions du codeur par tour de roue (prendre en compte le réducteur selon le cas !)
    #define WHEELS_DIAMETER         98        ///< Diamètre des roues (en mm)
    #define WHEELS_DIST            259        ///< Distance entre les 2 roues (en mm)
    #define CON_MOTOR_LEFT          -1
    #define CON_MOTOR_RIGHT         -1
#elif defined(KROBOT_PROTO)
    // Constantes pour le robot protoype basé sur des EMG30s
    #define ENCODER_RES             90       ///< Impulsions du codeur par tour de roue (prendre en compte le réducteur selon le cas !)
    #define WHEELS_DIAMETER         99       ///< Diamètre des roues (en mm)
    #define WHEELS_DIST            220       ///< Distance entre les 2 roues (en mm)
    #define CON_MOTOR_LEFT          -1
    #define CON_MOTOR_RIGHT          1
#else
    #error Il est nécessaire de déclarer un type de robot à utiliser
#endif

/**
 * Paramètres par défault du correcteur.
 * Pour débuter, ne mettre qu'un KP, de faible valeur (5 ou 10)
 * Vérifier que le système est stable (pas d'oscillation)
 * On peut alors augmenter le KP et ajuster KI et KD.
*/
#if defined(KROBOT_2010)
    // Constantes pour [Kro]bot 2010
    #define DEFAULT_KP            1        ///< Constante action proportionnelle
    #define DEFAULT_KI            0        ///< Constante action intégrale
    #define DEFAULT_KD            0        ///< Constante action dérivée
    #define DEFAULT_IL         1000        ///< Integration Limit -- limite la contribution du terme intégrateur (doit être non nul si KI est non nul !)
    #define LM_PES_LIMIT      10000        ///< Position Error Stop Limit
#elif defined(KROBOT_PROTO)
    // Constantes pour le robot protoype basé sur des EMG30s
    #define DEFAULT_KP         1000        ///< Constante action proportionnelle
    #define DEFAULT_KI            1        ///< Constante action intégrale
    #define DEFAULT_KD            0        ///< Constante action dérivée
    #define DEFAULT_IL        10000        ///< Integration Limit -- limite la contribution du terme intégrateur (doit être non nul si KI est non nul !)
    #define LM_PES_LIMIT      10000        ///< Position Error Stop Limit
#else
    #error Il est nécessaire de déclarer un type de robot à utiliser
#endif

// Constantes
#define PS_COMMAND            0        ///< Envoyer une commande
#define PS_DATA               1        ///< Envoyer des données
#define DATA_OUT              0        ///< Envoyer des données vers le LM
#define DATA_IN               1        ///< Recevoir des données du LM

// Constantes calculées
#define PI                     3.1415926535898
#define LM_SAMPLE_TIME        (2048.0 / LM_CLOCK)
#define CONST_POS             ((PI / 180.0) / 2.0)
#define CONST_VEL             (65536.0 * LM_SAMPLE_TIME)
#define CONST_ACC             (65536.0 * LM_SAMPLE_TIME * LM_SAMPLE_TIME)
#define COEF_WHEEL            (ENCODER_RES * 4.0 / (PI * WHEELS_DIAMETER))
#define COEF_RIGHT_WHEEL      (ENCODER_RES * 4.0 / (PI * WHEELS_DIAMETER))
#define COEF_LEFT_WHEEL       (ENCODER_RES * 4.0 / (PI * WHEELS_DIAMETER))

// Entrées / sorties
#define LM_CS1                PORTDbits.RD6        // = MOTOR RIGHT
#define LM_READ               PORTDbits.RD7
#define LM_RST                PORTCbits.RC2

#if defined(REV_1_0)
    // Board revision 1.0
    #define LM_CS2            PORTBbits.RB4        // = MOTOR LEFT
    #define LM_PS             PORTAbits.RA5
    #define LM_WRITE          PORTAbits.RA4

    #define initBoardIO() {                                                                                        \
        TRISA&= 0b11001111;        /* LM_WRITE, LM_PS en sortie */                                                 \
        TRISB&= 0b11101111;        /* LM_CS2 en sortie */                                                          \
        TRISC&= 0b11111011;        /* LM_RST en sortie */                                                          \
        TRISD&= 0b00111111;        /* LM_CS1, LM_READ en sortie */                                                 \
    }
#elif defined(REV_1_1)
    // Board revision 1.1
    #define LM_CS2            PORTAbits.RA5        // = MOTOR LEFT
    #define LM_PS             PORTAbits.RA4
    #define LM_WRITE          PORTAbits.RA3

    #define initBoardIO() {                                                                                        \
        TRISA&= 0b11000111;        /* LM_WRITE, LM_PS, LM_CS2 en sortie */                                         \
        TRISC&= 0b11111011;        /* LM_RST en sortie */                                                          \
        TRISD&= 0b00111111;        /* LM_CS1, LM_READ en sortie */                                                 \
    }
#else
    #error Unknown board revision
#endif

#define LM_D0                PORTEbits.RE0
#define LM_D1                PORTEbits.RE1
#define LM_D2                PORTEbits.RE2
#define LM_D3                PORTDbits.RD4
#define LM_D4                PORTCbits.RC1
#define LM_D5                PORTCbits.RC7
#define LM_D6                PORTCbits.RC0
#define LM_D7                PORTCbits.RC6
#define LM_DATA_BUS          ((LM_D7 << 7) | (LM_D6 << 6) | (LM_D5 << 5) | (LM_D4 << 4) | (LM_D3 << 3) | (LM_D2 << 2) | (LM_D1 << 1) | LM_D0)

// Constantes du LM
#define LM_CMD_RESET         0x00        ///< Initialize -- Reset LM628
#define LM_CMD_PORT8         0x05        ///< Initialize -- Select 8-Bit Output
#define LM_CMD_PORT12        0x06        ///< Initialize -- Select 12-Bit Output
#define LM_CMD_DFH           0x02        ///< Initialize -- Define Home
#define LM_CMD_SIP           0x03        ///< Interrupt -- Set Index Position
#define LM_CMD_LPEI          0x1B        ///< Interrupt -- Interrupt on Error
#define LM_CMD_LPES          0x1A        ///< Interrupt -- Stop on Error
#define LM_CMD_SBPA          0x20        ///< Interrupt -- Set Breakpoint, Absolute
#define LM_CMD_SBPR          0x21        ///< Interrupt -- Set Breakpoint, Relative
#define LM_CMD_MSKI          0x1C        ///< Interrupt -- Mask Interrupts
#define LM_CMD_RSTI          0x1D        ///< Interrupt -- Reset Interrupts
#define LM_CMD_LFIL          0x1E        ///< Filter -- Load Filter Parameters
#define LM_CMD_UDF           0x04        ///< Filter -- Update Filter
#define LM_CMD_LTRJ          0x1F        ///< Trajectory -- Load Trajectory
#define LM_CMD_STT           0x01        ///< Trajectory -- Start Motion
#define LM_CMD_RDSTAT                    ///< Report -- Read Status Byte
#define LM_CMD_RDSIGS        0x0C        ///< Report -- Read Signals Register
#define LM_CMD_RDIP          0x09        ///< Report -- Read Index Position
#define LM_CMD_RDDP          0x08        ///< Report -- Read Desired Position
#define LM_CMD_RDRP          0x0A        ///< Report -- Read Real Position
#define LM_CMD_RDDV          0x07        ///< Report -- Read Desired Velocity
#define LM_CMD_RDRV          0x0B        ///< Report -- Read Real Velocity
#define LM_CMD_RDSUM         0x0D        ///< Report -- Read Integration Sum

#define LM_BUSY_BIT                       1        ///< Busy bit
#define LM_COMMAND_ERROR                  2        ///< Command Error [Interrupt]
#define LM_TRAJECTORY_COMPLETE            4        ///< Trajectory Complete [Interrupt]
#define LM_INDEX_PULSE                    8        ///< Index Pulse Observed [Interrupt]
#define LM_WRAP_AROUND                   16        ///< Wraparound Occurred [Interrupt]
#define LM_POSITION_ERROR                32        ///< Excessive Position Error [Interrupt]
#define LM_BREAKPOINT                    64        ///< Breakpoint Reached [Interrupt]
#define LM_MOTOR_OFF                    128        ///< Motor Off

#define LM_LFIL_IL                        1
#define LM_LFIL_KD                        2
#define LM_LFIL_KI                        4
#define LM_LFIL_KP                        8

#define LM_LTRJ_POS_REL                   1
#define LM_LTRJ_LOAD_POS                  2
#define LM_LTRJ_VEL_REL                   4
#define LM_LTRJ_LOAD_VEL                  8
#define LM_LTRJ_ACC_REL                  16
#define LM_LTRJ_LOAD_ACC                 32
#define LM_LTRJ_MOTOR_OFF               256
#define LM_LTRJ_STOP_ABRUPT             512
#define LM_LTRJ_STOP_SMOOTH            1024
#define LM_LTRJ_VEL_MODE               2048
#define LM_LTRJ_FORWARD_DIR            4096

// Haut niveau
#define GOTO_STRAIGHT                     0
#define GOTO_CURVE_RIGHT                  1
#define GOTO_CURVE_LEFT                   2

/**
 * Initialise l'interface entre le PIC et les LM629 puis effectue un RESET des LM629.
 * Cette fonction n'a besoin d'être appelée qu'une seule fois (avant la boucle programme principale).
 * Il est supposée qu'il s'écoule au moins 1.5 ms entre l'appel de cette fonction et
 * toute autre fonction de communication avec les LMs (durée du RESET).
*/
#define initBoard() {                                                                                            \
    /* Initialisation des PINs */                                                                                \
    initBoardIO();                                                                                               \
    dataBusDirection(DATA_IN);                                                                                   \
                                                                                                                 \
    /* Initialisation de l'état des PINs */                                                                      \
    LM_CS1 = 1;                                                                                                  \
    LM_CS2 = 1;                                                                                                  \
    LM_RST = 1;                                                                                                  \
    LM_PS = PS_DATA;                                                                                             \
    LM_READ = 1;                                                                                                 \
    LM_WRITE = 1;                                                                                                \
                                                                                                                 \
    /* Reset hard des LMs */                                                                                     \
    LM_RST = 0;             /* doit rester à 0 pendant au moins 8 périodes d'horloge du LM, à 8 MHz = 1 us */    \
    Delay100TCYx(1);        /* attends 100 cycles, à 48 MHz = 2.08 us */                                         \
    LM_RST = 1;                                                                                                  \
                                                                                                                 \
    /* Compter 1.5 ms à partir d'ici pour que le RESET soit effectif. */                                         \
}

void checkLM629Interrupt(void);

// Fonctions de bas niveau
void dataBusDirection(char dir);
void waitBusyLM(char axis);
void writeCommand(char axis, BYTE command);
void writeDataWord(char axis, WORD data);
WORD readDataWord(char axis);
BYTE readStatus(char axis);
BOOL readStatusBit(char axis, BYTE type);
void resetInterrupt(char axis, WORD type);
void resetAllInterrupt(char axis);
void initLM(char axis, WORD kp, WORD ki, WORD kd, WORD il);

// Fonctions de récupérations des grandeurs actuelles
DWORD getDesiredPosition(char axis);
DWORD getRealPosition(char axis);
DWORD getDesiredVelocity(char axis);
DWORD getRealVelocity(char axis);
WORD getIntegrationSum(char axis);
short getRelPos(char axis);

// Fonctions de génération des trajectoires
void newPosition(char axis, DWORD pos, DWORD vel, DWORD acc);
void newVelocity(char axis, DWORD vel, DWORD acc, char dir);
void changeVelocity(char axis, DWORD vel, char dir);
void start(char axis);
void stop(char axis, WORD type);
void setBreakpoint(char axis, DWORD pos, BOOL rel);

// Fonctions de haut niveau
BOOL initLMs(void);
void moveForward(short pos, short vel, short acc);
void moveBackward(short pos, short vel, short acc);
void turnRight(short angle, short vel, short acc);
void turnLeft(short angle, short vel, short acc);
void turn(short angle, short vel, short acc, short c, BOOL rel);
void goTo(short x, short y, short vel, short acc, BYTE mode, short d);
BOOL isTrajComplete(void);
void waitTrajComplete(void);
BOOL isTrajEngaged(void);

#endif
