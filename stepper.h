#ifndef _STEPPER_H_
#define _STEPPER_H_

/* Define the number of steps per cycle and LCM */
#define STEPS_PER_CYCLE     32  /* Steps per cycle (complete set of phases) */
//#define SIDERIAL_LCM        (3 * 16)   /* Divides to give all speeds for eq6 original */
#define SIDERIAL_LCM        (3 * 320)   /* Divides to give all speeds for eq6 pro */
/* Define the encoding used to hold the exictation values for each
 * stepper coil.
 */
#define EX_0    0   // Inactive
#define EX_P_0_2  1   // +0.2 active
#define EX_P_0_4  2   // +0.4 active
#define EX_P_0_56 3   // +0.56 active
#define EX_P_0_7  4             // +0.7 active
#define EX_P_0_83 5   // +0.83 active
#define EX_P_0_92 6   // +0.92 active
#define EX_P_0_98 7   // +0.98 active
#define EX_P_1    8   // +1 Active

#define EX_M_0_2  9   // -0.2 active
#define EX_M_0_4  10    // -0.4 active
#define EX_M_0_56 11              // -0.56 active
#define EX_M_0_7  12              // -0.7 active
#define EX_M_0_83 13    // -0.83 active
#define EX_M_0_92 14    // -0.92 active
#define EX_M_0_98 15    // -0.98 active
#define EX_M_1    16    // -1 Active

/* Define the struture used to pass excitation information from
 * the stepper module to the excitation module
 */
struct excitation_s
{
    uint8_t     c1Ex;           // Excitation for the first coil
    uint8_t     c2Ex;           // Excitation for the second coil
    uint8_t     useRelay;       // Activate relay to use high current RA coils
    uint8_t     counter;        // Decremented every 10 ms if not zero
    uint8_t     halt;           // Prevent stepping
};

extern struct excitation_s      raExcitation;
extern struct excitation_s      decExcitation;

/* Define a structure to hold the current "state" for each axis. This
 * allows up to use the same code for both axis, so I don't feel
 * tempted to skimp on the features
 */
struct stepState_s
{
    // Input (from the combiner)
    int8_t      reqSpeed;       // Rate requested

    // Output (to the driver)
    struct excitation_s         *pExcite;       

    // Configuration
    uint16_t    backlash;       // #steps to counteract backlash
    uint8_t     finPos;         // Finish movement in positive direction
    uint8_t     finNeg;         // Finish movement in negative direction
    int8_t      dir;            // 1 = normal, -1 = reverse

    // Machine state
    uint8_t     stepCtr;        // Current step in cycle
    int8_t      curSpeed;       // Current rate (when moving after spin)
    void        *pState;        // State pointer
    uint16_t    count;          // Counter used in states

    uint8_t     *pTable;        // Current step table

    // Support function state
    uint16_t     clkDivRatio;    // Current clock div
    uint16_t     divCtr;         // Clock division counter

    // Information for GOTO
    uint8_t     isMoving;       // Currently slewing/guiding [not backlash]
};

/* The following structure holds the tracking rate table */
#define NUM_RATES       3
struct trackRate_s
{
    uint16_t    div;            // tint
    uint16_t    adj;            // add/drop one int every ....
    uint8_t     doDropInt;      // drop ints if true, add extra ints if false
    uint32_t    secsPerRotation;        // Seconds per 360deg rotation
};

extern struct trackRate_s      trackRateTable[NUM_RATES];

 /* These are held in stepper.c */
extern struct stepState_s      raState;
extern struct stepState_s      decState;
extern int8_t                  trackingRate;

/* Prototypes */
void stepperInit(void);
void setRaSpeed(int8_t speed);
void setDecSpeed(int8_t speed);

void setTrackRate(int8_t rate);
void stepperInMotion(uint8_t *pRaMoving, uint8_t *pDecMoving);
uint32_t getSecsPerRotation(void);

/* DEBUG: Uses half step instread of microstep if true */
extern uint8_t  doHalfStep;
#endif /* _STEPPER_H_ */
