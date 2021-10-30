#include "Arduino.h"
#include "stepper.h"
#include "eq6.h"
#include "combine.h"
#include "debug.h"
#include "serial.h"
#include "driver.h"

int8_t  trackingRate = 0;

/* Structure holding information used to generate stepper pulses
 * that generate motion at the siderial, solar, and lunar tracking
 * rates.
 *
 * Note: The track rate table contains the amount that the
 * XTAL frequency needs to be divided by to generate step interrupts.
 *
 * The divisor is:
 *
 * (secs_per_360 * clk_freq) / (lcm * mech_div * steps_per_step_rotn * microstep)
 *
 * This will not necessarily be an integer. If rounded we may get errors up
 * to 1 in 10^3. The fractional part can be realized by dropping one
 * interrupt every rnd(divisor) / (divisor - rnd(divisor).
 *
 * Negative numbers represent cases where interrupts should be added (ie the
 * ISR executed again)
 */

// Scale div from 16 MHz to current clk div
#define _DIVVAL(x)      (((x) * (16UL)) / (CLK_RATE / 1000000UL))

struct trackRate_s trackRateTable[NUM_RATES] =
{
#if 1//eq6 pro stepper use 1/16, motor max current is important to protect gear escape
    { _DIVVAL(637UL), 1429UL, 0, 86164 },      // Sidereal rate
    { _DIVVAL(638UL), 2141UL, 1, 86400 },     // Solar/planetary rate
    { _DIVVAL(660UL), 3124UL, 0, 89309 }      // Lunar rate
#else
    { _DIVVAL(3129UL), 9947UL, 0, 86164 },      // Sidereal rate
    { _DIVVAL(3137UL), 12306UL, 1, 86400 },     // Solar/planetary rate
    { _DIVVAL(3243UL), 27736UL, 0, 89309 }      // Lunar rate
#endif
};

/* Define the stepping table. This defines the excitation to be used
 * over a complete "cycle" of the stepper motor
 *
 * These are signed, four bit values. Coil 1 is the LSN, Coil 2 is the MSN
 */

#if 0
/* Step table. Values scaled such that one coil is always fully driven.
 * Gives lots of torque, but the actual travel is lumpy
 */
uint8_t    microTable[STEPS_PER_CYCLE] =
{
    EX_0, 
    
    EX_P_0_2, EX_P_0_4, EX_P_0_7,
    
    EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, 
    
    EX_P_0_7, EX_P_0_4, EX_P_0_2,

    EX_0, 
    
    EX_M_0_2, EX_M_0_4, EX_M_0_7,

    EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, 

    EX_M_0_7, EX_M_0_4, EX_M_0_2
};
#else
/* Conventional microstep table. Torque vector with magnitude 1. Gives
 * less torque that the first table, but the change in smoothness doesn't
 * seem to be worth the loss of torque
 */
uint8_t    microTable[STEPS_PER_CYCLE] =
{
    EX_0,

    EX_P_0_2, EX_P_0_4, EX_P_0_56, EX_P_0_7, EX_P_0_83, EX_P_0_92, EX_P_0_98,

    EX_P_1,

    EX_P_0_98, EX_P_0_92, EX_P_0_83, EX_P_0_7, EX_P_0_56, EX_P_0_4, EX_P_0_2,

    EX_0,

    EX_M_0_2, EX_M_0_4, EX_M_0_56, EX_M_0_7, EX_M_0_83, EX_M_0_92, EX_M_0_98,

    EX_M_1,

    EX_M_0_98, EX_M_0_92, EX_M_0_83, EX_M_0_7, EX_M_0_56, EX_M_0_4, EX_M_0_2,
};
#endif /* 0 */

uint8_t    halfTable[STEPS_PER_CYCLE] = 
{
    EX_P_1, EX_P_1, EX_P_1, EX_P_1,

    EX_P_1, EX_P_1, EX_P_1, EX_P_1,

    EX_P_1, EX_P_1, EX_P_1, EX_P_1,

    EX_0, EX_0, EX_0, EX_0,

    EX_M_1, EX_M_1, EX_M_1, EX_M_1,

    EX_M_1, EX_M_1, EX_M_1, EX_M_1,

    EX_M_1, EX_M_1, EX_M_1, EX_M_1,

    EX_0, EX_0, EX_0, EX_0,
};

uint8_t    fullTable[STEPS_PER_CYCLE] =
{
    EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1,
    EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1, EX_P_1,

    EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1,
    EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1, EX_M_1
};

/* Setup the table of divisors of the siderial interrupt use to
 * achieve the required tracking rate.
 */
#define USE_RELAY        0              // Activate the magic relay [RA only]
#define USE_MICRO        1              // Use the microstep table
struct
{
    uint16_t        divisor;             // Siderial interrupts per step
    uint8_t        flags;               // Control flags
} rateConvert[] =
{
    [SPEED_0_X] = {1/*3 * SIDERIAL_LCM*/, _BV(USE_MICRO)},  // Special value,change 1-> 48 because there is no chance to stepProcess for RA when DEC is frequent
    [SPEED_0_33_X] = {3 * SIDERIAL_LCM, _BV(USE_MICRO)},
    [SPEED_0_67_X] = {(3 * SIDERIAL_LCM) / 2, _BV(USE_MICRO)},
    [SPEED_1_X] = {SIDERIAL_LCM, _BV(USE_MICRO)},
    [SPEED_1_33_X] = {(3 * SIDERIAL_LCM) / 4,  _BV(USE_MICRO)},
    [SPEED_1_67_X] = {(3 * SIDERIAL_LCM) / 5,  _BV(USE_MICRO)},
    [SPEED_2_X] = {SIDERIAL_LCM / 2, _BV(USE_MICRO) | _BV(USE_RELAY)},
    [SPEED_4_X] = {SIDERIAL_LCM / 4, _BV(USE_MICRO) | _BV(USE_RELAY)},
    [SPEED_8_X] = {SIDERIAL_LCM / 8, _BV(USE_MICRO) | _BV(USE_RELAY)},
    [SPEED_16_X] = {SIDERIAL_LCM / 16, _BV(USE_RELAY)},
    [SPEED_SPIN] = {SIDERIAL_LCM / 24, _BV(USE_RELAY)},
    [SPEED_320_X] = {SIDERIAL_LCM / 320, _BV(USE_RELAY)}
};

/* Create the instance of the stepper excitation info
 */
struct excitation_s     raExcitation;
struct excitation_s     decExcitation;

/* Define instances of stepper state info
 */
struct stepState_s      raState;
struct stepState_s      decState;
uint8_t                 doHalfStep = 0;

/* Info for tracking rate correction */
uint16_t        adjCtr;
uint16_t        adjLimit;
uint16_t        doDropInt;
uint32_t        secsPerRotation;

const int MYdirpin = 4;
const int MYsteppin = 5;
const int MYenable = 12;
const int MXdirpin = 7;
const int MXsteppin = 6;
const int MXenable = 8;
volatile boolean MXtoggle = 0;
volatile boolean MYtoggle = 0;
volatile boolean MXdirLevel = LOW;
volatile boolean MYdirLevel = LOW;
void drv8825_init(void) {
  pinMode(MXdirpin, OUTPUT);
  digitalWrite(MXdirpin, MXdirLevel); 
  pinMode(MXsteppin, OUTPUT);
  digitalWrite(MXsteppin, HIGH);
  pinMode(MXenable, OUTPUT);
  digitalWrite(MXenable, LOW); //enable

  pinMode(MYdirpin, OUTPUT);
  digitalWrite(MXdirpin, MYdirLevel); 
  pinMode(MYsteppin, OUTPUT);
  digitalWrite(MYsteppin, HIGH);
  pinMode(MYenable, OUTPUT);
  digitalWrite(MYenable, LOW); //enable 
}

/* stepperInit() initializes the state of the stepper code.
 *
 * The current implementation uses a single 16-bit timer with a fixed
 * period shared between RA and DEC.
 *
 * Passed:
 *      Nothing
 *
 * Returns:
 *      Nothing
 *
 * Notes:
 * An alternate implementation would use a pair of 16 bit timers with 
 * their timeouts set to the step period. This would minimize the
 * number of interrupts, but would take an extra timer.
 *
 * The current implementation is preferred until we're sure the extra
 * timer isn't needed elsewhere or until there is a performance
 * problem caused by the extra interrupt load caused by having
 * multiple interrupts per step.
 */
void
stepperInit(void)
{
    drv8825_init();
    /* Initialize the excitation state */
    raExcitation.c1Ex = raExcitation.c2Ex = EX_0;
    raExcitation.useRelay = 0;
    raState.pExcite = &raExcitation;       
    raState.dir = 1;

    decExcitation.c1Ex = decExcitation.c2Ex = EX_0;
    decState.pExcite = &decExcitation;       
    decState.dir = 1;

    /* Initialize the siderial rate timer */
    cli();
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
    sei();
}

/* setTrackRate() sets the tracking rate used by the stepper module.
 *
 * Passed:
 *      rate            The tracking rate (index)
 *
 * Returns:
 *      Nothing
 *
 * Note:
 *      If an illegal rate is entered the current rate will not be changed
 */
void
setTrackRate(int8_t rate)
{
    /* If the track rate is <0 then disable siderial rate use in
     * combine.c and return, leaving the current clock steup
     */
    if (rate < 0)
    {
        noTrack = 1;
        return;
    }    

    /* Do nothing if the rate is not supported */
    if ((uint8_t)rate >= (sizeof(trackRateTable) / sizeof(struct trackRate_s)))
        return;
    
    /* Enable tracking */
    noTrack = 0;

  //Timer1 setting celestial rate
//    noInterrupts();
    TCCR1A = 0; 
    TCCR1B = 0; 
    TCNT1 = 0;
    OCR1A = trackRateTable[rate].div - 1;
    #if 0
    //print log {
    str = getBuff();
    sprintf(str, "OCR1A:%d",OCR1A);
    putstr(str);
    // }
    #endif

    TCCR1B |= (1 << WGM12);   // CTC mode
    TCCR1B |= (1 << CS10);    // no prescaler
    //TCCR1B |= (1 << CS11);    // 8 prescaler
    //TCCR1B |= (1 << CS11 | 1 << CS10);    // 64 prescaler
    //TCCR1B |= (1 << CS12);    // 256 prescaler
    //TCCR1B |= (1 << CS12 | 1 << CS10);    // 1024 prescaler
//    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
//    interrupts();             // enable all interrupts
    /* Update adjustment data */
    adjCtr = 0;
    adjLimit = trackRateTable[rate].adj;
    doDropInt = trackRateTable[rate].doDropInt;
    secsPerRotation = trackRateTable[rate].secsPerRotation;

    /* Update global state */
    trackingRate = rate;

#if 0
    print16(trackRateTable[rate].div);
    serialTx(":");
    print16(trackRateTable[rate].adj);
    serialTx(":");
    print16(trackRateTable[rate].doDropInt);
    serialTx(":");
    print32(trackRateTable[rate].secsPerRotation);
    serialTx(".");
#endif
}

/* getSecsPerRotation() returns the number of seconds required to perform
 * a complete rotation of the mount in RA at the current tracking
 * speed
 *
 * Passed:
 *      Nothing
 *
 * Returns:
 *      The rotation time, in seconds
 */
uint32_t
getSecsPerRotation(void)
{
    return secsPerRotation;
}

/* setSpeed() is called by by the combiner to set the requested speed
 * for the axis
 *
 * Passed:
 *      pState          Axis state
 *      rate            Requested rate
 *
 * Returns:
 *      Nothing
 *
 * Notes:
 *      setRaSpeed() and setDecSpeed() are wrappers used by the combiner
 */
static void
setSpeed(struct stepState_s *pState, int8_t speed)
{
    /* If the current speed is zero then start the clock */
    if (pState->clkDivRatio == 0)
        pState->clkDivRatio = 1;        // Almost immediate clock

    pState->reqSpeed = speed;
    
}

void
setRaSpeed(int8_t speed)
{
    setSpeed(&raState, speed);
}

void
setDecSpeed(int8_t speed)
{
    setSpeed(&decState, speed);
}

/* setTickRate() is called by the state machine to set the clock interrupt
 * rate.
 *
 * Passed
 *      pState          The axis state
 *      tickRate        The clock rate to set
 *
 * Returns
 *      nothing
 */
void
setTickRate(struct stepState_s *pState, uint8_t tickRate)
{
    pState->clkDivRatio = rateConvert[tickRate].divisor;
}
 
/* stepperUpdateExcite() is a helper function that updates the stepper
 * coil excitation values from from the current stepping table.
 *
 * This function assumes that the coil 1 and coil 2 waveforms are 90deg
 * out of phase
 *
 */
static void
stepperUpdateExcite(struct stepState_s *pState, char RaDec)
{
    if(RaDec == 'R') {
      if(1/*pState->stepCtr % 8 == 0*/) {//Because of 1/8 microstep use for RA Motor
          digitalWrite(MXsteppin, MXtoggle);
          MXtoggle = !MXtoggle;
          digitalWrite(MXsteppin, MXtoggle);
          MXtoggle = !MXtoggle;
      }
    } else if(RaDec == 'D') {
      if(1/*pState->stepCtr % 8 == 0*/) {//Because of 1/8 microstep use for Dec Motor
          digitalWrite(MYsteppin, MYtoggle);
          MYtoggle = !MYtoggle;
          digitalWrite(MYsteppin, MYtoggle);
          MYtoggle = !MYtoggle;
      }      
    }
    #if 1
    pState->pExcite->c1Ex = pState->pTable[pState->stepCtr];
    pState->pExcite->c2Ex = pState->pTable[(pState->stepCtr + STEPS_PER_CYCLE / 4) & (STEPS_PER_CYCLE - 1)];
    #endif

    #if 0
    //print log {
    str = getBuff();
    sprintf(str, "stepperUpdateExcite():c1Ex:%d, c2Ex:%d", raExcitation.c1Ex,raExcitation.c2Ex);
    putstr(str);
    // }
    #endif
}

/* stepperProcess is the state machine that makes this whole thing
 * work! It is executed each axis interrupt to run the state machine
 * that handles operation and backlash processing.
 *
 * Like the other state machines in the program it takes advantage
 * of the GNU computed goto to operate very efficiently.
 *
 * Passed
 *      pState          The axis state
 *
 * Returns
 *      Nothing
 */
#define _GET_TABLE(f)   ((f) ? (doHalfStep ? halfTable : microTable) : fullTable)

void
stepperProcess(struct stepState_s *pState, char RaDec)
{
    #if 0
    //print log {
    str = getBuff();
    sprintf(str, "stepProc start: pState:%d, reqSpeed:%d",pState->pState,pState->reqSpeed);
    putstr(str);
    // }
    #endif

    // Step up the initial state pointer
    if (pState->pState == 0)
        pState->pState = &&enter_idle_pos;

    // If we've been asked to halt by the stepper driver then do nothing
    if (pState->pExcite->halt)
        return;

    /* Make sure both finPos and finNeg are not set - that will
     * lead to a loop as the code tries to meet both!
     */
    if (pState->finPos && pState->finNeg)
        pState->finPos = pState->finNeg = 0;

    // Jump to the current state
    goto *pState->pState;

    /* There are six states in the machine
     *
     * - idle_pos       Idle (last move in positive direction)
     * - spin_pos       Taking up backlash in positive direction
     * - move_pos       Moving in the positive direction
     *
     * There are "negative" versions of these states.
     *
     * Just to make things simple we use the "idle" state as a central
     * decision point.
     */

enter_idle_pos:
    #if 0
    str = getBuff();
    sprintf(str, "enter_idle_pos,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* We're about to move into the idle_pos state. We end up here if
     * we're stopping or changing direction
     */
    if (pState->reqSpeed == SPEED_0_X)
    {
        /* We're going to stop - if we're in the correct direction then
         * stop, else start spinning in the other direction
         */
        pState->isMoving = 0;
        if (pState->finNeg)
            goto enter_spin_neg;
        else
        {
            /* Stop now! */
            setTickRate(pState, SPEED_0_X);
            pState->pExcite->c1Ex = pState->pExcite->c2Ex = EX_0;
            pState->pExcite->useRelay = 0;

            // For this state just call the entry point each interrupt
            pState->pState = &&enter_idle_pos;
            if(RaDec == 'R') {
                digitalWrite(MXenable, HIGH);//diable
            } else if(RaDec == 'D') {
                digitalWrite(MYenable, HIGH);//diable 
            }
        }
    }
    else if (pState->reqSpeed > SPEED_0_X)
    {
        if(RaDec == 'R') {
             digitalWrite(MXenable, LOW);//enable
        } else if(RaDec == 'D') {
            digitalWrite(MYenable, LOW);//enable 
        }
        /* We're now moving in the positive direction. As we are
         * already engaged in the positive direction we can start
         * running
         */
        goto enter_move_pos;
    }
    else
    {
        if(RaDec == 'R') {
             digitalWrite(MXenable, LOW);//enable
        } else if(RaDec == 'D') {
            digitalWrite(MYenable, LOW);//enable 
        }
        /* Must be a negative move direction. Take up the backlash
         * in the negative direction
         */
        goto enter_spin_neg;
    }

    return;

enter_idle_neg:
    #if 0
    str = getBuff();
    sprintf(str, "enter_idle_neg,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* We're about to move into the idle_neg state. We end up here if
     * we're stopping or changing direction
     */
    pState->isMoving = 0;
    if (pState->reqSpeed == SPEED_0_X)
    {
        /* We're going to stop - if we're in the correct direction then
         * stop, else start spinning in the other direction
         */
        if (pState->finPos)
            goto enter_spin_pos;
        else
        {
            /* Stop now! */
            setTickRate(pState, SPEED_0_X);
            pState->pExcite->c1Ex = pState->pExcite->c2Ex = EX_0;
            pState->pExcite->useRelay = 0;

            // For this state just call the entry point each interrupt
            pState->pState = &&enter_idle_neg;
            if(RaDec == 'R') {
                digitalWrite(MXenable, HIGH);//diable
            } else if(RaDec == 'D') {
                digitalWrite(MYenable, HIGH);//diable 
            }
        }
    }
    else if (pState->reqSpeed < SPEED_0_X)
    {
        if(RaDec == 'R') {
             digitalWrite(MXenable, LOW);//enable
        } else if(RaDec == 'D') {
            digitalWrite(MYenable, LOW);//enable 
        }
        /* We're now moving in the negative direction. As we are
         * already engaged in the negative direction we can start
         * running
         */
        goto enter_move_neg;
    }
    else
    {
        if(RaDec == 'R') {
             digitalWrite(MXenable, LOW);//enable
        } else if(RaDec == 'D') {
            digitalWrite(MYenable, LOW);//enable 
        }
        /* Must be a positive move direction. Take up the backlash
         * in the positive direction
         */
        goto enter_spin_pos;
    }
    return;

enter_spin_pos:
    #if 0
    str = getBuff();
    sprintf(str, "enter_spin_pos,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* Spin in the positive direction to take up backlash in the
     * gear chain
     */
    pState->isMoving = 0;
    if (pState->backlash == 0)
    {
        /* No backlash - go to the idle_pos state which will take us
         * to the correct place
         */
        goto enter_idle_pos;
    }
    else
    {
        uint8_t flags = rateConvert[SPEED_SPIN].flags;
        
        /* There is a backlash setting - get ready to spin! */
        pState->count = 0;

        setTickRate(pState, SPEED_SPIN);
        pState->pTable = _GET_TABLE(flags & _BV(USE_MICRO));
        pState->pExcite->useRelay = flags & _BV(USE_RELAY);
        pState->pState = &&run_spin_pos;
        
        // Fall through to run the spin state
    }

run_spin_pos:
    #if 0
    str = getBuff();
    sprintf(str, "run_spin_pos,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    // determine direction
    if(RaDec == 'R') {
      (pState->dir > 0) ? digitalWrite(MXdirpin, MXdirLevel): digitalWrite(MXdirpin, !MXdirLevel);
    } else if(RaDec == 'D') {
      (pState->dir > 0) ? digitalWrite(MYdirpin, MYdirLevel): digitalWrite(MYdirpin, !MYdirLevel);
    }
    // Update excitation value
    stepperUpdateExcite(pState, RaDec);
    pState->stepCtr = (pState->stepCtr + pState->dir) & (STEPS_PER_CYCLE - 1);

    /* Check the count. If we've spun enough then go back to the
     * idle_pos state which will send us the right way
     */
    if (++pState->count > pState->backlash)
       goto enter_idle_pos;
    return;

enter_spin_neg:
    #if 0
    str = getBuff();
    sprintf(str, "enter_spin_neg,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* Spin in the negative direction to take up backlash in the
     * gear chain
     */
    pState->isMoving = 0;
    if (pState->backlash == 0)
    {
        /* No backlash - go to the idle_neg state which will take us
         * to the correct place
         */
        goto enter_idle_neg;
    }
    else
    {
        uint8_t flags = rateConvert[SPEED_SPIN].flags;
        
        /* There is a backlash setting - get ready to spin! */
        pState->count = 0;

        setTickRate(pState, SPEED_SPIN);
        pState->pTable = _GET_TABLE(flags & _BV(USE_MICRO));
        pState->pExcite->useRelay = flags & _BV(USE_RELAY);
        pState->pState = &&run_spin_neg;
        // Fall through to run the spin state
    }

run_spin_neg:
    #if 0
    str = getBuff();
    sprintf(str, "run_spin_neg,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    // determine direction
    if(RaDec == 'R') {
      (pState->dir > 0) ? digitalWrite(MXdirpin, !MXdirLevel): digitalWrite(MXdirpin, MXdirLevel);
    } else if(RaDec == 'D') {
      (pState->dir > 0) ? digitalWrite(MYdirpin, !MYdirLevel): digitalWrite(MYdirpin, MYdirLevel);
    }
    // Update excitation value
    stepperUpdateExcite(pState, RaDec);
    pState->stepCtr = (pState->stepCtr - pState->dir) & (STEPS_PER_CYCLE - 1);

    /* Check the count. If we've spun enough then go back to the
     * idle_neg state which will send us the right way
     */
    if (++pState->count > pState->backlash)
       goto enter_idle_neg;

    return;

enter_move_pos:
    #if 0
    str = getBuff();
    sprintf(str, "enter_move_pos,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* Start moving in the positive direction. Save the requested
     * speed as the current speed so we can detect changes in the
     * requested speed
     */
    pState->isMoving = 1;
    if (pState->reqSpeed > SPEED_0_X)
    {
        uint8_t flags = rateConvert[pState->reqSpeed].flags;
        
        setTickRate(pState, pState->reqSpeed);
        pState->pTable = _GET_TABLE(flags & _BV(USE_MICRO));
        pState->pExcite->useRelay = flags & _BV(USE_RELAY);
        pState->pState = &&run_move_pos;
        pState->curSpeed = pState->reqSpeed;

        /* Fall through to move action */
    }
    else
    {
        /* We're not going in the positive direction any more */
        goto enter_idle_pos;
    }
    return;

run_move_pos:
    #if 0
    str = getBuff();
    sprintf(str, "run_move_pos,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    #if 0
    str = getBuff();
    sprintf(str, "portc:0x%x,portb:0x%x,dec1:%d,dec2:%d",PORTC, PORTB,decExcitation.c1Ex, decExcitation.c2Ex);
    putstr(str);
    #endif
    if (pState->curSpeed == pState->reqSpeed)
    {
        // determine direction
        if(RaDec == 'R') {
          (pState->dir > 0) ? digitalWrite(MXdirpin, MXdirLevel): digitalWrite(MXdirpin, !MXdirLevel);
        } else if(RaDec == 'D') {
          (pState->dir > 0) ? digitalWrite(MYdirpin, MYdirLevel): digitalWrite(MYdirpin, !MYdirLevel);
        }
        /* We're still moving at the same speed. Do it
         */
        stepperUpdateExcite(pState, RaDec);
        pState->stepCtr = 
                    (pState->stepCtr + pState->dir) & (STEPS_PER_CYCLE - 1);
    }
    else
    {
        /* Go baxk to idle_pos that will decide the next state */
        goto enter_idle_pos;
    }
    return;

enter_move_neg:
    #if 0
    str = getBuff();
    sprintf(str, "enter_move_neg,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    /* Start moving in the negative direction. Save the requested
     * speed as the current speed so we can detect changes in the
     * requested speed
     */
    pState->isMoving = 1;
    if (pState->reqSpeed < SPEED_0_X)
    {
        uint8_t flags = rateConvert[-pState->reqSpeed].flags;
        
        setTickRate(pState, -pState->reqSpeed);
        pState->pTable = _GET_TABLE(flags & _BV(USE_MICRO));
        pState->pExcite->useRelay = flags & _BV(USE_RELAY);
        pState->pState = &&run_move_neg;
        pState->curSpeed = pState->reqSpeed;

        /* Fall through to move action */
    }
    else
    {
        /* We're not going in the negative direction any more. Stop and
         * continue from there
         */
        goto enter_idle_neg;
    }
    return;

run_move_neg:
    #if 0
    str = getBuff();
    sprintf(str, "run_move_neg,reqSpeed:%d",pState->reqSpeed);
    putstr(str);
    #endif
    
    if (pState->curSpeed == pState->reqSpeed)
    {
        // determine direction
        if(RaDec == 'R') {
          (pState->dir > 0) ? digitalWrite(MXdirpin, !MXdirLevel): digitalWrite(MXdirpin, MXdirLevel);
        } else if(RaDec == 'D') {
          (pState->dir > 0) ? digitalWrite(MYdirpin, !MYdirLevel): digitalWrite(MYdirpin, MYdirLevel);
        }
        /* We're still moving at the same speed. Do it
         */
        stepperUpdateExcite(pState, RaDec);
        pState->stepCtr = 
                    (pState->stepCtr - pState->dir) & (STEPS_PER_CYCLE - 1);
    }
    else
    {
        /* Go back to the idle_neg. It will determine the next state */
        goto enter_idle_neg;
    }
    return;
}

/* stepperInt() is called each siderial interrupt. This is divided down
 * in software to derive the actual stepper timing.
 *
 * Passed:
 *      Nothing
 *
 * Returns:
 *      Nothing
 */
ISR(TIMER1_COMPA_vect)
{ 
    /* Update the tracking rate adjustment counter */
    ++adjCtr;

    /* If we're dropping then drop if necessary */
    if (doDropInt && adjLimit && adjCtr >= adjLimit)
    {
        /* Drop interrupt */
        adjCtr = 0;
        return;
    }

do_again:
    /* Run the state machine for the DEC and RA axis */
    if (raState.clkDivRatio != 0 && ++raState.divCtr >= raState.clkDivRatio)
    {
        // Execute the RA state machine
        raState.divCtr = 0;
        stepperProcess(&raState, 'R');
        #if 0
        //print log {
        str = getBuff();
        //sprintf(str, "stepper.cpp:stepProcRa():pState=%d,reqSpeed=%d",raState.pState,raState.reqSpeed);
        sprintf(str, "raState.divCtr=%d,raState.clkDivRatio=%d",raState.divCtr,raState.clkDivRatio);
        putstr(str);
        #endif
        #if 0
        //print log {
        str = getBuff();
        sprintf(str, "poetd:0x%x",RAPORT);
        putstr(str);
        #endif
    }

    if (decState.clkDivRatio != 0 && ++decState.divCtr >= decState.clkDivRatio)
    {
        // Execute the DEC state machine
        decState.divCtr = 0;
        stepperProcess(&decState, 'D');
        #if 0
        //print log {
        str = getBuff();
        sprintf(str, "stepper.cpp:stepProcDec():pState=%d,reqSpeed=%d",decState.pState,decState.reqSpeed);
        putstr(str);
        // }
        #endif
        #if 0
        //print log {
        str = getBuff();
        sprintf(str, "decState.divCtr=%d,decState.clkDivRatio=%d",decState.divCtr,decState.clkDivRatio);
        putstr(str);
        // }
        #endif
    }

    /* If we need to "insert" an interrupt do it now */
    if (!doDropInt && adjLimit && adjCtr >= adjLimit)
    {
        adjCtr = 0;
        goto do_again;
    }
}

/* stepperInMotion() is called to determine whether the RA and/or DEC axis
 * are in motion. Note that the axis is not considered to be "in motion"
 * when it is performing backlash take up when starting, or ending, motion.
 *
 * Passed:
 *      Nothing
 *
 * Returns:
 *      pRaMoving
 *      pDecMoving
 */
void
stepperInMotion(uint8_t *pRaMoving, uint8_t *pDecMoving)
{
    *pRaMoving = raState.isMoving;
    *pDecMoving = decState.isMoving;
}
