#include "Arduino.h"
#include "eq6.h"
#include "stepper.h"
#include "paddle.h"
#include "driver.h"
#include "sr.h"
#include "combine.h"
#include "serial.h"
#include "goto.h"

/* This file implements what is I've named "delta goto". The idea was
 * originally suggested by Rajiva.
 * (Actually it turns up in several controllers ...)
 *
 *
 * Basically the user points the scope to a star (or other object that
 * is near the target object) and tells the mount to "goto" that position
 * using any planetarium program (i.e. Cartes du Ciel)
 *
 * The user then uses the planetarium program to goto a position near the
 * first position. The mount calculates the difference and does the short
 * slew position.
 *
 * This program needs two bits of information:
 *
 * 1. Which coordinate is to be set, and which one to go to
 * 2. Which side of the mount you're working on
 *
 * The side information comes from a switch in the mount control box.
 *
 * The coordinate set uses one of two methods
 *
 * a. The LX200 :CM# command
 * b. A target of a long (12deg +) slew
 *
 * The second method avoids the Cartes du Ceil 'are you sure' message.
 */

uint8_t gotoSpeed;
uint16_t gotoRate;
uint8_t gotoDeltaSync;
uint8_t gotoSouth;

/* State data */
static uint8_t          precision;

static uint32_t         raNext;         // RA specified by host
static int32_t          decNext;        // DEC specified by host

static int32_t         raDisp;         // RA to be returned to host
static int32_t          decDisp;        // DEC to be returned to host

static uint32_t         raRunTime;      // 10 ms ticks to keep RA running
static uint32_t         decRunTime;     // 10 ms ticks to keep DEC running

static int8_t           raDir;          // RA inc [+] /dec [-]
static int8_t           decDir;         // DEC inc [+] /dec [-]

static int32_t          effRaGotoRate;    // Effective goto rate (siderial adj)

static uint8_t          isSync = 0;      // Sync position is set?

#define DEC_RANGE       (180L * 60L * 60L)
#define DEC_PER_360     (DEC_RANGE * 2)
#define DEC_MODULUS     (90L * 60L * 60L)
#define DEC_MAX_DELTA   (DEC_PER_360 / 30)

#define RA_RANGE        (24L * 60L * 60L)
#define RA_PER_360      (RA_RANGE)
#define RA_MODULUS      RA_RANGE
#define RA_MAX_DELTA    (RA_PER_360 / 30)

/* gotoOutputRA() transmits the current mount RA
 *
 */
#define _INS_DEC(p, n)  { \
                            div_t       tmp; \
                            tmp = div((n), 10); \
                            *p++ = tmp.quot + '0'; \
                            *p++ = tmp.rem + '0'; \
                        }

void
gotoOutputRA(void)
{
    static char         raStr[16];
    char                *pStr = raStr;

    uint32_t            disp = isSync ? raDisp : 0;

    ldiv_t              lower;
    ldiv_t              upper;

    /* Get the number of hours, minutes, and seconds represented by 
     * raDisp (or zero is the location hasn't been set)
     */
    lower = ldiv(disp, 60);          // secs in lower.rem
    upper = ldiv(lower.quot, 60);    // mins is upper.rem, hours is upper.quot

    /* Output hours and minutes */
    _INS_DEC(pStr, upper.quot);
    *pStr++ = ':';
    _INS_DEC(pStr, upper.rem);

    /* Output the seconds in the current precision */
    if (precision)
    {
        // High precision is ':xx' seconds
        *pStr++ = ':';
        _INS_DEC(pStr, lower.rem);
    }
    else
    {
        // Low precision is '.x' 1/10's of a minute
        *pStr++ = '.';
        *pStr++ = lower.rem / 6 + '0';
    }
   
    // Terminate the string
    *pStr++ = '#';
    *pStr = '\0';

    serialTx(raStr);
}

/* gotoOutputDec() transmits the current mount DEC
 *
 */
void
gotoOutputDec(void)
{
    static char         decStr[16];
    char                *pStr = decStr;

    uint32_t            dec;
    int32_t             disp;
   
    ldiv_t              lower;
    ldiv_t              upper;

    // Set the value to display. Point at the NCP or SCP depending
    // on the tracking direction
    if (isSync)
        disp = decDisp;
    else
    {
        disp = (rateInput.siderialRate > SPEED_0_X)
                            ? DEC_MODULUS - 1
                            : -(DEC_MODULUS - 1);
    }

    // Display the leading minus (if needed) and convert to positive number
    if (disp >= 0)
    {
        *pStr++ = '+';
        dec = disp;
    }
    else
    {
        *pStr++ = '-';
        dec = -disp;
    }

    /* Get the number of degrees, minutes, and seconds */
    lower = ldiv(dec, 60);           // secs in lower.rem
    upper = ldiv(lower.quot, 60);    // mins is upper.rem, degrees is upper.quot

    /* Output degrees and minutes */
    _INS_DEC(pStr, upper.quot);
    *pStr++ = '*';
    _INS_DEC(pStr, upper.rem);

    /* Output seconds if required by the precision */
    if (precision)
    {
        // High precision is ':xx' seconds
        *pStr++ = '\:';
        _INS_DEC(pStr, lower.rem);
    }
   
    *pStr++ = '#';
    *pStr = '\0';

    serialTx(decStr);
}

/* gotoTogglePrecision() is called to toggle the precision for
 * RA/DEC output
 *
 */
void
gotoTogglePrecision(void)
{
    precision = ! precision;
}


/* gotoSetRA() and gotoSetDec() set the next RA nad DEC positions
 *
 * Passed
 *      secs    The RA or DEC in seconds (of 360deg or 24 hrs respectively)
 *
 * Returns
 *      Nothing
 */
void
gotoSetRA(char *pStr)
{
    uint16_t        hr;
    uint16_t        min;
    uint16_t        sec;

    /* Parse the RA string. It can be in one of two formats:
     *
     * HH:MM:SS or HH:MM.S
     */
    while (*pStr == ' ')
         ++pStr;

    hr = (pStr[0] - '0') * 10 + (pStr[1] - '0');
    min = (pStr[3] - '0') * 10 + (pStr[4] - '0');

    if (pStr[5] == '.')
        sec = (pStr[6] - '0') * 6;
    else
        sec = (pStr[6] - '0') * 10 + (pStr[7] - '0');

    raNext = (((hr * 60L) + min) * 60L) + sec;
    serialTx("1");
}

void
gotoSetDec(char *pStr)
{
    int16_t     deg;
    int16_t     min;
    int16_t     sec;
    int16_t     sign;


    /* Parse the DEC string. It can be in one of two formats:
     *
     * sDD:MM:SS or sDD:MM*S
     */
    
    // Skip leading space and '+' sign
    while (*pStr == ' ' || *pStr == '+')
         ++pStr;

    // Get the sign
    if (*pStr == '-')
    {
        sign = -1;
        ++pStr;
    }
    else
        sign = 1;

    // Get deg & min
    deg = (pStr[0] - '0') * 10 + (pStr[1] - '0');
    min = (pStr[3] - '0') * 10 + (pStr[4] - '0');

    // Get the seconds (if present)
    if (pStr[5] == ':')
        sec = (pStr[6] - '0') * 10 + (pStr[7] - '0');
    else
        sec = 0; 

    decNext = sign * ((((deg * 60L) + min) * 60L) + sec);
    serialTx("1");
}

/* gotoTarget() is called when the remote PC says to slew to the target
 * position
 *
 */
void
gotoTarget(void)
{
    int32_t     raDelta;
    int32_t     decDelta;
    int32_t     secsPerRotation = getSecsPerRotation();

    /* Ok, now we have the new position. There are two things we can do
     * with it
     *
     * If either the RA or DEC are more than 1/45 of revolution
     * (i.e 8deg/360deg) we will just set the location as our
     * current location.
     *
     * If it is less then we subtract it from the current location to get the
     * delta.
     *
     * The time is:
     *
     * time = (angle * spr) / (apr * ss)
     *
     * angle:   Angle change in seconds
     * spr:     Clock seconds per rotation
     * apr:     Angle seconds per rotation
     * ss:      Effective slew speed
     *
     * Note: In DEC, ss = actual slew speed, in RA ss = actual slew + 1 if we
     * are rotating opposite siderial dir, actual slew - 1 if rotating in
     * siderial direction
     */

    /* Calculate the delta angles. Note that these are signed. If there are
     * greater than half to total angle then we must have wrapped around
     * and need to fix it
     */
    raDelta = raNext - raDisp;

    if (raDelta > (RA_RANGE / 2))
        raDelta -= RA_RANGE;
    else if (raDelta < -(RA_RANGE / 2))
        raDelta += RA_RANGE;

    decDelta = decNext - decDisp;
    if (decDelta > (DEC_RANGE / 2))
        decDelta -= 2 * DEC_RANGE;
    else if (decDelta < -(DEC_RANGE / 2))
        decDelta += 2 * DEC_RANGE;
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "decNext:%ld",decNext);
                putstr(str);
                // }
                #endif
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "decDisp:%ld",decDisp);
                putstr(str);
                // }
                #endif
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "decDelta:%ld",decDelta);
                putstr(str);
                // }
                #endif
    /* Stop motion and sync to the current position if delta sync is enabled
     * and any of the following are true
     *
     * - The newly requested position is too far away
     * - A position has never need synced
     */
    if (        gotoDeltaSync 
                && (            raDelta > RA_MAX_DELTA
                                || raDelta < -RA_MAX_DELTA
                                || decDelta > DEC_MAX_DELTA
                                || decDelta < -DEC_MAX_DELTA
                                || !isSync))
    {
        // Sync with the requested position
        gotoSync(0);

        // Stop any motion currently in progress
        gotoStop();

        // Return success indication
        serialTx("0");

        return;
    }

    /* Make sure we have a valid posiiton. If not then reject the GOTO
     */
    if (! isSync)
    {
        serialTx("1POS_NOT_SYNCED#");
        return;
    }

    /* Oh well, time to move I guess!
     *
     * 1. Set the RA/DEC direction flags. This direction will be used to
     *     update the angles during the move
     *
     * 2. Calculate the time, in 1/100sec units and set the run time
     *
     * 3. Start the motion in the relevent direction - it will be
     *    cancelled at the end of motion by the tick interrupt handler
     */
   
    // Set direction and convert delta to positive for easy calc
    if (raDelta > 0)
         raDir = 1;
    else
    {
        raDir = -1;
        raDelta = -raDelta;
    }

    /* Calculate time in 10 ms increments. Allow for the fact that the
     * the "effective" rotation rate increases if we're moving in the
     * opposite direction as siderial rotation and decreases when
     * moving in the same direction
     */
    if (raDir > 0)
        effRaGotoRate = gotoRate + 1;
    else
        effRaGotoRate = gotoRate - 1;

    raRunTime =         (raDelta * secsPerRotation)
                        / ((RA_PER_360 / 100 ) * effRaGotoRate);
    
    rateInput.gotoRaRate = (raRunTime > 0)
                                                ? -((int32_t)gotoSpeed) * raDir
                                                : SPEED_0_X;
    
    // Set direction and convert delta to positive for easy calc
    if (decDelta > 0)
         decDir = 1;
    else
    {
        decDir = -1;
        decDelta = -decDelta;
    }
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "decDelta:%ld",decDelta);
                putstr(str);
                // }
                #endif
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "secsPerRotation:%ld",secsPerRotation);
                putstr(str);
                // }
                #endif
                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "gotoRate:%d",gotoRate);
                putstr(str);
                // }
                #endif
    /* Calculate time in 10 ms increments for DEC. Use extra factor of 400 to
     * overflow of 32bit signed integer
     */
    decRunTime =        (decDelta * (secsPerRotation / 400))
                        / ((DEC_PER_360 / 400 / 100) * ((uint32_t)gotoRate));

    rateInput.gotoDecRate = (decRunTime > 0)
                                        ? ((int32_t)gotoSpeed) * decDir
                                        : SPEED_0_X;

                #if 0
                //print log {
                str = getBuff();
                sprintf(str, "decRunTime:%ld",decRunTime);
                putstr(str);
                // }
                #endif
    if (GOTO_SW_PIN & _BV(GOTO_SW_BIT))
        rateInput.gotoDecRate = -rateInput.gotoDecRate;

    // Get the slew on the road
    updateMountSpeed();

    // Tell the scope controller that the slew is OK
    serialTx("0");
}

/* gotoStop() is called to stop ant slewing associated with a GOTO command
 *
 * Passed
 *      nothing
 *
 * Returns
 *      nothing
 */
void
gotoStop(void)
{
        // Stop any motion currently in progress!
        raRunTime = 0;
        decRunTime = 0;

        rateInput.gotoRaRate = SPEED_0_X;
        rateInput.gotoDecRate = SPEED_0_X;
        GOTO_SW_PORT &= ~_BV(GOTO_LED_BIT);
        updateMountSpeed();
}

/* gotoSync() sets the mount position to the last position specified by the
 * controlling PC
 *
 * It also stops any motion in progress
 *
 * Passed
 *      resp            Set the LX200 response to a CM command
 *
 * nothing
 *
 * Returns
 *      nothing
 *
 * Notes:
 *      As this function is used to respond to a sync command (requiring
 *      a response) and as part of a lon
 */
void
gotoSync(uint8_t resp)
{
        // Update current coordinate to specified coordinates
        raDisp = raNext;
        decDisp = decNext;
        
        isSync = 1;     /* Tells the software to return current location
                         * when requested
                         */
        if (resp)
            serialTx("star#");
}

/* gotoInt() is called every 10 ms to do the goto processing
 *
 * Passed
 *      nothing
 *
 * Returns
 *      nothing
 *
 */
void
gotoInt(void)
{
    uint8_t             raMoving;
    uint8_t             decMoving;

    static uint8_t      tDiv;

    // FInd out which axis are moving
    stepperInMotion(&raMoving, &decMoving);

    /* Maintain the time divider. A value of zero indicates we've
     * crossed a second boundary, which we use to do a rough update
     * of the RA and DEC
     */
    if (++tDiv == 100)
        tDiv = 0;

    /* Turn GOTO LED off */
    GOTO_SW_PORT &= ~_BV(GOTO_LED_BIT);
    
    /* Update DEC */
    if (decMoving)
    {
        // If the dec timer has expiring then stop the goto motion
        switch (decRunTime)
        {
        case 0:     // Timer expired - nothing to do
            break;

        case 1:     // Timer about to expire - stop motion
            rateInput.gotoDecRate = SPEED_0_X;
            updateMountSpeed();
            decDisp = decNext;

            decRunTime = 0;
            break;

        default:    // Still running
            --decRunTime;

            GOTO_SW_PORT |= _BV(GOTO_LED_BIT);  // Turn GOTO LED on

            if (tDiv == 0)
            {
                /* Roughly modify the displayed angle that goes back
                 * to the contoller so it can see we're moving
                 */
                decDisp += decDir * 15 * ((int32_t)gotoRate);

                if (decDisp >= DEC_MODULUS)
                    decDisp -= DEC_RANGE;

                if (decDisp <= -DEC_MODULUS)
                    decDisp += DEC_RANGE;
            }
        }
    }

    if (raMoving)
    {
        switch (raRunTime)
        {
        case 0:     // Timer expired - nothing to do
            break;

        case 1:     // Timer about to expire - stop motion
            rateInput.gotoRaRate = SPEED_0_X;
            updateMountSpeed();
            raDisp = raNext;

            raRunTime = 0;
            break;

        default:    // Still running
            --raRunTime;
            
            GOTO_SW_PORT |= _BV(GOTO_LED_BIT);  // Turn GOTO LED on

            if (tDiv == 0)
            {
                /* Roughly modify the displayed angle that goes back
                 * to the contoller so it can see we're moving
                 */
                raDisp += raDir * effRaGotoRate;

                if (raDisp > RA_MODULUS)
                    raDisp -= RA_RANGE;

                if (raDisp < 0)
                    raDisp += RA_RANGE;
            }
        }
    }
}

/* gotoInit() is called to initialize the goto functions, mainly the
 * switch input used to decide which yhe DEC axis should be driven during
 * GOTO
 *
 * Passed
 *      Nothing
 *
 * Returns
 *      Nothing
 */
void
gotoInit(void)
{
    // Setup switch bit with pullup
    GOTO_SW_DDR &= ~_BV(GOTO_SW_BIT);
    GOTO_SW_PORT |= _BV(GOTO_SW_BIT);
    
    GOTO_SW_DDR |= _BV(GOTO_LED_BIT);
    GOTO_SW_PORT &= ~_BV(GOTO_LED_BIT);

    // Set default GOTO parameters
    gotoRate = 320;
    gotoSpeed = SPEED_320_X;

    gotoDeltaSync = 0;
}
