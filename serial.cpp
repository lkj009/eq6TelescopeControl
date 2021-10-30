/* This file accepts serial data from a serial guider and performs the
 * requested guiding operation.
 *
 * This interface is also is designed to support future mount management
 * operations via the same serial interface
 *
 * The current protocol is:
 *
 * [0-9]    Add the number to the "accumulator"
 * -        Denote accumulator as negative
 * #        Clear the accumulator and flag
 * <        Clear the flag
 * >        Set the flag
 * U        Copy flag state to the DEC UP key
 * D        Copy flag state to the DEC DOWN key
 * L        Copy flag state to the RA left key
 * R        Copy flag state to the RA right key
 * C        Release all keys
 *
 * B        Set DEC backlash to accum
 * M        Set DEC backlash mode (+ = finish pos, 0 = none, - = finish neg)
 * b        Set RA backlash to accum
 * m        Set RA backlash mode (see DEC backlash)
 *
 * O        Reverse DEC motion if flag set
 * o        Reverse RA motion if flag set
 *
 * G        Set 2X paddle to guiding speed (0.3x) if flag set, 1X if clear
 *
 * S        Set the slewing speed used for GOTO (TRUE = 24x, FALSE = 16x)
 *
 * t        Use halfStep for slow step if flag set, microStep if clear (TEST)
 *
 * T        Set the tracking rate as-per the accumulator
 *          -1 = No tracking (Terrestial)
 *          0 = Sidereal
 *          1 = Solar / Planetary
 *          2 = Lunar
 *
 * P        Set the polar scope illumination level from accum 0..255
 *
 * g        Set transmission (gearbox) ratio [retired - no operation]
 *
 * c        Set "cursor" position for tracking table update to accumulator
 * s        Set divisor for tracking rate clock @'cursor' to accumulator
 * a        Set adjustment (one tick [+]dropped/[-]added every accum ticks)
 * p        Set 360deg rotation time for tracking entry
 *
 * d        Set delta sync enable as per flag
 *
 * The '#' and accumulator support future value-based entries
 *
 * A subset of LX200 commands are supported for compatibility
 * with various autoguiding programs.
 *
 * :Me# :Mw# :Ms# :Mn#          Start slewing East(right), West(left),
 *                              North (up), South (down)
 * :Qe# :Qw# :Qs# :Qn# :Q#      Halt selected or all slews
 * :RG# :RC# :RM# :RS#          Set guiding rate for LX200 motions
 *                              G = 0.3x
 *                              C = 1x
 *                              M = 8x
 *                              S = 16x
 * :Mg[snew]nnnn#               Pulseguide. Move in specified direction
 *                              for nnnn milliseconds and return to
 *                              tracking (for RA) or stop (for DEC).
 *
 *                              Conflicting movements are resolved (per axis)
 *                              by the later command overriding the earlier
 *                              one.
 *                                      
 * ACK                          Returns alignment mode (always 'P' polar)
 *
 * :GVD# :GVN# :GVP# :GVT:      Identification info
 *
 * A subset of the LX200 commands are supported for GOTO support
 *
 * :GD#                         Get DEC from mount in sDD*HH# or sDD*HH'SS#
 *                              form (depends on :U# output precision)
 * :GR#                         Get RA from mount in HH:MM.T# or HH:MM:SS#
 *                              form (depends on :U# output precision)
 * :U#                          Toggle output precision
 *
 * :SdsDD*MM#                   Set DEC of next GOTO
 * :SdsDD*MM:SS#                Set DEC of next GOTO
 *
 * :SrHH:MM.T#                  Set RA of next GOTO
 * :SrHH:MM.SS#                 Set RA of next GOTO
 *
 * :MS#                         Execute the GOTO.
 * :CM#                         Sync scope to current position
 */
#include "Arduino.h"
#include "serial.h"
#include "paddle.h"
#include "combine.h"
#include "stepper.h"
#include "sr.h"
#include "eq6.h"
#include "goto.h"
//#include <SoftwareSerial.h>
//SoftwareSerial mySerial(3, 2); // RX, TX
char buff[100] = {0, };
char* str = NULL;
char cUDR1 = '!';

volatile uint16_t raPulseTime;
volatile uint16_t decPulseTime;

void
serialInit(void)
{
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  //Serial.println("Good night~~!");
 
  // set the data rate for the SoftwareSerial port
  Serial3.begin(PADDLE_RATE);
}

uint8_t readSerial(void) {
  return Serial3.read();
}

int serialAvailable(void) {
  return Serial3.available();
}

void putch(char c) {
  Serial.println(c);
}

void
putstr(char *pStr) {
  Serial.println(pStr);
}

char* getBuff(void) {
  memset(buff, 0x0, sizeof(buff));
  return buff;
}

/* serialTx() is called to send a string of characters on the scope UART
 *
 * Passed:
 *      pStr            Null-terminated string to send
 *
 * Returns:
 *      0               All OK
 *      non-zero        String alread being sent
 */
char
serialTx(const char *pStr)
{
  Serial.print(pStr);
  return 0;
}

#define MAX_RX_QUEUED   64
static char             rxQueue[MAX_RX_QUEUED];
unsigned char           rxQWr;
unsigned char           rxQRd;

/* serialRxInt() is called whenever a character is received on USART1.
 * These characters are placed into a queue for later processing
 *
 * Passed:
 *         Nothing
 *
 * Returns:
 *         Nothing
 *
 * Notes:
 *         Interrupts are disabled during processing
 */
void sigSerialReceive(char c) {
    unsigned char       newRxQWr;

    /* Get the character from the port. This will dismiss the interrupt
     */
    cUDR1 = c;

    /* Push it into the RX queue */
    rxQueue[rxQWr] = cUDR1;

    /* Update pointer if there was room */
    newRxQWr = (rxQWr + 1) % MAX_RX_QUEUED;
    if (newRxQWr != rxQRd)
        rxQWr = newRxQWr;
    else
        cUDR1 = '!';  
}

/* rxProcessing() is called to process any received characters in the
 * queue.
 *
 * Passed
 *      nothing
 *
 * Returns
 *      nothing
 */
void
rxProcessing(void)
{
    /* Variables for flags/accumulator */
    static uint8_t      flag = 0;
    static uint32_t     accum = 0;
    static uint8_t      signNeg = 0;
    static uint16_t     cursor = (uint16_t)-1;

    /* Variable holding current guiding rate */
    static int8_t       guideRate = SPEED_0_33_X;

    /* Flags holding current requested slewing directions */
    static uint8_t      upFlag;
    static uint8_t      downFlag;
    static uint8_t      leftFlag;
    static uint8_t      rightFlag;

    /* LX200 command state */
#define LX200_CMD_LEN   16
    static char         lxCmd[LX200_CMD_LEN];
    static uint8_t      lxPos;

    uint8_t             ch;
    uint8_t             raChg = 0;              // RA direction changed
    uint8_t             decChg = 0;             // DEC direction changed
    uint8_t             timed = 0;              // Change is timed move

    static uint8_t      lxMode;

    /* Get the character from the queue. If there is no character then
     * stop processing
     */
    if (rxQRd == rxQWr)
        return;

    ch = rxQueue[rxQRd];
    rxQRd = (rxQRd + 1) % MAX_RX_QUEUED;
    
    /* This code processes commands when a LX200 command is not currently
     * being processed.
     */
    if (lxMode == 0)
    {
        switch(ch)
        {
        case '0' ... '9':
            /* Add it to the accumulator */
            accum = (accum * 10) + ch - '0';
 //Test {
            rateInput.paddleRaRate = ch - '0';
            updateMountSpeed();
 //Test }
            break;

        case '#':
            /* Clear the accumulator */
            accum = 0;
            signNeg = 0;
            break;

        case '-':
            /* Set the sign of the accumulator to negative */
            signNeg = 1;
            break;

        case '<':
            /* Clear the flag */
            flag = 0;
            break;

        case '>':
            /* Set the flag */
            flag = 1;
            break;

        case 'U':
            /* Guide UP (DEC) */
            upFlag = flag;
            decChg = 1;
            break;

        case 'D':
            /* Guide DOWN (DEC) */
            downFlag = flag;
            decChg = 1;
            break;

        case 'R':
            /* Guide RIGHT (RA) */
            rightFlag = flag;
            raChg = 1;
            break;

        case 'L':
            /* Guide LEFT (RA) */
            leftFlag = flag;
            raChg = 1;
            break;

        case 'C':
            /* Clear all keys */
            upFlag = downFlag = leftFlag = rightFlag = 0;
            raChg = decChg = 1;
            break;

        case 'b':
            /* Set RA backlash steps */
            raState.backlash = accum;
            doSave = 1;
            break;

        case 'B':
            /* Set DEC backlash steps */
            decState.backlash = accum;
            doSave = 1;
            break;

        case 'm':
            /* Set RA backlash mode */
            raState.finNeg = raState.finPos = 0;
            if (accum != 0)
            {
                if (signNeg)
                    raState.finNeg = 1;
                else
                    raState.finPos = 1;
            }
            doSave = 1;
            break;

        case 'M':
            /* Set DEC backlash mode */
            decState.finNeg = decState.finPos = 0;
            if (accum != 0)
            {
                if (signNeg)
                    decState.finNeg = 1;
                else
                    decState.finPos = 1;
            }
            doSave = 1;
            break;
        
        case 'o':      /* Reverse RA direction */
            raState.dir = flag ? -1 : 1;
            doSave = 1;
            break;

        case 'O':      /* Reverse DEC direction */
            decState.dir = flag ? -1 : 1;
            doSave = 1;
            break;

        case 'G':
            /* Set the speed for the 2x paddle button. This has
             * no effect on the rate used for serial commands
             */
            paddleGuideRate = flag ? SPEED_0_33_X : SPEED_1_X;
            doSave = 1;
            break;

        case 'T':
            /* Set the tracking speed */
            setTrackRate(signNeg ? -accum : accum);
            doSave = 1;
            break;

        case 'g':       /* Set transmission (gearbox) ratio */
            // This command has been retired - do nothing
            break;

        case 'P':       /* Set polar scope illumination level */
#ifdef _POLAR_H_
            polarLevel = (accum < 256) ? accum : 255;
            doSave = 1;
#endif /* _POLAR_H_ */
            break;

        case 'c':       /* Set cursor to accumulator */
            cursor = accum;
            break;

        case 's':       /* Set rate divisor for tracking rate 'cursor' */
            if (cursor < NUM_RATES)
            {
                trackRateTable[cursor].div = accum;
                doSave = 1;
            }
            break;

        case 'a':       /* Set adjustment for tracking rate 'cursor' */
            if (cursor < NUM_RATES)
            {
                trackRateTable[cursor].adj = accum;
                trackRateTable[cursor].doDropInt = ! signNeg;
                doSave = 1;
            }
            break;

        case 'p':       /* Set period for tracking rate 'cursor' */
            if (cursor < NUM_RATES)
            {
                trackRateTable[cursor].secsPerRotation = accum;
                doSave = 1;
            }
            break;

        case 'S':       /* Get GOTO slewing speed */
#ifdef _GOTO_H_
            if (flag)
            {
                /* Use the faster GOTO speed. This is the default. Use
                 * this instead of the slow rate if the mount can handle it
                 */
                gotoRate = 320;
                gotoSpeed = SPEED_320_X;
            }
            else
            {
                /* Use a slower GOTO slew rate. This may be necessary if
                 * the mount if heavily loaded, or badly balanced
                 */
                gotoRate = 320;
                gotoSpeed = SPEED_320_X;
            }
            doSave = 1;
#endif /* _GOTO_H_ */
            break;

        case 'd':      /* Control delta sync */
#ifdef _GOTO_H_
            gotoDeltaSync = flag;
            doSave = 1;
#endif /* _GOTO_H_ */
            break;

        case 't':
            /* *TEST* Allow half step table to be specified instead of
             * the microstep table
             */
            doHalfStep = flag;
            break;

        case '\006':        /* LX200: ACK */
            serialTx("P");
            break;

        case ':':           /* LX200: Start command */
            /* This indicates the start of a LX200 command */
            lxMode = 1;
            lxPos = 0;
            break;

        default:
            /* OK, now we're confused .... */
            cUDR1 = '?';
            break;
        }
    }
    else
    {
        /* This section of code supports the LX200 commands. They
         * have a fundimentally different syntax to the existing
         * commands, so they are implemented in this code seperately
         * for clarity.
         */
        if (ch != '#')
        {
            /* Add this to the command characters so far */
            lxCmd[lxPos++] = ch;
            if (lxPos == LX200_CMD_LEN)
            {
                /* Too much data for any command */
                cUDR1 = '?';
                lxMode = 0;
            }
        }
        else
        {
            /* We are going back to native mode after this */
            lxMode = 0;

            if (lxPos >= 1)
            {
                /* We have a complete LX200 command (without the delimiters).
                 * Do what it asks
                 */
                switch(lxCmd[0])
                {

                case 'C':       // Sync commands
                    if (lxPos == 2 && lxCmd[1] == 'M')
                    {
#ifdef _GOTO_H_
                         gotoSync(1);
#endif /* _GOTO_H_ */
                    }
                    else
                        cUDR1 = '?';
                    break;

                case 'G':       // Get mount position information
                    if (lxPos == 2)
                    {
                        // We have the right #chars - do it
                        switch (lxCmd[1])
                        {
                        case 'D':       // Get current scope DEC
#ifdef _GOTO_H_
                            gotoOutputDec();
#endif /* _GOTO_H_ */
                            break;
                            
                        case 'R':       // Get current scope RA
#ifdef _GOTO_H_
                            gotoOutputRA();
#endif /* _GOTO_H_ */
                            break;

                        case 'S':       // Output siderial time
                            /* ASCOM driver wants this from Meade/Autostar.
                             * Fake it.
                             */
                            serialTx("00:00:00#");
                            break;


                        default:
                            cUDR1 = '?';
                        }
                    }
                    else if (lxPos == 3 && lxCmd[1] == 'V')
                    {
                        switch (lxCmd[2])
                        {
                        case 'D':       // Get firmware date
                            serialTx(FIRMWARE_DATE "#");
                            break;

                        case 'N':       // Get firmware revision
                            serialTx(FIRMWARE_REV "#");
                            break;

                        case 'P':       // Get product name
                            serialTx(PRODUCT_NAME "#");
                            break;

                        case 'T':       // Get firmware time
                            serialTx(FIRMWARE_TIME "#");
                            break;

                        default:
                            cUDR1 = '?';
                        }
                    }
                    break;

                case 'M':       // Start guiding in specified direction
                    if (lxPos == 2)
                    {
                        // We have the right number of chars */
                        switch (lxCmd[1])
                        {
                        case 'n': upFlag = 1; downFlag = 0; decChg = 1; break;
                        case 's': upFlag = 0; downFlag = 1; decChg = 1; break;
                        case 'e': rightFlag = 1; leftFlag = 0; raChg = 1; break;
                        case 'w': rightFlag = 0; leftFlag = 1; raChg = 1; break;

                        case 'S':
                            // GOTO the specified position
#ifdef _GOTO_H_
                            gotoTarget();
#endif /* _GOTO_H_ */
                            break;

                        default: cUDR1 = '?'; break;
                        }
                    }
                    else if (lxPos >= 4 && lxCmd[1] == 'g')
                    {
                        uint16_t     pulseTime;

                        // This is a pulseguide (timed guide correction) command
                        lxCmd[lxPos] = '\0';
                        pulseTime = atoi(lxCmd + 3);
                        
                        switch (lxCmd[2])
                        {
                        case 'n': 
                            upFlag = 1;
                            downFlag = 0;
                            decChg = 1;
                            break;
                        case 's': 
                            upFlag = 0; 
                            downFlag = 1; 
                            decChg = 1;
                            break;
                        case 'e': 
                            rightFlag = 1; 
                            leftFlag = 0; 
                            raChg = 1;
                            break;
                        case 'w': 
                            rightFlag = 0; 
                            leftFlag = 1; 
                            raChg = 1;
                            break;
                        }

                        /* Clear all movement flags if time == 0. The raChg
                         * and decChg flags will cause any movement in
                         * the specified axis to be stopped
                         */
                        if (pulseTime == 0)
                             upFlag = downFlag = leftFlag = rightFlag = 0;
                        else
                             timed = 1;

                        /* Assign RA and DEC timer values depending on what
                         * the pulseguide requested
                         */
                        if (raChg)
                        {
                            raPulseTime = pulseTime;
                        }

                        if (decChg)
                        {
                            decPulseTime = pulseTime;
                        }

                    }
                    break;

                case 'Q':       // Stop guiding in specified direction
                    if (lxPos == 1)
                    {
                        // Stop slewing
                        upFlag = downFlag = leftFlag = rightFlag = 0;
                        raChg = decChg = 1;

                        // Stop goto
#ifdef _GOTO_H_
                        gotoStop();
#endif /* _GOTO_H_ */
                    }
                    else if (lxPos == 2)
                    {
                        // Stop slewing is specified direction
                        switch (lxCmd[1])
                        {
                        case 'n': upFlag = 0; decChg = 1; break;
                        case 's': downFlag = 0; decChg = 1; break;
                        case 'e': rightFlag = 0; raChg = 1; break;
                        case 'w': leftFlag = 0; raChg = 1; break;
                        default: cUDR1 = '?'; break;
                        }
                    } else
                        cUDR1 = '?';
                    break;

                case 'R':           // Set guiding speed
                    if (lxPos == 2)
                    {
                        switch (lxCmd[1])
                        {
                        case 'G':   guideRate = SPEED_0_33_X; break;
                        case 'C':   guideRate = SPEED_1_X; break;
                        case 'M':   guideRate = SPEED_8_X; break;
                        case 'S':   guideRate = SPEED_16_X; break;
                        case '1':   break;      // Undocumented: Meade DSI
                        default: cUDR1 = '?'; break;
                        }
                    }
                    break;

                case 'S':       // Set GOTO RA/DEC
                    // Null-terminate the string
                    lxCmd[lxPos] = '\0';

                    // Choose which thing to set
                    switch (lxCmd[1])
                    {
                    case 'r':
#ifdef _GOTO_H_
                        gotoSetRA(lxCmd + 2);
#endif /* _GOTO_H_ */
                        break;

                    case 'd':
#ifdef _GOTO_H_
                        gotoSetDec(lxCmd + 2);
#endif /* _GOTO_H_ */
                        break;

                    default:
                        cUDR1 = '?';
                    }

                    break;

                case 'U':       // Toggle display precision
#ifdef _GOTO_H_
                    if (lxPos == 1)
                         gotoTogglePrecision();
                    else
#endif /* _GOTO_H_ */
                        cUDR1 = '?';
                    break;

                default:
                    cUDR1 = '?';
                }
            }
        }
    }

    /* Update the serial guiding rate data if it has changed */
    if (decChg)
    {
        // Set direction and speed if requested
        if (upFlag)
            rateInput.serialDecRate = guideRate;
        else if (downFlag)
            rateInput.serialDecRate = -guideRate;
        else
            rateInput.serialDecRate = SPEED_0_X;

        // If this move isn't timed, cancel the timer
        if ( ! timed)
            decPulseTime = 0;
        
        updateMountSpeed();
    }

    if (raChg)
    {
        // Set direction and speed if requested
        if (rightFlag)
            rateInput.serialRaRate = guideRate;
        else if (leftFlag)
            rateInput.serialRaRate = -guideRate;
        else
            rateInput.serialRaRate = SPEED_0_X;

        // If this move isn't timed, cancel the timer
        if ( ! timed)
            raPulseTime = 0;
        
        updateMountSpeed();
    }

}

#if 1

/* The following code is for debugging. Disable it to reduce code size in
 * production versions
 */
char    toHex[] = "0123456789abcdef";

void
print16(uint16_t num)
{
    char        out[5];

    out[0] = toHex[(num >> 12) & 0xf];
    out[1] = toHex[(num >> 8) & 0xf];
    out[2] = toHex[(num >> 4) & 0xf];
    out[3] = toHex[(num >> 0) & 0xf];
    out[4] = '\0';

    serialTx(out);
}

void
print32(uint32_t num)
{
    print16(num >> 16);
    print16(num & 0xffff);
}
#endif
