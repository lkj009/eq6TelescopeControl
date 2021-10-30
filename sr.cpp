#include "Arduino.h"
#include "sr.h"
#include "stepper.h"
#include "paddle.h"
#include "serial.h"
#include "goto.h"
#include <EEPROM.h>

/* Define the staging structure used to save configurable parameters */
struct sr_s
{
    uint8_t     raFinPos;
    uint8_t     raFinNeg;
    uint16_t    raBacklash;
    int8_t      raDir;

    uint8_t     decFinPos;
    uint8_t     decFinNeg;
    uint16_t    decBacklash;
    int8_t      decDir;

    uint8_t     paddleGuideRate;

    int8_t      trackingRate;

    struct trackRate_s      trackRateTable[NUM_RATES];
#ifdef _GOTO_H_
    uint16_t     gotoRate;
    uint8_t     gotoSpeed;
    uint8_t     gotoDeltaSync;
#endif /* _GOTO_H_ */
#ifdef _PGUIDE_H
    uint8_t     polarLevel;
#endif /* _PGUIDE_H */
    uint8_t     csum;   // Two's complement of other fields
};

volatile uint8_t        doSave;

/* srSaveState() collects the configurable parameters from all over the
 * system and saves them into the EEPROM
 *
 * Passed
 *      Nothing
 *
 * Returns
 *      Nothing
 *
 * Notes:
 *      This function assumes that the interrupts are enabled on
 *      entry.
 */
void
srSaveState(void)
{
    static struct sr_s          state;
    static uint8_t              i;
    static uint8_t              *p;

    static uint8_t              step = 0;

    /* This code is structured so that the save occurs over several executions
     * so that other processing (such as serial processing) can continue
     */

    switch(step)
    {
    case 0:
        if (doSave)
        {
            doSave = 0;
            ++step;
        }
        break;

    case 1:   
        /* Turn off the interrupts while we "snapshot" the current
         * state
         */
        cli();

        /* Copy the current state */
        state.raFinPos = raState.finPos;
        state.raFinNeg = raState.finNeg;
        state.raBacklash = raState.backlash;
        state.raDir = raState.dir;

        state.decFinPos = decState.finPos;
        state.decFinNeg = decState.finNeg;
        state.decBacklash = decState.backlash;
        state.decDir = decState.dir;

        state.paddleGuideRate = paddleGuideRate;
        state.trackingRate = trackingRate;
        memcpy(state.trackRateTable, trackRateTable, sizeof(trackRateTable));
#ifdef _GOTO_H_
        state.gotoSpeed = gotoSpeed;
        state.gotoRate = gotoRate;
        state.gotoDeltaSync = gotoDeltaSync;
#endif /* _GOTO_H_ */
#ifdef _PGUIDE_H
        state.polarLevel = polarLevel;
#endif /* _PGUIDE_H */
        /* Reenable interrupts */
        sei();

        ++step;
        break;

    case 2:
        /* Calculate the checksum of the structure */
        for (p = (uint8_t *)&state, i = 0 ; p != (uint8_t *)&state.csum ; )
             i += *p++;
        state.csum = i;

        // Setup for write pass
        p = (uint8_t *)&state;
        i = 0;

        ++step;         // Do next step next execution
        break;

    case 3:
        /* Write the complete structure into the EEPROM */

        /* If the EEPROM is not ready for the next byte then do nothing */
        if ( ! eeprom_is_ready())
            break;

        /* Write a byte and increment to the next position */
        EEPROM.write(i, *p);
        ++i;
        ++p;

        /* Go back to state 0 if done */
        if (i == sizeof(state))
        {
            step = 0;
        }
#if 0
        serialTx("EEPROM write finish");
#endif
        break;
    }
    
    /* Done, at least until the next execution */
}

/* srLoadState() reads the configurable parameters from EEPROM and copies
 * them to where they need to do.
 *
 * The data is protected by a checksum calculated when the data was
 * saved.
 *
 * Passed
 *      Nothing
 *
 * Returns
 *      0       Restore OK
 *      Else    Restore failed
 *
 * Notes:
 *      This interrupt does not change the interrupt state and does
 *      not return until all the data is read
 */
uint8_t
srLoadState(void)
{
    struct sr_s         state;
    uint8_t             i;
    uint8_t             *p;

    /* Read the state data from the EEPROM */
    while ( ! eeprom_is_ready())
        ;
    
    EEPROM.get(0, state);

    /* Calculate the checksum of the structure. If it doesn't
     * match then the data is either corrupt or not initialized
     *
     * Either way keep the current data
     */
    for (p = (uint8_t *)&state, i = 0 ; p != (uint8_t *)&state.csum ; )
         i += *p++;
    if (state.csum != i)
        return 1;
    /* Copy the restored data to the current state */
    raState.finPos = state.raFinPos;
    raState.finNeg = state.raFinNeg;
    raState.backlash = state.raBacklash;
    raState.dir = state.raDir;

    decState.finPos = state.decFinPos;
    decState.finNeg = state.decFinNeg;
    decState.backlash = state.decBacklash;
    decState.dir = state.decDir;
    
    paddleGuideRate = state.paddleGuideRate;
    trackingRate = state.trackingRate;
   // memcpy(trackRateTable, state.trackRateTable, sizeof(trackRateTable));

#ifdef _GOTO_H_
    gotoRate = state.gotoRate;
    gotoSpeed = state.gotoSpeed;
    gotoDeltaSync = state.gotoDeltaSync;
#endif /* _GOTO_H_ */
#ifdef _PGUIDE_H
    polarLevel = state.polarLevel;
#endif /* _PGUIDE_H */
#if 0
    str = getBuff();
    sprintf(str, "SR gotoSpeed:%d",gotoSpeed);
    putstr(str);
#endif
    /* All done */
    return 0;
}
