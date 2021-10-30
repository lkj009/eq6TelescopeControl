#include "Arduino.h"
#include "paddle.h"
#include "combine.h"
#include "stepper.h"
#include "debug.h"
#include "serial.h"

/* Prototypes for paddle.c functions that can be called from outside
 */
uint8_t  paddleGuideRate = SPEED_1_X;

 
/* paddleInit() initializes the serial port used for the EQ6/Atlas
 * hand paddle. The communication with this port is only in one
 * direction (from paddle to mount).
 *
 * Passed:
 *         Nothing
 *
 * Returns:
 *         Nothing.
 */
void
paddleInit(void)
{

}

void paddleProcess(uint8_t c) {
    uint8_t             lowerByte;

    static void         *pLabel = &&wait_for_sync;

    static uint8_t      raDir = 0;
    static uint8_t      raSpeed = 0;
    static uint8_t      decDir = 0;
    static uint8_t      DecSpeed = 0;

    static uint8_t      oldRaDir = (uint8_t)-1;
    static uint8_t      oldRaSpeed = (uint8_t)-1;
    static uint8_t      oldDecDir = (uint8_t)-1;
    static uint8_t      oldDecSpeed = (uint8_t)-1;

    lowerByte = c;
    /* Jump to the code that handles the current reception system. Note that
     * this feature is GCC specific.
     */
    goto *pLabel;

    /* Wait for the RA sync byte
     */
wait_for_sync:
    if (lowerByte == RA_SYNC){
        pLabel = &&get_ra_direction;
    }
    return;

    /* Get the RA direction - this will have b8 == 0
     */
get_ra_direction:
    raDir = lowerByte & _BV(RA_DIR_BIT);
    pLabel = &&get_ra_speed;
    return;

    /* Get the RA speed - this will have b8 == 0
     */
get_ra_speed:
    raSpeed = lowerByte & RA_SPEED_MASK;
    pLabel = &&get_dec_sync;
    return;

    /* Get the mystery fourth byte
     */
get_dec_sync:
    pLabel = &&get_dec_direction;
    return;

    /* Get the DEC direction - this will have b8 != 0
     */
get_dec_direction:
    decDir = lowerByte & _BV(DEC_DIR_BIT);
    pLabel = &&get_dec_speed;
    return;

    /* Get the RA speed - this will have b8 == 0
     */
get_dec_speed:
    // We've got all the words - the next state will always be wait for sync
    pLabel = &&wait_for_sync;
    DecSpeed = lowerByte & DEC_SPEED_MASK;

    /* If the parity is correct and any of the bytes are different then
     * process them
     */
    if ((oldRaDir != raDir) 
        || (oldRaSpeed != raSpeed) 
        || (oldDecDir != decDir) 
        || (oldDecSpeed != lowerByte))
    {
        /* Update "old" indications */
        oldRaDir = raDir;
        oldRaSpeed = raSpeed;
        oldDecDir = decDir;
        oldDecSpeed = DecSpeed;


        /* Process the data from the paddle
         * - if the RA speed is one then setup the siderial speed
         * - convert the RA speed/direction into a device independent form
         * - convert the DEC speed/direction into a device independent form
         */
        if (raSpeed == RA_SPEED_0)
        {
            rateInput.paddleRaRate = -paddleGuideRate;
            
            /* This is a little trick because the "direction" bit doesn't
             * work then the speed is zero. Do a pre-invert if the
             * siderial rate is for the northern hemisphere
             */
            if (rateInput.siderialRate == SPEED_SIDERIAL)
                rateInput.paddleRaRate = -rateInput.paddleRaRate;
        }
        else if (raSpeed == RA_SPEED_1)
        {
            rateInput.paddleRaRate = SPEED_0_X;
            rateInput.siderialRate = raDir ? SPEED_SIDERIAL : -SPEED_SIDERIAL;
        }
        else if (raSpeed == RA_SPEED_2)
            rateInput.paddleRaRate = paddleGuideRate;
        else if (raSpeed == RA_SPEED_8)
            rateInput.paddleRaRate = SPEED_8_X;
        else if (raSpeed == RA_SPEED_16)
            rateInput.paddleRaRate = SPEED_16_X;

#if 0
        /* The direction of the RA keys is reversed when operating in
         * the southern hemisphere, so modify them back
         */
        if ((raDir == 0) ^ (rateInput.siderialRate == SPEED_SIDERIAL))
             rateInput.paddleRaRate = -rateInput.paddleRaRate;
#else
        /* Use the keys as returned by the paddle. These are reversed when
         * operating in the southern hemisphere. This ensures that the
         * "right" key always increases the RA angle
         */
        if (raDir == 0)
             rateInput.paddleRaRate = -rateInput.paddleRaRate;
#endif

        if (lowerByte == DEC_SPEED_0)
            rateInput.paddleDecRate = SPEED_0_X;
        else if (lowerByte == DEC_SPEED_2)
            rateInput.paddleDecRate = paddleGuideRate;
        else if (lowerByte == DEC_SPEED_8)
            rateInput.paddleDecRate = SPEED_8_X;
        else if (lowerByte == DEC_SPEED_16)
            rateInput.paddleDecRate = SPEED_16_X;

        if (decDir != 0)
            rateInput.paddleDecRate = -rateInput.paddleDecRate;

        /* Ok, all parsed. Force an update of the combined speed
         */
          updateMountSpeed();
    }
}
