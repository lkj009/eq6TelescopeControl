#include "Arduino.h"
#include "combine.h"
#include "stepper.h"
#include "eq6.h"
#include "serial.h"

/* Instance of rate input/output variables. Normally we would pass these via
 * pointers, but this is a small code/data/environment, so they're
 * accessed globally
 */
struct rateInput_s    rateInput = {SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X, SPEED_0_X};
struct rateOutput_s rateOutput = {SPEED_0_X, SPEED_0_X};

/* TEST: defeat tracking for testing */
uint8_t         noTrack = 0;

/* updateMountSpeed() takes the various speed inputs from the paddle,
 * serial port, and guiding inputs and determines the actual RA and
 * DEC rates
 *
 * Passed:
 *         nothing
 *
 * Returns:
 *         nothing
 *
 * Notes:
 *         Updates rateOutput global structure
 */
void
updateMountSpeed(void)
{
    int8_t      guideRate;
#if 0
    str = getBuff();
    sprintf(str, "updateMountSpeed");
    putstr(str);
#endif
    /* Determine the DEC rate. This is the simple one! */
    if (rateInput.gotoDecRate != SPEED_0_X)
        guideRate = rateInput.gotoDecRate;
    else if (rateInput.paddleDecRate != SPEED_0_X)
        guideRate = rateInput.paddleDecRate;
    else if (rateInput.serialDecRate != SPEED_0_X)
        guideRate = rateInput.serialDecRate;
    else if (rateInput.guideDecRate != SPEED_0_X)
        guideRate = rateInput.guideDecRate;
    else
        guideRate = SPEED_0_X;

    rateOutput.decRate = guideRate;
    setDecSpeed(rateOutput.decRate);
#if 0
    str = getBuff();
    sprintf(str, "guideRate:%d",guideRate);
    putstr(str);
#endif
    /* Determine the RA rate. This is complicated by the need to perform
     * tracking as well as guiding on this axis
     */
    if (rateInput.gotoRaRate != SPEED_0_X)
        guideRate = rateInput.gotoRaRate;
    else if (rateInput.paddleRaRate != SPEED_0_X)
        guideRate = rateInput.paddleRaRate;
    else if (rateInput.serialRaRate != SPEED_0_X)
        guideRate = rateInput.serialRaRate;
    else if (rateInput.guideRaRate != SPEED_0_X)
        guideRate = rateInput.guideRaRate;
    else
        guideRate = SPEED_0_X;
#if 0
    str = getBuff();
    sprintf(str, "guideRate:%d",guideRate);
    putstr(str);
#endif
    /* Now we need to add the traking rate to the guiding rate. Fractional
     * guiding rates simply adjust the tracking rate, x1 guiding
     * doubles/stops the motion, higher tracking rate override the
     * guiding rate.
     */
    if (noTrack || (guideRate > SPEED_1_X) || (guideRate < -SPEED_1_X))
        rateOutput.raRate = guideRate;
    else if ((guideRate < SPEED_1_X) && (guideRate > -SPEED_1_X))
        rateOutput.raRate = rateInput.siderialRate + guideRate;
    else if ((guideRate == SPEED_1_X) && (rateInput.siderialRate == SPEED_1_X))
        rateOutput.raRate = SPEED_2_X;
    else if ((guideRate == -SPEED_1_X) && (rateInput.siderialRate == -SPEED_1_X))
        rateOutput.raRate = -SPEED_2_X;
    else
        rateOutput.raRate = SPEED_0_X;

    /* The RA axis needs to turn in the opposite direction of the
     * DEC axis. This is the simplest place to do it
     */
    setRaSpeed(-rateOutput.raRate);

    /* All done! */
}
