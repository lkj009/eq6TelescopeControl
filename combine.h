#ifndef _COMBINE_H_
#define _COMBINE_H_

/* This file gives the "public" interface for the "combining" function
 * that takes the various inputs (siderial rate, paddle buttons, serial
 * input, guider input) to derive speeds for the RA and DEC axis
 */

/* Speed definitions */
#define SPEED_0_X       0
#define SPEED_0_33_X    1
#define SPEED_0_67_X    2
#define SPEED_1_X       3
#define SPEED_1_33_X    4
#define SPEED_1_67_X    5
#define SPEED_2_X       6
#define SPEED_4_X       7
#define SPEED_8_X       8
#define SPEED_16_X      9
#define SPEED_24_X      10
#define SPEED_320_X      11

#define SPEED_SPIN      (SPEED_16_X + 1)
#define SPEED_SIDERIAL  SPEED_1_X

struct rateInput_s
{
  int8_t    siderialRate;

  int8_t          gotoRaRate;
  int8_t          gotoDecRate;

  int8_t    paddleRaRate;
  int8_t    paddleDecRate;

  int8_t    serialRaRate;
  int8_t    serialDecRate;

  int8_t    guideRaRate;
  int8_t    guideDecRate;
};

extern struct rateInput_s rateInput;

struct rateOutput_s
{
  int8_t    raRate;
  int8_t    decRate;
};

extern struct rateOutput_s rateOutput;

void updateMountSpeed(void);

/* TEST: defeat tracking for testing */
extern uint8_t          noTrack;

#endif /* _COMBINE_H_ */
