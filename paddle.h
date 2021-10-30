#ifndef _PADDLE_H_
#define _PADDLE_H_

/* Define the serial rate used by the paddle
 */
#define PADDLE_RATE  935L    /* Bit rate for the paddle */

/* The serial stream is a set of 6, 9 bit words. The first three words
 * are for RA control, the second three words are for DEC control.
 *
 * Note that the ordering here is arbitary (through reverse engineering), but
 * seems sensible.
 *
 * Word1: RA sync (B8 = 0)
 * Word2: RA direction (B8 = 0)
 * Word3: RA speed (B8 = 0)
 * Word4: ??? (Assumed to be DEC sync) (B8 = 1)
 * Word5: DEC direction (B8 = 1)
 * Word6: DEC speed (B8 = 1)
 *
 */

/* Word #1 - RA SYNC
 */
#define RA_SYNC   0x7e

/* Word #2 - RA DIRECTION
 */
#define RA_DIR_BIT  0 // RA Direction bit
#define RA_DIR_LEFT 0
#define RA_DIR_RIGHT  (1 << RA_DIR_BIT)

/* Note: Will be 0 for southern hemispere siderial, 1 for northern siderial
 */


/* Word #3 - RA SPEED
 */
#define RA_SPEED_MASK 0x0f  // Speed bits
#define RA_SPEED_0  0x00  // x2, opposite direction to siderial
#define RA_SPEED_1  0x01  // No RA button pressed
#define RA_SPEED_2  0x02  // x2, same direction as siderial
#define RA_SPEED_8  0x04  // siderial x8
#define RA_SPEED_16 0x08  // siderial x16

/* Word #4 - ???. This word always seems to be zero. Assume it's a sync word
 * unless we hear something different
 */
#define DEC_SYNC  0x00

/* Word #5 - DEC DIRECTION
 */
#define DEC_DIR_BIT 0 // DEC Direction bit
#define DEC_DIR_UP  0
#define DEC_DIR_DOWN  (1 << DEC_DIR_BIT)

/* Word #6 - DEC SPEED
 */
#define DEC_SPEED_MASK  0x0f  // Speed bits
#define DEC_SPEED_0 0x00  // No buttons pressed
#define DEC_SPEED_2 0x02  // siderial x2
#define DEC_SPEED_8 0x04  // siderial x8
#define DEC_SPEED_16  0x08  // siderial x16

/* Prototypes for paddle.c functions that can be called from outside
 */
extern void paddleInit(void);

extern uint8_t  paddleDecRate;
extern uint8_t  paddleRaRate;
extern uint8_t  siderialRate;

/* Configuration variable - sets whether the 2X rate for the paddle should
 * be considered as 1X or 0.3X
 */
extern uint8_t  paddleGuideRate;

extern void paddleProcess(uint8_t c);

#endif /* _PADDLE_H_ */
