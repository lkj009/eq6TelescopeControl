#ifndef _EQ6_H_
#define _EQ6_H_

/* Define the default clock rate used by the software
 * replacement firmware
 */
#define DEF_CLK_RATE  (16L * 1000L * 1000L) // 16 MHz

#ifndef CLK_RATE
#define CLK_RATE        DEF_CLK_RATE
#endif /* CLK_RATE */

#define TMR2_FREQ       100     // Timer 2 frequency [Hz]. Used for debounce
                                // and pulseguide
/* Define the identification information returned in response to L200
 * :GV commands
 */
#define FIRMWARE_DATE       "DEC 10 2005"       /* MMM DD YYYY */
#define FIRMWARE_TIME       "00:00:00"          /* HH:MM:SS */
#define FIRMWARE_REV        "07.0"              /* nn.n */
#define PRODUCT_NAME        "EQ6 GBDT"
#endif /* _EQ6_H_ */
