#ifndef _GOTO_H_
#define _GOTO_H_

#include "stepper.h"

/* Define the bits with the switch that controls the DEC direction
 */

#define GOTO_SW_PORT    PORTC
#define GOTO_SW_PIN     PINC
#define GOTO_SW_DDR     DDRC

#define GOTO_SW_BIT     2
#define GOTO_LED_BIT    3


/* Prototypes for the goto functions */

void    gotoOutputRA(void);
void    gotoOutputDec(void);
void    gotoTogglePrecision(void);

void    gotoSetRA(char *pStr);
void    gotoSetDec(char *pStr);
void    gotoTarget(void);
void    gotoSync(uint8_t resp);
void    gotoStop(void);

void gotoInt(void);
void gotoInit(void);

// Save/restored controls for GOTO rate
extern uint16_t  gotoRate;               // Numeric speed
extern uint8_t  gotoSpeed;              // Speed specifier ie. SPEED_24_X
extern uint8_t  gotoDeltaSync;          // Synchronise to target of long slew

#endif /* _GOTO_H_ */
