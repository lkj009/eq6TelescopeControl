#ifndef _DRIVER_H_
#define _DRIVER_H_

/* This file contains the interface for the actual stepper motor driver */

/* Bit use to connect to the "magic" relay that determines which
 * set ot RA coils are connected. This allows low current operation when
 * not slewing at high speed
 */
#define MAGIC_PORT      PORTC
#define MAGIC_DDR       DDRC
#define MAGIC_BITS      (_BV(PC0))   // EQ6

// Arduino pin names for interface to 74HCT595 latch

#define RA_DDR  DDRD
#define RAPORT  PORTD
#define DEC_DDR DDRB
#define DECPORT PORTB

/* Interface prototypes */
void driverInit(void);
#endif /* _DRIVER_H_ */
