#include "Arduino.h"
#include "eq6.h"
#include "stepper.h"
#include "paddle.h"
#include "driver.h"
#include "sr.h"
#include "combine.h"
#include "serial.h"
#include "goto.h"

void setup() {
    // put your setup code here, to run once:

    serialInit();
    paddleInit();
    stepperInit();
    //driverInit();
    gotoInit();
    /* Load save data (if any) from the EEPROM */
    if (srLoadState()) {
        serialTx("*** EEPROM ERROR ***");
    }
    /* Use saved transmission ratio information */
    setTrackRate(trackingRate);
    #if 0
    //print log {
    str = getBuff();
    sprintf(str, "trackingRate:%d",trackingRate);
    putstr(str);
    // }
    #endif
    //Test {
    rateInput.siderialRate = SPEED_1_X;
    updateMountSpeed();
    //Test }

    /* Start timer2. This is used to generate a 10ms (100 Hz) clock for
     * the parallel guiding debounce and the goto code.
     *
     * The activities are not timing critical, so a 8-bit timer is used.
     * There is an error of less than 0.5% associated with this.
     */
    cli();
    TCCR2A = 0;// set entire TCCR2A register to 0
    TCCR2B = 0;// same for TCCR2B
    //TCNT2  = 0;//initialize counter value to 0
    OCR2A = (CLK_RATE / 1024 / TMR2_FREQ) - 1;
    // turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set CS21 bit for 8 prescaler
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
    // enable timer compare interrupt
    TIMSK2 |= (1 << OCIE2A);
    /* Enable interrupts */
    sei();
}

/* The following timer handles TIMER 2 interrupts. Several functions use this
 * 10 ms timer for time keeping functions
 */
ISR(TIMER2_COMPA_vect)
{
#ifdef _PGUIDE_H_
    pguideInt();
#endif /*PGUIDE_H_ */
#ifdef _GOTO_H_
    gotoInt();
#endif /* _GOTO_H */

    /* Decrement the step counter in the excitation entry. This is used
     * to time relay state changes
     */
    if (raExcitation.counter > 0)
        --raExcitation.counter;
    if (decExcitation.counter > 0)
        --decExcitation.counter;
    #if 0
    //print log {
    str = getBuff();
    sprintf(str, "Timer2");
    putstr(str);
    // }
    #endif
    /* Update the pulseguide timers (if runnning) */
#define TMR2_MS (1000 / TMR2_FREQ)
#if 0
    if (raPulseTime > 0)
    {
        // Update timer
        if (raPulseTime > TMR2_MS)
            raPulseTime -= TMR2_MS;
        else
        {
            raPulseTime = 0;
            rateInput.serialRaRate = SPEED_0_X;
            updateMountSpeed();
        }
    }
        
    if (decPulseTime > 0)
    {
        // Update timer
        if (decPulseTime > TMR2_MS)
            decPulseTime -= TMR2_MS;
        else
        {
            decPulseTime = 0;
            rateInput.serialDecRate = SPEED_0_X;
            updateMountSpeed();
        }
    }
#endif//#if 0
}

void loop() // run over and over
{
  // Receive from bluetooth and send it to PC
  if (Serial3.available()) {
    uint8_t c = Serial3.read();
    paddleProcess(c);
  }
  if (Serial.available()) {
    uint8_t c = Serial.read();
    sigSerialReceive(c);
  }
  srSaveState();
  rxProcessing();
}
