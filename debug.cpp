#include "Arduino.h"
#include "debug.h"


#if 0 //Sample
    //print log {
    str = getBuff();
    sprintf(str, "raDir:%d,raSpeed:%d,decDir:%d,DecSpeed:%d",raDir,raSpeed,decDir,DecSpeed);
    putstr(str);
    // }

    static unsigned int stepCtr = 0;
    raExcitation.c1Ex = _GET_TABLE(0)[stepCtr];
    raExcitation.c2Ex = _GET_TABLE(0)[(stepCtr + STEPS_PER_CYCLE / 4) & (STEPS_PER_CYCLE - 1)];
    stepCtr = (stepCtr + 1) & (STEPS_PER_CYCLE - 1);
    return;
#endif //#if 0
