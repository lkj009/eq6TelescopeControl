#ifndef _SR_H_
#define _SR_H_

void srSaveState(void);
uint8_t srLoadState(void);

/* Flag to request parameter save */
extern volatile uint8_t         doSave;

#endif /* _SR_H_ */
