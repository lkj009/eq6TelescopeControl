#ifndef _SERIAL_H_
#define _SERIAL_H_
/* Define the constants for the guiding input
 */
#define GUIDE_BAUD      9600

void serialInit(void);
uint8_t readSerial(void);
int serialAvailable(void);
void putch(char c);
void putstr(char *pStr);
void sigSerialReceive(char c);
char serialTx(const char *pStr);
void rxProcessing(void);
char* getBuff(void);
void print16(uint16_t num);
void print32(uint32_t num);

/* Flag to request parameter save */
extern char* str;
/* Global timer for pulseguide */
extern volatile uint16_t raPulseTime;
extern volatile uint16_t decPulseTime;
#endif /* _SERIAL_H_ */
