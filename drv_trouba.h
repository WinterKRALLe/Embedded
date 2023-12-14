#include "MKL25Z4.h"
#include <stdbool.h>

#define SWITCH_PRESSED  	(1)
#define SWITCH_NOT_PRESSED  (0)

bool SW1_pressed;
bool SW2_pressed;
bool SW3_pressed;
bool SW4_pressed;

bool sw1_last_state;
bool sw2_last_state;
bool sw3_last_state;
bool sw4_last_state;

void ADCInit(void);
uint32_t ADCCalibrate(void);
void delay(void);
void TaskSwitches(void);
