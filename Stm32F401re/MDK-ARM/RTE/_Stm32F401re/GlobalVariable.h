#ifndef _GLOBALVARIABLE_
#define _GLOBALVARIABLE_

#include "TJ_MPU6050.h"
#include "i2c-lcd.h"
#include <stdio.h>
#include "cmsis_os.h"
#include "math.h"

extern void ADC_Select_CH0 (void);
extern void ADC_Select_CH1 (void);
extern void ADC_Select_CH4 (void);
extern void ADC_MultiplexData(void);


extern ADC_HandleTypeDef hadc1;

extern uint16_t raw;
extern float voltage,temp;
extern int ADC_VAL[3];

#endif