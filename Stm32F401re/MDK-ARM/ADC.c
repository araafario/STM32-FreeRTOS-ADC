#include "GlobalVariable.h"

void ADC_MultiplexData(){

		ADC_Select_CH0();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
		
		
	  ADC_Select_CH1();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);

		
	  ADC_Select_CH4();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
		//osDelay(5);
}