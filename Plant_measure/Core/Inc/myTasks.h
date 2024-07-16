/*
 * myTasks.h
 *
 *  Created on: Jun 25, 2024
 *      Author: maciej
 */

#ifndef INC_MYTASKS_H_
#define INC_MYTASKS_H_

#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern volatile uint16_t ADC_VALUE[2];

extern void GROUND_MEASURE_task(void* vParameters);
extern void LCD_PRINT_task(void* vParameters);

#endif /* INC_MYTASKS_H_ */
