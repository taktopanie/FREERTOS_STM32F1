/*
 * myTasks.c
 *
 *  Created on: Jun 25, 2024
 *      Author: maciej
 */

#include "myTasks.h"
#include "HD44780.h"
#include "stdlib.h"

void GROUND_MEASURE_task(void* vParameters)
{

	uint32_t value = 0;
	char meas_str_value [16];
	while(1)
	{

		lcd_clear();

		itoa(value++, meas_str_value, 10);

		lcd_send_text(meas_str_value);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		vTaskDelay(pdMS_TO_TICKS(500));
	}
}
