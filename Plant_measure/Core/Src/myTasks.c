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


	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(500));
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VALUE, 2);
	}
}

void LCD_PRINT_task(void* vParameters)
{
	uint32_t measure_addr = 0;
	uint16_t measure[2];
	char meas_str_value [16];

	while(1)
	{
		xTaskNotifyWait(0,0,&measure_addr,portMAX_DELAY);

		uint16_t * wsk = (uint16_t *)measure_addr;

		measure[0] = *wsk;
		measure[1] = *(wsk+1);


		lcd_clear(); // <<<<<< LCD CLEAR MAKES DISPLAY BLINKS
		//TODO: CLEAR LEFT NUMBERS IF VALUE IS LOWER ==>> lcd_send_command(LCD_RETURN_HOME);

		itoa(measure[0], meas_str_value, 10);

		lcd_send_text("ADC_1: ");
		lcd_send_text(meas_str_value);


		itoa(measure[1], meas_str_value, 10);

		lcd_send_command(SET_DDRAM_ADDR|LCD_LINE2);
		lcd_send_text("ADC_2: ");
		lcd_send_text(meas_str_value);
	}

}
