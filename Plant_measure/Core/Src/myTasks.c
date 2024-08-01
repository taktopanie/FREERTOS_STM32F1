/*
 * myTasks.c
 *
 *  Created on: Jun 25, 2024
 *      Author: maciej
 */

#include "myTasks.h"


//IMPORTED FUNCTIONS
extern void GROUND_MEASURE_task(void* vParameters);
extern void LCD_PRINT_task(void* vParameters);
extern void WATERING_task(void* vParameters);

//IMPORTED HANDLERS
extern xTimerHandle PUMP_TIMER[NUMBER_OF_SENSORS];
extern SemaphoreHandle_t PUMP_SEMAPHORE[NUMBER_OF_SENSORS];
extern ADC_HandleTypeDef hadc1;
extern TaskHandle_t WATERING_hndl;


//IMPORTED VARIABLES
extern volatile uint16_t ADC_VALUE[NUMBER_OF_SENSORS];



void GROUND_MEASURE_task(void* vParameters)
{

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(5000));
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VALUE, 2);
	}
}

void LCD_PRINT_task(void* vParameters)
{
	uint32_t measure_addr = 0;

	uint16_t* ADC_actual_measures = malloc(MAX_MEAS_NUMBER * sizeof(uint16_t));
	uint16_t* ADC_saved_measures = malloc(MAX_MEAS_NUMBER * sizeof(uint16_t));

	char meas_str_value [16];

	lcd_state_reset();

	lcd_clear();


	while(1)
	{
		xTaskNotifyWait(0,0,&measure_addr,portMAX_DELAY);

		uint16_t * wsk = (uint16_t *)measure_addr;

		ADC_actual_measures[0] = *wsk;
		ADC_actual_measures[1] = *(wsk+1);

		//FRIST ADC MEASUREMENT
		if(ADC_saved_measures[0] != ADC_actual_measures[0])
		{
			ADC_saved_measures[0] = ADC_actual_measures[0];
			lcd_send_command(LCD_RETURN_HOME);

			itoa(ADC_actual_measures[0], meas_str_value, 10);

			lcd_send_text("ADC_1: ");

			lcd_send_text(meas_str_value);

			//CLEAR THE REST OF THE RESULT ON LCD
			uint8_t j = 0;
			while(meas_str_value[j]!= '\0')j++;

			for(int i = j; i < 4; i++)
			{
				lcd_send_data(0x20);
			}
		}

		//SECOND ADC MEASUREMENT
		if(ADC_saved_measures[1] != ADC_actual_measures[1])
		{
			ADC_saved_measures[1] = ADC_actual_measures[1];

			itoa(ADC_actual_measures[1], meas_str_value, 10);

			lcd_send_command(SET_DDRAM_ADDR|LCD_LINE2);

			lcd_send_text("WATERING: ");
			if(ADC_actual_measures[1]<1000)
			{
				lcd_send_text("NO ");
			}else{
				xTaskNotify(WATERING_hndl,0x1 ,eSetBits);
				lcd_send_text("YES");

			}

///////////////////////THE VALUE OF HIGH/LOW PIN ///
//			lcd_send_text(meas_str_value);
//
//			//CLEAR THE REST OF THE RESULT ON LCD
//			j = 0;
//			while(meas_str_value[j]!= '\0')j++;
//
//			for(int i = j; i < 4; i++)
//			{
//				lcd_send_data(0x20);
//			}
/////////////////////////////////////////////////////

		}
	}

}

void WATERING_task(void* vParameters)
{
	uint16_t pin_number = 0;

	while(1)
	{
		if(xTaskNotifyWait(0,0xFFFF,(uint32_t *)&pin_number,pdMS_TO_TICKS(5000)) == pdPASS)
		{
			//TODO: IN FOR LOOP
			if(pin_number & 0x1)
			{
				//first pin
				if(xSemaphoreTake(PUMP_SEMAPHORE[0], 0) == pdTRUE)
				{
					HAL_GPIO_WritePin(PUMP_0_GPIO_Port, PUMP_0_Pin, ENABLE);
					//TIMER STOPS NEXT WATERING FOR 3 HRs
					xTimerStart(PUMP_TIMER[0], 0);
				}
			}
			if(pin_number & 0x2)
			{
				//second pin
				if(xSemaphoreTake(PUMP_SEMAPHORE[1], 0) == pdTRUE)
				{
					HAL_GPIO_WritePin(PUMP_1_GPIO_Port, PUMP_1_Pin, ENABLE);
					//TIMER STOPS NEXT WATERING FOR 3 HRs
					xTimerStart(PUMP_TIMER[1], 0);
				}

			}

		}
		else{
			//TURN OFF ALL PUMPS
			HAL_GPIO_WritePin(PUMP_0_GPIO_Port, PUMP_0_Pin, DISABLE);
			HAL_GPIO_WritePin(PUMP_1_GPIO_Port, PUMP_1_Pin, DISABLE);
		}

	}
}

void TIMER_PUMP_EXPIRED(TimerHandle_t xTimer)
{
	if(xTimer == PUMP_TIMER[0])
	{
		//do sth
		xSemaphoreGive(PUMP_SEMAPHORE[0]);

	}else if(xTimer == PUMP_TIMER[1])
	{
		//do sth
		xSemaphoreGive(PUMP_SEMAPHORE[1]);
	}
}
