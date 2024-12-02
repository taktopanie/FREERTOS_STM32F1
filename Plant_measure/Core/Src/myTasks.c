/*
 * myTasks.c
 *
 *  Created on: Jun 25, 2024
 *      Author: maciej
 */

#include "myTasks.h"

//////////////////////////////////////
// CHOSE WHICH LCD WILL BE USED \/

//UNCOMMENT BELOW IF ST7735 is used
#define USE_ST7745

//UNCOMMENT BELOW IF LCD_16x2 is used
//#define USE_LCD_16x2

//////////////////////////////////////

#ifdef USE_ST7745
	/////////////// ST7735 ADDED ///////////////
	#include "LCD_MCUDEV.h"
	#include "ST7735.h"
	////////////////////////////////////////////
#endif

//IMPORTED FUNCTIONS
extern void GROUND_MEASURE_INIT_task(void* vParameters);
extern void GROUND_MEASURE_CALCULATE_task(void* vParameters);
extern void LCD_PRINT_task(void* vParameters);
extern void WATERING_task(void* vParameters);

//IMPORTED HANDLERS
extern xTimerHandle PUMP_TIMER[NUMBER_OF_SENSORS];
extern SemaphoreHandle_t PUMP_SEMAPHORE[NUMBER_OF_SENSORS];
extern ADC_HandleTypeDef hadc1;
extern TaskHandle_t WATERING_hndl;
extern TaskHandle_t LCD_PRINT_hndl;

//IMPORTED VARIABLES
extern volatile uint16_t ADC_VALUE[NUMBER_OF_SENSORS];



void GROUND_MEASURE_INIT_task(void* vParameters)
{

	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(5000));
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_VALUE, NUMBER_OF_SENSORS);
	}
}

void GROUND_MEASURE_CALCULATE_task(void* vParameters)
{
	uint32_t measure_addr = 0;

	uint16_t* ADC_measures = malloc(MAX_MEAS_NUMBER * sizeof(uint16_t));

	while(1)
	{
		xTaskNotifyWait(0,0,&measure_addr,portMAX_DELAY);

		uint16_t * wsk = (uint16_t *)measure_addr;

		ADC_measures[0] = *wsk;
		ADC_measures[1] = *(wsk+1);

		xTaskNotify(LCD_PRINT_hndl, (uint32_t)measure_addr, eSetValueWithOverwrite);

		//WATERING IF REQUIRED
		if(ADC_measures[0]>1800)
		{
			xTaskNotify(WATERING_hndl,0x1 ,eSetBits);
		}
		if(ADC_measures[1]>1800)
		{
			xTaskNotify(WATERING_hndl,0x2 ,eSetBits);
		}
	}
}

void LCD_PRINT_task(void* vParameters)
{
	uint32_t measure_addr = 0;

	uint16_t* ADC_actual_measures = malloc(MAX_MEAS_NUMBER * sizeof(uint16_t));
	uint16_t* ADC_saved_measures = malloc(MAX_MEAS_NUMBER * sizeof(uint16_t));

	char meas_str_value [MAX_MEAS_NUMBER];

#ifdef USE_LCD_16x2
	// LCD 16x2 INIT
	lcd_state_reset();
	lcd_init();
	lcd_state_reset();
	lcd_clear();

#endif

#ifdef USE_ST7745
	/////////////// ST7735 ADDED ///////////////
	lcd_st7735_init();

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_DrawString(0, 0, "MEASURE", Font_16x26, ST7735_BLACK, ST7735_YELLOW);

	FontDef _used_font = Font_11x18;
	uint16_t bg = ST7735_CYAN;
	////////////////////////////////////////////
#endif

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
			itoa(ADC_actual_measures[0], meas_str_value, 10);
#ifdef USE_LCD_16x2
			lcd_send_command(LCD_RETURN_HOME);

			lcd_send_text("ADC_1: ");

			lcd_send_text(meas_str_value);

			//CLEAR THE REST OF THE RESULT ON LCD
			uint8_t j = 0;
			while(meas_str_value[j]!= '\0')j++;

			for(int i = j; i < 4; i++)
			{
				lcd_send_data(0x20);
			}
#endif
#ifdef USE_ST7745
			/////////////// ST7735 ADDED ///////////////
			ST7735_DrawString(0, 50, "ADC_1:", _used_font, ST7735_BLACK, bg);
			ST7735_DrawString(0+_used_font.width*7, 50, meas_str_value, _used_font, ST7735_BLACK, bg);

			//CLEAR THE REST OF THE RESULT ON ST7735
			if(ADC_actual_measures[0] < 9)
			{
				  ST7735_FillRectangle(0+(8 * (_used_font.width)), 50,((_used_font.width) * 3),(_used_font.height),bg);
			}else if(ADC_actual_measures[0] < 99)
			{
				  ST7735_FillRectangle(0+(9 * (_used_font.width)), 50,((_used_font.width) * 2),(_used_font.height),bg);
			}else if(ADC_actual_measures[0] < 999)
			{
				  ST7735_FillRectangle(0+(10 * (_used_font.width)), 50,(_used_font.width),(_used_font.height),bg);
			}
			////////////////////////////////////////////
#endif
		}

		//SECOND ADC MEASUREMENT
		if(ADC_saved_measures[1] != ADC_actual_measures[1])
		{
			ADC_saved_measures[1] = ADC_actual_measures[1];

			itoa(ADC_actual_measures[1], meas_str_value, 10);
#ifdef USE_LCD_16x2
			lcd_send_command(SET_DDRAM_ADDR|LCD_LINE2);

			lcd_send_text("ADC_2: ");

			lcd_send_text(meas_str_value);

			//CLEAR THE REST OF THE RESULT ON LCD
			uint8_t j = 0;
			while(meas_str_value[j]!= '\0')j++;

			for(int i = j; i < 4; i++)
			{
				lcd_send_data(0x20);
			}
#endif
#ifdef USE_ST7745
			/////////////// ST7735 ADDED ///////////////
			ST7735_DrawString(0, 50+_used_font.height, "ADC_2:", _used_font, ST7735_BLACK, bg);
			ST7735_DrawString(0+_used_font.width*7, 50+_used_font.height, meas_str_value, _used_font, ST7735_BLACK, bg);

			//CLEAR THE REST OF THE RESULT ON ST7735
			if(ADC_actual_measures[1] < 9)
			{
				  ST7735_FillRectangle(0+(8 * (_used_font.width)), 50+_used_font.height,((_used_font.width) * 3),(_used_font.height),bg);
			}else if(ADC_actual_measures[1] < 99)
			{
				  ST7735_FillRectangle(0+(9 * (_used_font.width)), 50+_used_font.height,((_used_font.width) * 2),(_used_font.height),bg);
			}else if(ADC_actual_measures[1] < 999)
			{
				  ST7735_FillRectangle(0+(10 * (_used_font.width)), 50+_used_font.height,(_used_font.width),(_used_font.height),bg);
			}
			////////////////////////////////////////////
#endif
		}
	}

}

void WATERING_task(void* vParameters)
{
	uint16_t pin_number = 0;

	GPIO_TypeDef* PUMP_GPIO_PORT_TABLE[MAX_MEAS_NUMBER];
	uint16_t PUMP_GPIO_PIN_TABLE[MAX_MEAS_NUMBER];

	for(uint16_t i = 0 ; i < NUMBER_OF_SENSORS; i++)
	{
		if(i == 0)
		{
			PUMP_GPIO_PORT_TABLE[i] = PUMP_0_GPIO_Port;
			PUMP_GPIO_PIN_TABLE[i] = PUMP_0_Pin;
		}else if(i == 1)
		{
			PUMP_GPIO_PORT_TABLE[i] = PUMP_1_GPIO_Port;
			PUMP_GPIO_PIN_TABLE[i] = PUMP_1_Pin;
		}else if(i == 2)
		{
			PUMP_GPIO_PORT_TABLE[i] = PUMP_2_GPIO_Port;
			PUMP_GPIO_PIN_TABLE[i] = PUMP_2_Pin;
		}else if(i == 1)
		{
			//REST TO DO IN THE FUTURE
		}
	}

	while(1)
	{
		if(xTaskNotifyWait(0,0xFFFF,(uint32_t *)&pin_number,pdMS_TO_TICKS(5000)) == pdPASS)
		{

			for(uint16_t i = 0 ; i < NUMBER_OF_SENSORS; i++)
			{
				if(pin_number & (1 << i))
				{
					if(xSemaphoreTake(PUMP_SEMAPHORE[i], 0) == pdTRUE)
					{

						//TURN ON THE PUMP
						HAL_GPIO_WritePin(PUMP_GPIO_PORT_TABLE[i], PUMP_GPIO_PIN_TABLE[i], DISABLE);
						//WATERING TIME
						vTaskDelay(pdMS_TO_TICKS(5000));
						//TURN OFF THE PUMP
						HAL_GPIO_WritePin(PUMP_GPIO_PORT_TABLE[i], PUMP_GPIO_PIN_TABLE[i], ENABLE);

						//TIMER STOPS NEXT WATERING FOR 3 HRs
						xTimerStart(PUMP_TIMER[i], 0);
					}
				}
			}

		}
//		else{
//			//TURN OFF ALL PUMPS after xTaskNotifyWait gets no IRQ ==> pdMS_TO_TICKS(5000)
//			for(uint16_t i = 0 ; i < NUMBER_OF_SENSORS; i++)
//			{
//				HAL_GPIO_WritePin(PUMP_GPIO_PORT_TABLE[i], PUMP_GPIO_PIN_TABLE[i], DISABLE);
//			}
//
//		}

	}
}

void TIMER_PUMP_EXPIRED(TimerHandle_t xTimer)
{
	for(uint16_t i = 0 ; i < NUMBER_OF_SENSORS; i++)
	{
		if(xTimer == PUMP_TIMER[i])
		{
			//free the semaphore
			xSemaphoreGive(PUMP_SEMAPHORE[i]);

		}
	}
}
