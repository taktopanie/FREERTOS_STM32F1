/*
 * HD44780.h
 *
 *  Created on: Oct 25, 2023
 *      Author: maciej
 */

#ifndef MY_DRIVER_INC_HD44780_H_
#define MY_DRIVER_INC_HD44780_H_

#include <stm32f1xx.h>

/*
 *
 * STM32 LCD 2x16 HD44780 driver
 *
 *
 *
 *
 *
 */


#define LCD_DATA_4_PIN				GPIO_PIN_3
#define LCD_DATA_5_PIN				GPIO_PIN_4
#define LCD_DATA_6_PIN				GPIO_PIN_5
#define LCD_DATA_7_PIN				GPIO_PIN_6

#define LCD_RS_PIN					GPIO_PIN_0
#define LCD_RW_PIN					GPIO_PIN_1
#define LCD_E_PIN					GPIO_PIN_2

#define LCD_DATA_4_PORT				GPIOA
#define LCD_DATA_5_PORT				GPIOA
#define LCD_DATA_6_PORT				GPIOA
#define LCD_DATA_7_PORT				GPIOA

#define LCD_RS_PORT					GPIOA
#define LCD_RW_PORT					GPIOA
#define LCD_E_PORT					GPIOA

#define LCD_LINE1 0x00
#define LCD_LINE2 0x40


/*
 * LCD FUNCTIONS
 */

#define SET_DDRAM_ADDR (1<<7)

#define SET_CGRAM_ADDR (1<<6)

#define FUNCTION_SET (1<<5)
	#define DATA_LENGHT_8BIT (1<<4)
	#define DATA_LENGHT_4BIT (0<<4)
	#define NUMBER_OF_LINES_1 (0<<3)
	#define NUMBER_OF_LINES_2 (1<<3)
	#define CHARACTER_FONT_5x10 (1<<2)
	#define CHARACTER_FONT_5x8 (0<<2)

#define LCD_CURSOR_DISPLAY_SHIFT (1<<4)
	#define DISPLAY_SHIFT (1<<3)
	#define CURSOR_SHIFT (0<<3)
	#define SHIFT_RIGHT (1<<2)
	#define SHIFT_LEFT (0<<2)

#define LCD_ONOFF (1<<3)
	#define LCD_DISPLAY_ON (1<<2)
	#define LCD_DISPLAY_OFF (0<<2)
	#define LCD_COURSOR_ON (1<<1)
	#define LCD_COURSOR_OFF (0<<1)
	#define LCD_BLINKINGCOURSOR_ON (1<<0)
	#define LCD_BLINKINGCOURSOR_OFF (0<<0)

#define ENTRY_MODE (1<<2)
	#define INCREMENT (1<<1)
	#define DECREMENT (0<<1)
	#define DISPLAY_SHIFT_ON (1<<0)
	#define DISPLAY_SHIFT_OFF (0<<0)

#define LCD_RETURN_HOME (1<<1)

#define LCD_CLEAR (1<<0)


/*
 * function definitions
 */


void Set_RS (void);
void Set_RW (void);
void Set_E (void);

void Reset_RS (void);
void Reset_RW (void);
void Reset_E (void);


void lcd_init(void);

void lcd_clear(void);

void lcd_sendHalf(uint8_t data);
void lcd_send_byte(uint8_t byte);
void lcd_send_command(unsigned char command);
void lcd_send_data(unsigned char data);

void lcd_send_text(char *text);

uint8_t lcd_readHalf(void);		//not used yet
uint8_t lcd_receive_byte(void);		//not used yet

void lcd_add_specials(void);

void _LCD_PIN_SET_OUTPUT(GPIO_TypeDef * LCD_DATA_PORT, uint16_t LCD_DATA_PIN );
void _LCD_PIN_SET_INPUT(GPIO_TypeDef * LCD_DATA_PORT, uint16_t LCD_DATA_PIN );


#endif /* MY_DRIVER_INC_HD44780_H_ */
