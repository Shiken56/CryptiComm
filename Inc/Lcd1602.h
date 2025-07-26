/*
 * Lcd1602.h
 *
 *  Created on: May 21, 2025
 *      Author: aayus
 */

#ifndef LCD1602_H_
#define LCD1602_H_

#include "stm32f446xx.h"

/*
 * Exposing the apis
 */

void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_char(uint8_t data);
void lcd_display_clear(void);
void lcd_print_string(char * message);
void lcd_display_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t column);
void mdelay(uint32_t cnt);
/*
 * Application configurable items
 */

//Using PA1-PA7 for the LCD configuration

#define LCD_GPIO_RS_PORT 		GPIOB
#define LCD_GPIO_RS			GPIO_PIN_NO5

//#define LCD_GPIO_RW_PORT 		GPIOA
//#define LCD_GPIO_RW			GPIO_PIN_NO5

#define LCD_GPIO_EN_PORT 		GPIOB
#define LCD_GPIO_EN			GPIO_PIN_NO4

#define LCD_GPIO_D4_PORT 		GPIOC
#define LCD_GPIO_D4			GPIO_PIN_NO7

#define LCD_GPIO_D5_PORT 		GPIOB
#define LCD_GPIO_D5			GPIO_PIN_NO6

#define LCD_GPIO_D6_PORT 		GPIOA
#define LCD_GPIO_D6			GPIO_PIN_NO7

#define LCD_GPIO_D7_PORT 		GPIOA
#define LCD_GPIO_D7			GPIO_PIN_NO6

/*
 * LCD commands macros
 */

#define LCD_CMD_4DL_2N_5X8F			0x28
#define LCD_CMD_DON_CURON			0x0E
#define LCD_CMD_INCADD				0x06
#define LCD_CMD_DIS_CLEAR			0x01
#define LCD_CMD_DIS_RETURN_HOME		0x02
#define LCD_CMD_DOFF_COFF			0x08




#endif /* LCD1602_H_ */

