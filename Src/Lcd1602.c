/*
 * Lcd1602.c
 *
 *  Created on: May 21, 2025
 *      Author: aayus
 */

#include "stm32f446xx.h"
#include "Lcd1602.h"

static void write_4_bits(uint8_t value);
static void lcd_enable();

static void udelay(uint32_t cnt);

void lcd_send_command(uint8_t cmd)
{
	/*RS=0 for LCD command*/
	GPIO_WriteToOutputPin(LCD_GPIO_RS_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

//	/*RW=0 for write */
//	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	//first nibble upper



	write_4_bits((cmd >>4));
	//second nibble lower
	write_4_bits(cmd & 0x0F);

}

void lcd_send_char(uint8_t data)
{
	/*RS=1 for LCD user data*/
	GPIO_WriteToOutputPin(LCD_GPIO_RS_PORT, LCD_GPIO_RS, GPIO_PIN_SET);

//	/*RW=0 for write*/
//	GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_bits(data>>4); /*Higher nibble*/
	write_4_bits(data & 0x0F); /*Lower nibble*/

}


void lcd_print_string(char * message)
{
	do
	{
		lcd_send_char((uint8_t)*message++);

	}
	while(*message |= '\0');
}

void lcd_init(void)
{

	//1. Configure the gpio pins which are used for lcd connections

	GPIO_Handle_t lcd_signalA;
	GPIO_Handle_t lcd_signalB;
	GPIO_Handle_t lcd_signalC;

	//Port A PA6,PA7

	lcd_signalA.pGPIOx = GPIOA;
	lcd_signalA.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	lcd_signalA.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signalA.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
 	lcd_signalA.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalA);

	lcd_signalA.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Init(&lcd_signalA);

	//Port B, PB4,PB5,PB6
	lcd_signalB.pGPIOx = GPIOB;
	lcd_signalB.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signalB.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	lcd_signalB.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signalB.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signalB.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalB);

	lcd_signalB.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	GPIO_Init(&lcd_signalB);

	lcd_signalB.pGPIOx->MODER &=~(1U<<9);
	lcd_signalB.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Init(&lcd_signalB);



	//Port C is PC7
	lcd_signalC.pGPIOx = GPIOC;
	lcd_signalC.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signalC.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	lcd_signalC.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signalC.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signalC.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;
	GPIO_Init(&lcd_signalC);



	GPIO_WriteToOutputPin(LCD_GPIO_RS_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);


	GPIO_WriteToOutputPin(LCD_GPIO_EN_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_D4_PORT, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_D5_PORT, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_D6_PORT, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_D7_PORT, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. Do the LCD initialization

	mdelay(40);


	/*RS = 0 , For LCD command */


//	/* RnW = 0, Writing to LCD */


	lcd_send_command(0x33);
	lcd_send_command(0x32);

	mdelay(5);
	//function set command
	lcd_send_command(LCD_CMD_4DL_2N_5X8F);

	mdelay(5);


	lcd_send_command(LCD_CMD_DIS_CLEAR);

	mdelay(5);


	//disply ON and cursor on

	lcd_send_command(LCD_CMD_DON_CURON);

	mdelay(5);


	//entry mode set
	lcd_send_command(LCD_CMD_INCADD);

	mdelay(5);

}


void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	//wait 2ms acc to datasheet
	mdelay(2);

}

/*Cursor returns to home position*/
void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	mdelay(2);

}


/*
 * LCD cursor setting to specified location
 */

void lcd_set_cursor(uint8_t row, uint8_t column)
{
	column--;
	switch(row)
	{
	case 1:
		/*Set cursor to first row + offset index*/
		lcd_send_command((column |=0x80));
		break;
	case 2:
		/*Set cursor to 2nd row address and add index*/
		lcd_send_command((column |=0xC0));
		break;
	default:
		break;

	}
}

/*Writes 4 bits (nibble) of data/cmd to the D4,D5,D6,D7 lines */
static void write_4_bits(uint8_t value)
{
	//LDB goes to db4, msb goes to db7, then write to the output

	GPIO_WriteToOutputPin(LCD_GPIO_D4_PORT, LCD_GPIO_D4, ((value>>0)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_D5_PORT, LCD_GPIO_D5, ((value>>1)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_D6_PORT, LCD_GPIO_D6, ((value>>2)&0x01));
	GPIO_WriteToOutputPin(LCD_GPIO_D7_PORT, LCD_GPIO_D7, ((value>>3)&0x01));



	GPIO_WriteToOutputPin(LCD_GPIO_EN_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	mdelay(1);
	GPIO_WriteToOutputPin(LCD_GPIO_EN_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);

}




static void lcd_enable()
{
	GPIO_WriteToOutputPin(LCD_GPIO_EN_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
	mdelay(1);
	GPIO_WriteToOutputPin(LCD_GPIO_EN_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
	//Any value >37uS
}

void mdelay(uint32_t cnt)
{

	//approx every exacution of the code line is a
	for(uint32_t i=0; i< (cnt*1000); i++);

}

static void udelay(uint32_t cnt)
{

	//approx every exacution of the code line is a microsec
	for(uint32_t i=0; i< (cnt*1); i++);

}






