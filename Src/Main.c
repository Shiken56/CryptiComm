

#include "stm32f446xx.h"
#include "aes.h"

#include <string.h>
#include <stdint.h>

#define AES_BLOCK_SIZE 16

void I2C1_GPIOInits(GPIO_Handle_t I2C1Pins)
{


	I2C1Pins.pGPIOx = GPIOB;
	I2C1Pins.GPIO_PinConfig.GPIO_PinMode= GPIO_MODE_ALTFN;
	I2C1Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2C1Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2C1Pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2C1Pins.GPIO_PinConfig.GPIO_Pinspeed = GPIO_SPEED_FAST;

	//scl pb6
	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO6;

	GPIO_Init(&I2C1Pins);

	//sda pb9

	I2C1Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO9;

	GPIO_Init(&I2C1Pins);






}

void I2C1_Inits(I2C_Handle_t I2C1Handle)
{


	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);


}

void I2C_SendEncryptedMessage(I2C_Handle_t pI2CHandle, uint8_t slaveAddress, uint8_t * data)
{
	//i2c pins
	I2C1_GPIOInits(pI2CHandle);

	//i2c peripheral
	I2C1_Inits(pI2CHandle);

	//enable i2c peripheral
	I2C_PeripheralControl(pI2CHandle->pI2Cx, ENABLE);

	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);

	I2C_MasterSendData(pI2CHandle, data, 16, slaveAddress);


}







int main(void)
{

		I2C_Handle_t I2C1Pins;


		//these pins for the gpio module

		GPIO_PeriClockControl(GPIOA, ENABLE);
		GPIO_PeriClockControl(GPIOB, ENABLE);
		GPIO_PeriClockControl(GPIOC, ENABLE);

		lcd_init();
		lcd_set_cursor(0,1);

		/*
		 * User application decides state of the GPIO Peripheral
		 */

		// 1. AES Key (128-bit key)
		uint8_t key[16] = {
			0x2b, 0x7e, 0x15, 0x16,
			0x28, 0xae, 0xd2, 0xa6,
			0xab, 0xf7, 0x15, 0x88,
			0x09, 0xcf, 0x4f, 0x3c
		};

		// 2. Input string from user
		char input[] = ("Mark 27");

		// 3. Prepare fixed 16-byte block
		uint8_t block[16];
		memset(block, ' ', 16);  // Fill with spaces
		strncpy((char *)block, input, 16);  // Copy up to 16 chars

		//printing the un encrypted value

		lcd_print_string(&input);

		mdelay(2000);

		// 4. Encrypt block
		struct AES_ctx ctx;

		AES_init_ctx(&ctx, key);
		AES_ECB_encrypt(&ctx, block);

		// 5. Print encrypted bytes

		lcd_print_string(&ctx);

		mdelay(2000);

		I2C_SendEncryptedMessage(I2C1Pins, 0x62, &ctx);






		return 0;
}

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);
}
/*
 * lcd.c
 *
 *  Created on: May 21, 2025
 *      Author: aayus
 */


