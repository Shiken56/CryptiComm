/*
 * 005_i2c_tx_testing.c
 *
 *  Created on: Jun 24, 2025
 *      Author: aayus
 */

#include "stm32f446xx.h"
#include <string.h>
#include "stm32f446xx_i2c_driver.h"
/*
 * PB6- I2C1 SCL
 * PB9- I2C1 SDA
 */
#define SLAVE_ADDR   0x68

I2C_Handle_t I2C1Handle;

uint8_t some_data[]= "we are testing i2c master";


void I2C1_GPIOInits(void)
{

	GPIO_Handle_t I2C1Pins;

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

void I2C1_Inits(void)
{

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);


}

int main(void)
{
	//i2c pins
	I2C1_GPIOInits();

	//i2c peripheral
	I2C1_Inits();

	//enable i2c peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, ENABLE);

	//send the data
	I2C_MasterSendData(&I2C1Handle, some_data, strlen((char*)some_data), SLAVE_ADDR);


		while(1);

}






