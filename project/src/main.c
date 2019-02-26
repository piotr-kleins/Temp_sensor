#include "stm32f1xx.h"
#include "string.h"
#include "math.h"

#define SENS_ADDR		0x82

UART_HandleTypeDef uart;
I2C_HandleTypeDef i2c;
char msg[50];


void UART_Init();
void I2C_Init();
float read_temp();

int main(void)
{
	SystemCoreClock = 8000000;
	HAL_Init();
	__HAL_RCC_GPIOA_CLK_ENABLE();			//GPIOA --> UART
	__HAL_RCC_GPIOB_CLK_ENABLE();			//GPIOB --> i2c
	__HAL_RCC_GPIOC_CLK_ENABLE();

	UART_Init();
	I2C_Init();

	float temp = 10;
	float humid = 23.13;

	while(1)
	{
		temp = read_temp();

		sprintf(msg, "Temp: %0.02f, Wilg: %.02f \r\n", temp, humid);

		HAL_UART_Transmit(&uart, (uint8_t*)msg, strlen(msg), 100);
		HAL_Delay(1000);
	}
}

void UART_Init()
{
	__HAL_RCC_USART2_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_2;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOA, &gpio);

	uart.Instance = USART2;
	uart.Init.BaudRate = 115200;
	uart.Init.WordLength = UART_WORDLENGTH_8B;
	uart.Init.Parity = UART_PARITY_NONE;
	uart.Init.StopBits = UART_STOPBITS_1;
	uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart.Init.OverSampling = UART_OVERSAMPLING_16;
	uart.Init.Mode = UART_MODE_TX_RX;
	HAL_UART_Init(&uart);

}

void I2C_Init()
{
	__HAL_RCC_I2C1_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Pull = GPIO_PULLUP;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio);

	i2c.Instance = I2C1;
	i2c.Init.ClockSpeed = 100000;
	i2c.Init.DutyCycle = I2C_DUTYCYCLE_2;
	i2c.Init.OwnAddress1 = 0xff;
	i2c.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	i2c.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	i2c.Init.OwnAddress2 = 0xff;
	i2c.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	i2c.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	HAL_I2C_Init(&i2c);

}


float read_temp()
{
	int16_t temp_data = 0;
	float temp_final = 0.0;
	uint8_t init_trans = 0x01;
	uint8_t data_read[2] = {0};
	HAL_StatusTypeDef status;

	HAL_Delay(100);
	status = HAL_I2C_Mem_Write(&i2c, SENS_ADDR, 0x0F, 1, &init_trans, 1, HAL_MAX_DELAY);
	HAL_Delay(100);

//	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0xFC, 1, data_read, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0xFD, 1, data_read+1, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0xFE, 1, data_read+2, 1, HAL_MAX_DELAY);
//	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0xFF, 1, data_read+3, 1, HAL_MAX_DELAY);
//	uint8_t i = 0;
//	for(i=0; i<2; i++){
//		HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x00 + i, 1, data_read + i, 1, HAL_MAX_DELAY);
//	}
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x00, 1, data_read, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x01, 1, data_read + 1, 1, HAL_MAX_DELAY);

	temp_data = ((int16_t)data_read[0]) + ((int16_t)data_read[1] << 8);
	temp_final = (float)temp_data;
	temp_final = (temp_final/pow(2,16)) * 165 - 40;

	return temp_final;
}

