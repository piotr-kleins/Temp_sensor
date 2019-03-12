#include "stm32f1xx.h"
#include "string.h"
#include <stdio.h>
#include <stdint.h>
#include "lcd_5110.h"
#include "hdc2010.h"


/*
 * Reading data from HDC2010 temperature and humidity sensor
 * and displaying it on LCD 5110. Also data is send to computer via UART.
 *
 * i2c interface is used to communicate with sensor:
 * 		PB6 - SDA
 * 		PB7 - SCL
 * 		VCC - 3.3V
 *
 * SPI interface is used to communicate with LCD display:
 * 		PA5 - SCK  (Clk)
 * 		PA6 - MISO (Unused)
 * 		PA7 - MOSI (Din)
 *
 * 		Additional control lines:
 * 		PC1 - Data/Command (DC)
 * 		PC2 - Chip enable (CE)
 * 		PC3 - RST (Reset)
 *
 * UART interface is used to communicate with computer:
 * 		PA2 - Tx
 * 		PA3 - Rx
 *
 * Diodes to inform about temperature and humidity alarm:
 * 		PC8 - Temperature
 * 		PC9 - Relative humidity
 *
 */

#define TEMP_ALARM_PIN			GPIO_PIN_8
#define HUMID_ALARM_PIN			GPIO_PIN_9

UART_HandleTypeDef uart;

char msg[50];
char temp_displ[10];
char humid_displ[10];

float temp_th = 25.0;				// Temperature threshold
float humid_th = 60.0;				// Relative humidity threshold

void UART_Init();
void I2C_Init();
void SPI_Init();
void GPIO_Alarm_Init();

TIM_HandleTypeDef tim2;

void TIM2_IRQHandler(void)
{
 HAL_TIM_IRQHandler(&tim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	HAL_StatusTypeDef status;
	uint16_t size = 0;
	size = sprintf(msg, "Temp: %0.02f, Wilg: %.02f \r\n", 20.1, 32.31);
	status = HAL_UART_Transmit_IT(&uart, (uint8_t*)msg, size);
	size = size+1;
}


int main(void)
{
	SystemCoreClock = 8000000;
	HAL_Init();
	__HAL_RCC_GPIOA_CLK_ENABLE();			//GPIOA --> UART
	__HAL_RCC_GPIOB_CLK_ENABLE();			//GPIOB --> i2c
	__HAL_RCC_GPIOC_CLK_ENABLE();			//GPIOC --> SPI
	__HAL_RCC_SPI1_CLK_ENABLE();

	UART_Init();
	I2C_Init();
	SPI_Init();
	GPIO_Alarm_Init();


	__HAL_RCC_TIM2_CLK_ENABLE();
	tim2.Instance = TIM2;
	tim2.Init.Period = 1000 - 1;
	tim2.Init.Prescaler = 8000 - 1;
	tim2.Init.ClockDivision = 0;
	tim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	tim2.Init.RepetitionCounter = 0;
	tim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&tim2);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	//HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_TIM_Base_Start_IT(&tim2);



	float temp = 1.1;
	float humid = 2.2;

	lcd_init();
	lcd_clear();
	lcd_text_display(0, 0, "Temperatura: ");
	lcd_text_display(3, 0, "Wilgotnosc: ");

	while(1)
	{
		temp = read_temp();
		humid = read_humid();

		sprintf(msg, "Temp: %0.02f, Wilg: %.02f \r\n", temp, humid);
		sprintf(temp_displ, "%0.02f C", temp);
		sprintf(humid_displ, "%0.02f %%", humid);

		lcd_text_display(1, 40, temp_displ);
		lcd_text_display(4, 40, humid_displ);

		//HAL_UART_Transmit(&uart, (uint8_t*)msg, strlen(msg), 100);
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

	HAL_NVIC_EnableIRQ(USART2_IRQn);
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

void SPI_Init()
{
	GPIO_InitTypeDef gpio;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7; // SCK, MOSI
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_AF_INPUT;
	gpio.Pin = GPIO_PIN_6; // MISO
	gpio.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &gpio);

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = LCD_DC|LCD_CE|LCD_RST;
	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, LCD_CE|LCD_RST, GPIO_PIN_SET);

	spi.Instance = SPI1;
	spi.Init.Mode = SPI_MODE_MASTER;
	spi.Init.NSS = SPI_NSS_SOFT;
	spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2; // 4MHz
	spi.Init.Direction = SPI_DIRECTION_2LINES;
	spi.Init.CLKPhase = SPI_PHASE_1EDGE;
	spi.Init.CLKPolarity = SPI_POLARITY_LOW;
	spi.Init.DataSize = SPI_DATASIZE_8BIT;
	spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
	spi.Init.TIMode = SPI_TIMODE_DISABLE;
	spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	spi.Init.CRCPolynomial = 7;
	HAL_SPI_Init(&spi);

	__HAL_SPI_ENABLE(&spi);
}

void GPIO_Alarm_Init(){

	GPIO_InitTypeDef gpio;

	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pin = TEMP_ALARM_PIN | HUMID_ALARM_PIN;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	HAL_GPIO_Init(GPIOC, &gpio);
	HAL_GPIO_WritePin(GPIOC, TEMP_ALARM_PIN|HUMID_ALARM_PIN, GPIO_PIN_RESET);
}








