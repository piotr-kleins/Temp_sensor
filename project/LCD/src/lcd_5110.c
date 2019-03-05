#include "lcd_5110.h"
#include "font.h"

#define LCD_BUFFER_SIZE  (48 * 84 / 8)
static uint8_t lcd_buffer[LCD_BUFFER_SIZE];

SPI_HandleTypeDef spi;


void lcd_reset()
{
 HAL_GPIO_WritePin(GPIOC, LCD_RST, GPIO_PIN_RESET);
 HAL_GPIO_WritePin(GPIOC, LCD_RST, GPIO_PIN_SET);
}

void lcd_cmd(uint8_t cmd)
{
 HAL_GPIO_WritePin(GPIOC, LCD_CE|LCD_DC, GPIO_PIN_RESET);
 HAL_SPI_Transmit(&spi, &cmd, 1, HAL_MAX_DELAY);
 HAL_GPIO_WritePin(GPIOC, LCD_CE|LCD_DC, GPIO_PIN_SET);
}

void lcd_init()
{
	lcd_reset();
	lcd_cmd(0x21);
	lcd_cmd(0x14);
	lcd_cmd(0x80 | 0x3A);
	lcd_cmd(0x20);
	lcd_cmd(0x0c);
}

void lcd_text_display(uint8_t y_pos, uint8_t x_pos, char* text){

	uint8_t* buff_ptr = &lcd_buffer[y_pos * 84 + x_pos];

	int i =0;
	while(*text != '\0'){
		for(i=0; i<5; i++){
			*buff_ptr = font_ASCII[*text - 32][i];
			buff_ptr++;
		}
		text++;
	}

	lcd_send_buffer();
}

void lcd_clear(void)
{
	memset(lcd_buffer, 0, LCD_BUFFER_SIZE);
	lcd_send_buffer();
}

void lcd_send_buffer(){
	 HAL_GPIO_WritePin(GPIOC, LCD_DC, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(GPIOC, LCD_CE, GPIO_PIN_RESET);
	 HAL_SPI_Transmit(&spi, lcd_buffer, LCD_BUFFER_SIZE, HAL_MAX_DELAY);
	 HAL_GPIO_WritePin(GPIOC, LCD_CE, GPIO_PIN_SET);
}
