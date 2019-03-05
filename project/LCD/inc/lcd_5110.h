#ifndef INC_LCD_5110_H_
#define INC_LCD_5110_H_

#include "stm32f1xx.h"

#define LCD_WIDTH		84
#define LCD_HEIGHT		48

#define LCD_DC			GPIO_PIN_1
#define LCD_CE			GPIO_PIN_2
#define LCD_RST			GPIO_PIN_3

extern const unsigned char logo[504];
extern SPI_HandleTypeDef spi;

void lcd_reset(); 					// Reset LCD
void lcd_cmd(uint8_t cmd);			// Send command to lcd
void lcd_data(const uint8_t* data, int size);	// Send data to LCD buffer
void lcd_init();					// Initialize LCD
void lcd_clear(void);
void lcd_send_buffer(void);
void lcd_text_display(uint8_t x_pos, uint8_t y_pos, char* text);


#endif /* INC_LCD_5110_H_ */
