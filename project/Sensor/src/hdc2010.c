#include "hdc2010.h"
#include "math.h"

I2C_HandleTypeDef i2c;

float read_temp(void)
{
	int16_t temp_data = 0;
	float temp_final = 0.0;
	uint8_t init_trans = 0x01;
	uint8_t data_read[2] = {0};
	//HAL_StatusTypeDef status;

	// Enable reading from sensor
	HAL_I2C_Mem_Write(&i2c, SENS_ADDR, 0x0F, 1, &init_trans, 1, HAL_MAX_DELAY);

	// Read 2 registers containing temperature data
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x00, 1, data_read, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x01, 1, data_read + 1, 1, HAL_MAX_DELAY);

	temp_data = ((int16_t)data_read[0]) + ((int16_t)data_read[1] << 8);
	temp_final = (float)temp_data;
	temp_final = (temp_final/pow(2,16)) * 165 - 40;

	return temp_final;
}

float read_humid(void)
{
	int16_t humid_data = 0;
	float humid_final = 0.0;
	uint8_t init_trans = 0x01;
	uint8_t data_read[2] = {0};

	// Enable reading from sensor
	HAL_I2C_Mem_Write(&i2c, SENS_ADDR, 0x0F, 1, &init_trans, 1, HAL_MAX_DELAY);

	// Read 2 registers containing humidity data
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x00, 1, data_read, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&i2c, SENS_ADDR, 0x01, 1, data_read + 1, 1, HAL_MAX_DELAY);

	humid_data = ((int16_t)data_read[0]) + ((int16_t)data_read[1] << 8);
	humid_final = (float)humid_data;
	humid_final = (humid_final/pow(2,16)) * 100;

	return humid_final;
}
