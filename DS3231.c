#include "DS3231.h"

#define USING_12_HOURS_TIME		(0x40)

#define DS3231_ADDRESS			(0x68 << 1)
#define SECOND_VALUE_ADDRESS	(0x00)
#define DATE_VALUE_ADDRESS		(0x03)

static uint8_t BCD_To_DEC(uint8_t p_BCD_value){
	return ((p_BCD_value >> 4) * 10) + (p_BCD_value & 0x0F);
}

static uint8_t DEC_To_BCD(uint8_t p_DEC_value){
	return ((p_DEC_value / 10) << 4) | (p_DEC_value % 10);
}

void DS3231_Init(DS3231_HandleTypeDef *p_ds3231, I2C_HandleTypeDef *p_hi2c){
	p_ds3231->hi2c = p_hi2c;
}

void DS3231_Set_Time(DS3231_HandleTypeDef *p_ds3231, uint8_t p_hour, uint8_t p_minute, uint8_t p_second, uint8_t p_day){
	uint8_t t_i2c_buffer[4];
	t_i2c_buffer[0] = DEC_To_BCD(p_second);
	t_i2c_buffer[1] = DEC_To_BCD(p_minute);
	t_i2c_buffer[2] = DEC_To_BCD(p_hour) & (~USING_12_HOURS_TIME);
	t_i2c_buffer[3] = DEC_To_BCD(p_day);
	HAL_I2C_Mem_Write(p_ds3231->hi2c, DS3231_ADDRESS, SECOND_VALUE_ADDRESS, I2C_MEMADD_SIZE_8BIT, t_i2c_buffer, 4, 1000);
}

uint8_t DS3231_Get_Time(DS3231_HandleTypeDef *p_ds3231){
	uint8_t t_i2c_buffer[4];
	if(HAL_I2C_Mem_Read(p_ds3231->hi2c, DS3231_ADDRESS, SECOND_VALUE_ADDRESS, I2C_MEMADD_SIZE_8BIT, t_i2c_buffer, 4, 1000) != HAL_OK){
		return DS3231_FAIL;
	}
	p_ds3231->current_second = BCD_To_DEC(t_i2c_buffer[0]);
	p_ds3231->current_minute = BCD_To_DEC(t_i2c_buffer[1]);
	p_ds3231->current_hour = BCD_To_DEC(t_i2c_buffer[2]);
	p_ds3231->current_day = BCD_To_DEC(t_i2c_buffer[3]);
	return DS3231_OK;
}

void DS3231_Set_Date(DS3231_HandleTypeDef *p_ds3231, uint8_t p_date, uint8_t p_month, uint8_t p_year){
	uint8_t t_i2c_buffer[3];
	t_i2c_buffer[0] = DEC_To_BCD(p_date);
	t_i2c_buffer[1] = DEC_To_BCD(p_month);
	t_i2c_buffer[2] = DEC_To_BCD(p_year);
	HAL_I2C_Mem_Write(p_ds3231->hi2c, DS3231_ADDRESS, DATE_VALUE_ADDRESS, I2C_MEMADD_SIZE_8BIT, t_i2c_buffer, 3, 1000);
}

uint8_t DS3231_Get_Date(DS3231_HandleTypeDef *p_ds3231){
	uint8_t t_i2c_buffer[3];
	if(HAL_I2C_Mem_Read(p_ds3231->hi2c, DS3231_ADDRESS, DATE_VALUE_ADDRESS, I2C_MEMADD_SIZE_8BIT, t_i2c_buffer, 3, 1000) != HAL_OK){
		return DS3231_FAIL;
	}
	p_ds3231->current_date = BCD_To_DEC(t_i2c_buffer[0]);
	p_ds3231->current_month = BCD_To_DEC(t_i2c_buffer[1]);
	p_ds3231->currnet_year = BCD_To_DEC(t_i2c_buffer[2]);
	return DS3231_OK;
}

