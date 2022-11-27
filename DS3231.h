#ifndef DS3231_H
#define DS3231_H

#include "main.h"
#include "stdint.h"

enum{
	DS3231_FAIL,
	DS3231_OK,
};

typedef struct{
	uint8_t current_hour;
	uint8_t current_minute;
	uint8_t current_second;
	uint8_t current_day;
	uint8_t current_date;
	uint8_t current_month;
	uint8_t currnet_year;
	I2C_HandleTypeDef *hi2c;
} DS3231_HandleTypeDef;

void DS3231_Init(DS3231_HandleTypeDef *p_ds3231, I2C_HandleTypeDef *p_hi2c);
void DS3231_Set_Time(DS3231_HandleTypeDef *p_ds3231, uint8_t p_hour, uint8_t p_minute, uint8_t p_second, uint8_t p_day);
uint8_t DS3231_Get_Time(DS3231_HandleTypeDef *p_ds3231);
void DS3231_Set_Date(DS3231_HandleTypeDef *p_ds3231, uint8_t p_date, uint8_t p_month, uint8_t p_year);
uint8_t DS3231_Get_Date(DS3231_HandleTypeDef *p_ds3231);

#endif
