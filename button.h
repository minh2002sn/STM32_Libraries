#ifndef BUTTON_H
#define BUTTON_H

#include "main.h"

typedef struct{
	uint8_t BTN_Current;
	uint8_t BTN_Last;
	uint8_t BTN_Filter;
	uint8_t is_debouncing;
	uint8_t is_press_long;
	uint32_t time_debounce;
	uint32_t time_start_press;
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
} BUTTON_HandleTypedef;

void BUTTON_Handle(BUTTON_HandleTypedef *ButtonX);
void BUTTON_Init(BUTTON_HandleTypedef *ButtonX, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif
