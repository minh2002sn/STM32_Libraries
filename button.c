#include "button.h"

//========== btn func ==========//
__weak void BTN_Pressing_Callback(BUTTON_HandleTypedef *ButtonX){
	;
}

__weak void BTN_Releasing_Callback(BUTTON_HandleTypedef *ButtonX){
	;
}

__weak void BTN_Short_Pressing_Callback(BUTTON_HandleTypedef *ButtonX){
	;
}

__weak void BTN_Long_Pressing_Callback(BUTTON_HandleTypedef *ButtonX){
	;
}

void BUTTON_Handle(BUTTON_HandleTypedef *ButtonX){
	uint8_t state = HAL_GPIO_ReadPin(ButtonX->GPIOx, ButtonX->GPIO_Pin);
	if(state != ButtonX->BTN_Filter){
		ButtonX->BTN_Filter = state;
		ButtonX->is_debouncing = 1;
		ButtonX->time_debounce = HAL_GetTick();
	}

	if(ButtonX->is_debouncing && (HAL_GetTick() - ButtonX->time_debounce >= 15)){
		ButtonX->BTN_Current = ButtonX->BTN_Filter;
		ButtonX->is_debouncing = 0;
	}

	if(ButtonX->BTN_Current != ButtonX->BTN_Last){
		if(ButtonX->BTN_Current == 0){
			BTN_Pressing_Callback(ButtonX);
			ButtonX->is_press_long = 1;
			ButtonX->time_start_press = HAL_GetTick();
		}
		else{
			if(HAL_GetTick() - ButtonX->time_start_press <= 1000){
				BTN_Short_Pressing_Callback(ButtonX);
			}
			BTN_Releasing_Callback(ButtonX);
		}
		ButtonX->BTN_Last = ButtonX->BTN_Current;
	}

	if(ButtonX->is_press_long && (HAL_GetTick() - ButtonX->time_start_press >= 3000)){
		BTN_Long_Pressing_Callback(ButtonX);
		ButtonX->is_press_long = 0;
	}
}

void BUTTON_Init(BUTTON_HandleTypedef *ButtonX, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){
	ButtonX->GPIOx = GPIOx;
	ButtonX->GPIO_Pin = GPIO_Pin;
	ButtonX->BTN_Current = 1;
	ButtonX->BTN_Last = 1;
	ButtonX->BTN_Filter = 1;
	ButtonX->is_debouncing = 0;
	ButtonX->is_press_long = 0;

}
