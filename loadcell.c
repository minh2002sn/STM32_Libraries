#include "loadcell.h"

extern UART_HandleTypeDef huart1;

void LC_Init(LC_HandleTypeDef *p_hlc, GPIO_TypeDef *p_DT_GPIOx, uint16_t p_DT_GPIO_Pin, GPIO_TypeDef *p_CLK_GPIOx, uint16_t p_CLK_GPIO_Pin, float p_a, float p_b, float p_error){
	p_hlc->DT_GPIOx = p_DT_GPIOx;
	p_hlc->DT_GPIO_Pin = p_DT_GPIO_Pin;
	p_hlc->CLK_GPIOx = p_CLK_GPIOx;
	p_hlc->CLK_GPIO_Pin = p_CLK_GPIO_Pin;
	p_hlc->calib_state = NO_CALIBRATE;
	p_hlc->a = p_a;
	p_hlc->b = p_b;
	p_hlc->b += p_error;
}

long LC_Read(LC_HandleTypeDef *p_hlc){
	long t_res = 0;
	uint8_t t_data[3] = {};
	HAL_GPIO_WritePin(p_hlc->CLK_GPIOx, p_hlc->CLK_GPIO_Pin, GPIO_PIN_RESET);
	while(HAL_GPIO_ReadPin(p_hlc->DT_GPIOx, p_hlc->DT_GPIO_Pin) == GPIO_PIN_SET);
	for(int i = 0; i < 3; i++){
	  for(int j = 0; j < 8; j++){
		HAL_GPIO_WritePin(p_hlc->CLK_GPIOx, p_hlc->CLK_GPIO_Pin, GPIO_PIN_SET);
		t_data[2-i] |= HAL_GPIO_ReadPin(p_hlc->DT_GPIOx, p_hlc->DT_GPIO_Pin) << (7-j);
		HAL_GPIO_WritePin(p_hlc->CLK_GPIOx, p_hlc->CLK_GPIO_Pin, GPIO_PIN_RESET);
	  }
	}
	HAL_GPIO_WritePin(p_hlc->CLK_GPIOx, p_hlc->CLK_GPIO_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(p_hlc->CLK_GPIOx, p_hlc->CLK_GPIO_Pin, GPIO_PIN_RESET);

	t_data[2] ^= 0x80; // if (MSB == 1) -> MSB = 0;
	t_res = ((uint32_t) t_data[2] << 16) | ((uint32_t) t_data[1] << 8) | (uint32_t) t_data[0];
	return t_res;
}

void LC_Calibration(LC_HandleTypeDef *p_hlc){
	float t_y1 = 100.0;
	float t_x1 = 0;
	float t_x0 = 0;

	HAL_UART_Transmit(&huart1, "Getting t_x0\n", 13, 100);

//	p_hlc->calib_state = GETTING_X0;
	for(int i = 0; i < NUMBER_OF_SAMPLE; i++){
		t_x0 += LC_Read(p_hlc);
		HAL_Delay(10);
	}
	t_x0 /= (float)NUMBER_OF_SAMPLE;

//	p_hlc->calib_state = WAITING_SAMPLE_MASS;
	HAL_UART_Transmit(&huart1, "Waiting sample mass\n", 20, 100);
	while(1){
		if(LC_Read(p_hlc) > t_x0 + 10000){
			HAL_UART_Transmit(&huart1, "Getting t_x1\n", 13, 100);
//			p_hlc->calib_state = GETTING_X1;
			HAL_Delay(2000);
			for(int i = 0; i < NUMBER_OF_SAMPLE; i++){
				t_x1 += LC_Read(p_hlc);
				HAL_Delay(10);
			}
			t_x1 /= (float)NUMBER_OF_SAMPLE;
			break;
		}
	}
	p_hlc->a = t_y1 / (t_x1 - t_x0);
	p_hlc->b = -t_y1 * t_x0 / (t_x1 - t_x0);

	uint8_t t_Tx_Buff[20] = {};
	sprintf((char*)t_Tx_Buff, "%lf ", p_hlc->a);
	HAL_UART_Transmit(&huart1, t_Tx_Buff, 20, 500);
	sprintf((char*)t_Tx_Buff, "%lf\n", p_hlc->b);
	HAL_UART_Transmit(&huart1, t_Tx_Buff, 20, 500);
	HAL_UART_Transmit(&huart1, "Calibrate done\n", 15, 100);

//	p_hlc->calib_state = CALIBRATED;
}

float LC_Get_Mass(LC_HandleTypeDef *p_hlc){
	return p_hlc->a * LC_Read(p_hlc) + p_hlc->b;
}
