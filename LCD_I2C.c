#include "LCD_I2C.h"

void LCD_Init(LCD_I2C_HandleTypeDef *p_lcd, I2C_HandleTypeDef *p_hi2c, uint8_t p_cols, uint8_t p_rows, uint8_t p_SLAVE_ADDRESS){
	p_lcd->SLAVE_ADDRESS = p_SLAVE_ADDRESS;
	p_lcd->LCD_Backlight_Value = LCD_BACKLIGHT;
	p_lcd->LCD_Columns = p_cols;
	p_lcd->LCD_Rows = p_rows;
	p_lcd->hi2c = p_hi2c;
	p_lcd->LCD_Display_Option = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;

	// 4 bit initialization
	HAL_Delay(50);  // wait for >40ms
	LCD_Send_Command(p_lcd, 0x30);
	HAL_Delay(5);  // wait for >4.1ms
	LCD_Send_Command(p_lcd, 0x30);
	HAL_Delay(1);  // wait for >100us
	LCD_Send_Command(p_lcd, 0x30);
	HAL_Delay(10);
	LCD_Send_Command(p_lcd, 0x20);  // 4bit mode
	HAL_Delay(10);

	// Display initialization
	LCD_Send_Command(p_lcd, 0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	LCD_Send_Command(p_lcd, 0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	LCD_Send_Command(p_lcd, 0x01);  // clear display
	HAL_Delay(1);
	HAL_Delay(1);
	LCD_Send_Command(p_lcd, 0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	LCD_Send_Command(p_lcd, LCD_DISPLAYCONTROL | p_lcd->LCD_Display_Option); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void LCD_Write(LCD_I2C_HandleTypeDef *p_lcd, const char *p_str, ...){
	char t_stringArray[20] = {};

	va_list t_args;
	va_start(t_args, p_str);
	vsprintf(t_stringArray, p_str, t_args);
	va_end(t_args);

	for(int i = 0; i < strlen(t_stringArray) && i < p_lcd->LCD_Columns; i++){
		LCD_Send_Data(p_lcd, t_stringArray[i]);
	}
}

void LCD_Write_String(LCD_I2C_HandleTypeDef *p_lcd, const char *p_str){
	for(int i = 0; i < strlen(p_str) && i < p_lcd->LCD_Columns; i++){
		if(p_str[i] == '\\'){
			LCD_Write_Custom_Char(p_lcd, 1);
		} else if(p_str[i] == '~'){
			LCD_Write_Custom_Char(p_lcd, 2);
		} else{
			LCD_Send_Data(p_lcd, p_str[i]);
		}
	}
}

void LCD_Write_Custom_Char(LCD_I2C_HandleTypeDef *p_lcd, char p_location){
	LCD_Send_Data(p_lcd, p_location);
}

void LCD_Clear(LCD_I2C_HandleTypeDef *p_lcd){
	LCD_Send_Command(p_lcd, LCD_CLEARDISPLAY);
	HAL_Delay(2);
	LCD_Set_Cursor(p_lcd, 0, 0);
}

void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_lcd, uint8_t p_col, uint8_t p_row){
	uint8_t t_row_Offets[] = {0x00, 0x40, 0x14, 0x54};
	if(p_row > p_lcd->LCD_Rows) p_row = p_lcd->LCD_Rows - 1;
	LCD_Send_Command(p_lcd, LCD_SETDDRAMADDR | (p_col + t_row_Offets[p_row]));
}

void LCD_Create_Char(LCD_I2C_HandleTypeDef *p_lcd, uint8_t p_location, uint8_t p_charMap[]){
	p_location &= 7; // only have 8 locations 0-7.
	p_location <<= 3;
	LCD_Send_Command(p_lcd, LCD_SETCGRAMADDR | p_location);
	for(int i = 0; i < 8; i++) LCD_Send_Data(p_lcd, p_charMap[i]);
}

void LCD_Backlight(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Backlight_Value = LCD_BACKLIGHT;
	LCD_Send_Command(p_lcd, 0);
}

void LCD_No_Backlight(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Backlight_Value = LCD_NOBACKLIGHT;
	LCD_Send_Command(p_lcd, 0);
}

void LCD_Cursor_Blink(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Display_Option |= LCD_BLINKON;
	LCD_Send_Command(p_lcd, LCD_DISPLAYCONTROL | p_lcd->LCD_Display_Option);
}

void LCD_Cursor_No_Blink(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Display_Option &= ~LCD_BLINKON;
	LCD_Send_Command(p_lcd, LCD_DISPLAYCONTROL | p_lcd->LCD_Display_Option);
}

void LCD_Display_On(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Display_Option |= LCD_DISPLAYON;
	LCD_Send_Command(p_lcd, LCD_DISPLAYCONTROL | p_lcd->LCD_Display_Option);
}

void LCD_Display_Off(LCD_I2C_HandleTypeDef *p_lcd){
	p_lcd->LCD_Display_Option &= ~LCD_DISPLAYON;
	LCD_Send_Command(p_lcd, LCD_DISPLAYCONTROL | p_lcd->LCD_Display_Option);
}

void LCD_Send_Command(LCD_I2C_HandleTypeDef *p_lcd, char cmd){
	char p_data_H, p_data_L;
	uint8_t p_I2C_Bufer[4];
	p_data_H = cmd & 0xF0;
	p_data_L = (cmd << 4) & 0xF0;

	p_I2C_Bufer[0] = p_data_H | p_lcd->LCD_Backlight_Value | En;
	p_I2C_Bufer[1] = p_data_H | p_lcd->LCD_Backlight_Value;
	p_I2C_Bufer[2] = p_data_L | p_lcd->LCD_Backlight_Value | En;
	p_I2C_Bufer[3] = p_data_L | p_lcd->LCD_Backlight_Value;

	HAL_I2C_Master_Transmit(p_lcd->hi2c, p_lcd->SLAVE_ADDRESS, p_I2C_Bufer, 4, 100);
}

void LCD_Send_Data(LCD_I2C_HandleTypeDef *p_lcd, char data){
	char p_data_H, p_data_L;
	uint8_t p_I2C_Bufer[4];
	p_data_H = data & 0xF0;
	p_data_L = (data << 4) & 0xF0;

	p_I2C_Bufer[0] = p_data_H | p_lcd->LCD_Backlight_Value | En | Rs;
	p_I2C_Bufer[1] = p_data_H | p_lcd->LCD_Backlight_Value | Rs;
	p_I2C_Bufer[2] = p_data_L | p_lcd->LCD_Backlight_Value | En | Rs;
	p_I2C_Bufer[3] = p_data_L | p_lcd->LCD_Backlight_Value | Rs;

	HAL_I2C_Master_Transmit(p_lcd->hi2c, p_lcd->SLAVE_ADDRESS, p_I2C_Bufer, 4, 100);
}

