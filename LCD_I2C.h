#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"

#include "stdarg.h"
#include "stdio.h"
#include "stdint.h"
#include "string.h"

// commands
#define	LCD_CLEARDISPLAY				0x01
#define LCD_RETURNHOME					0x02
#define LCD_ENTRYMODESET 				0x04
#define LCD_DISPLAYCONTROL 				0x08
#define LCD_CURSORSHIFT 				0x10
#define LCD_FUNCTIONSET 				0x20
#define LCD_SETCGRAMADDR 				0x40
#define LCD_SETDDRAMADDR 				0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 					0x00
#define LCD_ENTRYLEFT 					0x02
#define LCD_ENTRYSHIFTINCREMENT 		0x01
#define LCD_ENTRYSHIFTDECREMENT 		0x00

// flags for display on/off control
#define LCD_DISPLAYON					0x04
#define LCD_DISPLAYOFF 					0x00
#define LCD_CURSORON 					0x02
#define LCD_CURSOROFF 					0x00
#define LCD_BLINKON 					0x01
#define LCD_BLINKOFF 					0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 				0x08
#define LCD_CURSORMOVE 					0x00
#define LCD_MOVERIGHT 					0x04
#define LCD_MOVELEFT 					0x00

// flags for function set
#define LCD_8BITMODE 					0x10
#define LCD_4BITMODE 					0x00
#define LCD_2LINE 						0x08
#define LCD_1LINE 						0x00
#define LCD_5x10DOTS 					0x04
#define LCD_5x8DOTS 					0x00

// flags for backlight control
#define LCD_BACKLIGHT 					0x08
#define LCD_NOBACKLIGHT 				0x00

#define En 								0b00000100  // Enable bit
#define Rw 								0b00000010  // Read/Write bit
#define Rs 								0b00000001  // Register select bit

typedef struct{
	uint8_t LCD_Columns;
	uint8_t LCD_Rows;
	uint8_t SLAVE_ADDRESS;
	uint8_t LCD_Backlight_Value;
	uint8_t LCD_Display_Option;
	I2C_HandleTypeDef *hi2c;
} LCD_I2C_HandleTypeDef;

void LCD_Init(LCD_I2C_HandleTypeDef *p_lcd, I2C_HandleTypeDef *p_hi2c, uint8_t p_cols, uint8_t p_rows, uint8_t p_SLAVE_ADDRESS);
void LCD_Write(LCD_I2C_HandleTypeDef *p_lcd, const char *p_str, ...);
void LCD_Write_String(LCD_I2C_HandleTypeDef *p_lcd, const char *p_str);
void LCD_Write_Custom_Char(LCD_I2C_HandleTypeDef *p_lcd, char p_location);
void LCD_Clear(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_Set_Cursor(LCD_I2C_HandleTypeDef *p_lcd, uint8_t p_col, uint8_t p_row);
void LCD_Create_Char(LCD_I2C_HandleTypeDef *p_lcd, uint8_t p_location, uint8_t p_charMap[]);
void LCD_Backlight(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_No_Backlight(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_Cursor_Blink(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_Cursor_No_Blink(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_Display_On(LCD_I2C_HandleTypeDef *p_lcd);
void LCD_Display_Off(LCD_I2C_HandleTypeDef *p_lcd);

void LCD_Send_Command(LCD_I2C_HandleTypeDef *p_lcd, char p_cmd);
void LCD_Send_Data(LCD_I2C_HandleTypeDef *p_lcd, char p_data);

#endif
