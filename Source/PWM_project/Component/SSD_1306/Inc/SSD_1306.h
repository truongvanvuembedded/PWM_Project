
//==================================================================================================
//
// 	File Name		CP_uart.h
//
//	CPU Type		ESP32-S3
//	Builder			
//					
//	Coding			
//
//	Outline			-

//
//	History			
//==================================================================================================
//	Local Compile Option
//==================================================================================================
#ifndef SSD1306_H
#define SSD1306_H 

#include "fonts.h"
#include <stdlib.h>
#include <string.h>
//==================================================================================================
//	Define
//==================================================================================================

//==================================================================================================
//	Typedef
//==================================================================================================

typedef enum {
	SSD1306_COLOR_BLACK = 0x00, /*!< Black color, no pixel */
	SSD1306_COLOR_WHITE = 0x01  /*!< Pixel is set. Color depends on LCD */
} SSD1306_COLOR_t;

//==================================================================================================
//	Function prototype declaration
//==================================================================================================

void SSD1306_Init( void );
void SSD1306_UpdateScreen( void );
void SSD1306_GotoXY(uint16_t x, uint16_t y);
char SSD1306_Puts(char* str, FontDef_t* Font, SSD1306_COLOR_t color) ;
void SSD1306_Fill(SSD1306_COLOR_t color);
void SSD1306_Clear (void);
void SSD1306_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, SSD1306_COLOR_t c);


void SSD1306_ScrollRight(uint8_t start_row, uint8_t end_row);
void SSD1306_ScrollLeft(uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagright(uint8_t start_row, uint8_t end_row);
void SSD1306_Scrolldiagleft(uint8_t start_row, uint8_t end_row);
void SSD1306_Stopscroll(void);

void SSD1306_InvertDisplay (int i);

void SSD1306_DrawBitmap(int16_t x, int16_t y, const unsigned char* bitmap, int16_t w, int16_t h, uint16_t color);

void SSD1306_ToggleInvert(void);

void SSD1306_DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void SSD1306_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, SSD1306_COLOR_t c);
void SSD1306_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
void SSD1306_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, SSD1306_COLOR_t color);
void SSD1306_DrawCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);
void SSD1306_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, SSD1306_COLOR_t c);

void SSD1306_ON(void);
void SSD1306_OFF(void);

#endif
