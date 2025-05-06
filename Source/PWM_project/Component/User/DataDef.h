//==================================================================================================
//
// 	File Name		ESPMain.c
//
//	CPU Type		ESP32-S3
//	Builder			
//					
//	Coding			
//
//	Outline			
//					
//
//	History			Ver.0.01	2024.12.14	V.Vu	New
//
//==================================================================================================
//==================================================================================================
//	Local Compile Option
//==================================================================================================
#ifndef DATA_DEF_H
#define DATA_DEF_H

#include <stdint.h>
#include <stdio.h>

//==================================================================================================
//	Declare constants that are commonly used in all files
//==================================================================================================
#define U1_NO_PRESS		((uint8_t)0)
#define U1_SHORT_PRESS	((uint8_t)1)
#define U1_LONG_PRESS	((uint8_t)2)

#define U1_NO_ROTATE	((uint8_t)0)
#define U1_INCREASE		((uint8_t)1)
#define U1_DECREASE		((uint8_t)2)

//==================================================================================================
//	Declare a structure template that is commonly used for all files
//==================================================================================================
typedef struct 
{
	uint8_t u1_Flag;
	uint8_t u1_Cnt;
} ST_TIMER;

typedef struct 
{
	uint32_t u4_FreqValue;
	uint8_t	 u1_DutyValue;
} ST_PROFILE;

typedef enum
{
	EM_UI_MAIN = 0,
	EM_UI_SEL_MODE,
	EM_UI_FREQ_MODE,
	EM_UI_DUTY_MODE
} EM_MODE_SEQ;

typedef struct 
{
	uint8_t u1_ModeSel_Change;
	uint8_t u1_ModeSel_Cur;
} ST_MODE_SELECT;

typedef struct 
{
	uint8_t u1_Flag;
	uint32_t u4_Cnt;
} ST_ENCODER_SW;

#endif
/* ************************************* End of File ******************************************** */
