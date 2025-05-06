//==================================================================================================
//
// 	File Name		RamDef.h
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
#ifndef RAM_DEF_H
#define RAM_DEF_H

#include "DataDef.h"

#ifdef REAL_RAM											
#define EXTERN
#else
#define EXTERN		extern
#endif

#define U1_OFF  ((uint8_t)0)
#define U1_ON   ((uint8_t)1)

EXTERN ST_TIMER st_MainLoop;

EXTERN ST_PROFILE st_CH1_ProFile;
EXTERN EM_MODE_SEQ em_ModeSeq;
EXTERN ST_MODE_SELECT st_ModeSel;

EXTERN uint8_t u1_Init_First_F;

EXTERN uint32_t u4_Cnt;
//==================================================================================================
//	RAM area (global variable definition/declaration)
//==================================================================================================

#endif
/* ************************************* End of File ******************************************** */
