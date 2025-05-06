//==================================================================================================
//
// 	File Name		FuncDef.h
//	Customer		
//	Coding			
//	History			Ver.0.01	2024.12.14	V.Vu	New
//  Outline			
//
//==================================================================================================
//==================================================================================================
//	Local Compile Option
//==================================================================================================
#include <stdio.h>
#ifndef FUNC_DEF_H
#define FUNC_DEF_H

//==================================================================================================
//	Function Prototype
//==================================================================================================
// Encoder.c
void Encoder_Init( void );
void Encoder_Rotate_Process_IT( void );
uint8_t u1_Encoder_SW_Process( void );
uint8_t u1_Give_Encoder_Rotate(void);
void Clear_Encoder_State(void);
// Timer.c
void Timer_Init( void );
void PWM_Change( void );
void Timer_Process( ST_TIMER* apst_Timer, uint8_t au1_time_ms );
// Use_Interface.c
void UI_Sequence_Update( uint8_t En_SW_F, uint8_t En_Rot_F );
void UI_Init( void );

#endif
/* ************************************* End of File ******************************************** */
