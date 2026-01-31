//==================================================================================================
//
//	File Name		Drv_GPIO_Output.c
//	CPU Type		ESP32-S3-WROOM-1
//	Builder 		ESP-IDF V5.3.1
//					
//	Customer		
//	Coding			V.Vu 
//	History			Ver.0.01	2024.12.12	V.Vu New
//	Memo			Output driver
//
//==================================================================================================
//==================================================================================================
//	#pragma section
//==================================================================================================

//==================================================================================================
//	Local Compile Option
//==================================================================================================

//==================================================================================================
//	Header File
//==================================================================================================
#include <stdio.h>
#include "RamDef.h"
#include "DataDef.h"
#include "FuncDef.h"
#include "main.h"
//==================================================================================================
//	Local define
//==================================================================================================
#define U4_MAIN_LOOOP           ((uint32_t)10)			// 10ms
#define U4_LONG_PRESS_TIME      ((uint32_t)1500 / U4_MAIN_LOOOP)	// 3000MS
#define U4_SHORT_PRESS_TIME     ((uint32_t)200 / U4_MAIN_LOOOP)	// 100MS
//==================================================================================================
//	Local define I/O
//==================================================================================================

//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
static uint8_t          EncoderRotate;
static uint8_t          EncoderRotate_Last;
static ST_ENCODER_SW    st_SW_Long;
static ST_ENCODER_SW    st_SW_Short;
//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================
static void SWCounter(ST_ENCODER_SW *st_SW, uint32_t u4_Time);
static void clear_EncoderSW_State(void);
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Encoder_Init
//	Function:	    Init 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void Encoder_Init( void )
{
    EncoderRotate = U1_NO_ROTATE;
    EncoderRotate_Last = U1_NO_ROTATE;
    clear_EncoderSW_State(); // Clear state
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Encoder_Rotate_Process_IT
//	Function:	    Init 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void Encoder_Rotate_Process_IT( void )
{

    if ( HAL_GPIO_ReadPin( Encoder_IT_GPIO_Port, Encoder_IT_Pin ) == GPIO_PIN_RESET )   
    {
        if ( HAL_GPIO_ReadPin( Encoder_GPIO_Port, Encoder_Pin ) == GPIO_PIN_RESET )     // Decrease
        {
            EncoderRotate_Last = U1_DECREASE;
        }
        else                                                                            // Increase
        {
            EncoderRotate_Last = U1_INCREASE;
        }
    }
    else
    {
        EncoderRotate = EncoderRotate_Last; // Save last state
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        u1_Encoder_SW_Process
//	Function:	    Init 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t u1_Encoder_SW_Process( void )
{
    uint8_t u1_Ret;

    // Check switch state
    if ( HAL_GPIO_ReadPin( SW_GPIO_Port, SW_Pin ) == GPIO_PIN_RESET )
    {
        // Counter for determine long press
        SWCounter( &st_SW_Long, U4_LONG_PRESS_TIME );
        SWCounter( &st_SW_Short, U4_SHORT_PRESS_TIME );
        u1_Ret = U1_NO_PRESS; // No press detected
    }
    else
    {
        // If switch is released, check if it was long or short press
        if ( st_SW_Long.u1_Flag == U1_ON )
        {
            u1_Ret = U1_LONG_PRESS; // Long press detected
        }
        else if ( st_SW_Short.u1_Flag == U1_ON )
        {
            u1_Ret = U1_SHORT_PRESS; // Short press detected
        }
        else{
            u1_Ret = U1_NO_PRESS; // No press detected
        }
        clear_EncoderSW_State(); // Clear state
    }
    return u1_Ret; // Return the state of the switch
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        clear_Encoder_State
//	Function:	    Clear flag and counter of Encoder Switch state 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void clear_EncoderSW_State(void)
{
    // Reset counter when switch is released
    st_SW_Long.u4_Cnt = 0;
    st_SW_Long.u1_Flag = U1_OFF;
    st_SW_Short.u4_Cnt = 0;
    st_SW_Short.u1_Flag = U1_OFF;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        clear_Encoder_State
//	Function:	    Clear flag and counter of Encoder Switch state 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t u1_Give_Encoder_Rotate(void)
{
    return EncoderRotate;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        clear_Encoder_State
//	Function:	    Clear flag and counter of Encoder Switch state 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void Clear_Encoder_State(void)
{
    EncoderRotate = U1_NO_ROTATE;
    EncoderRotate_Last = U1_NO_ROTATE;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        SWCounter
//	Function:	    SW Counter for determine long press or short press
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void SWCounter(ST_ENCODER_SW *st_SW, uint32_t u4_Time)
{
    if ( st_SW->u4_Cnt < u4_Time ) // 100ms
    {
        st_SW->u4_Cnt++;
    }
    // Copare cnt with threshold value
    if ( st_SW->u4_Cnt >= u4_Time ) // 100ms
    {
        st_SW->u1_Flag = U1_ON;
    }
}
/* ************************************* End of File ******************************************** */

