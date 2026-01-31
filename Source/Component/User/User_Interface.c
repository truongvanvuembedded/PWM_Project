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
#include <string.h>
#include "SSD_1306.h"
#include "DataDef.h"
#include "RamDef.h"
#include "FuncDef.h"
#include "main.h"
#include <math.h>
//==================================================================================================
//	Local define
//==================================================================================================
//==================================================================================================
//	Local define I/O
//==================================================================================================
#define U1_FREQ_MODE 	((uint8_t)0)
#define U1_DUTY_MODE 	((uint8_t)1)

#define U1_DUTY_MAX_VAL	((uint8_t)99)
#define U1_DUTY_MIN_VAL	((uint8_t)0)

#define U1_FREQ_MAX_VAL	((uint8_t)99999)
#define U1_FREQ_MIN_VAL	((uint8_t)0)
//==================================================================================================
//	Local Struct Template
//==================================================================================================

//==================================================================================================
//	Local RAM 
//==================================================================================================
static uint8_t u1_FreqPosi_Pointer;					// Point to the position of Freq value (0 -> 4)
static uint8_t u1_FreqPosi_Indicate;				// Indicate the this number or underline
static uint8_t u1_Freq_Number_Change;				// Flag notify number of Freq change
static uint8_t u1_Freq_UnderLine_Change;			// Flag notify under line of Freq change

static uint8_t u1_DutyValue_Change;					// Duty value change

static uint8_t u1_UnderLine_Pos_x;					
static uint8_t *pu1_FreqNumber;

static uint8_t u1_FreqMode_EncoderChange;

static uint8_t u1_UI_MainModeFirst_F;					// Flag to indicate first time of UI main mode
static uint8_t u1_UI_SeleModeFirst_F;					// Flag to indicate first time of UI select mode
static uint8_t u1_UI_FreqModeFirst_F;					// Flag to indicate first time of UI Frequency mode
static uint8_t u1_UI_DutyModeFirst_F;					// Flag to indicate first time of UI Duty cycle mode

//==================================================================================================
//	Local ROM
//==================================================================================================

//==================================================================================================
//	Local Function Prototype
//==================================================================================================
// Upate UI when value change
static void UI_MainUpdate( void );
static void UI_SelectMode( void );
static void UI_FreqModeSelected( void );
static void UI_DutyModeSelected( void );
// Change value if Input change
static void Change_BaseEncoder( uint8_t u1_En_SW_F, uint8_t u1_Encoder_Sw_F );
static void Freq_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot);
static void Duty_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot);
static void Main_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot);
static void Select_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot);
typedef struct {
	uint8_t u1_Pos_0;
	uint8_t u1_Pos_1;
	uint8_t u1_Pos_2;
	uint8_t u1_Pos_3;
	uint8_t u1_Pos_4;
} ST_FREQ_POS;

static ST_FREQ_POS st_FreqNumber;
//==================================================================================================
//	Source Code
//==================================================================================================
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_Init
//	Function:	    Init 
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void UI_Init( void )
{
	u1_Init_First_F = U1_ON;
	
    // Init UI
	em_ModeSeq = EM_UI_MAIN;
	st_ModeSel.u1_ModeSel_Change = U1_FREQ_MODE;
	st_ModeSel.u1_ModeSel_Cur = U1_FREQ_MODE;

	u1_UI_MainModeFirst_F = U1_ON;
	u1_UI_SeleModeFirst_F = U1_ON;
	u1_UI_FreqModeFirst_F = U1_ON;
	u1_UI_DutyModeFirst_F = U1_ON;

	u1_FreqPosi_Pointer = 0;
	u1_FreqPosi_Indicate = 0;

	u1_Freq_Number_Change = U1_OFF;
	u1_Freq_UnderLine_Change = U1_OFF;
	u1_DutyValue_Change = U1_OFF;

	st_FreqNumber.u1_Pos_0 = 0x00;
	st_FreqNumber.u1_Pos_1 = 0x00;
	st_FreqNumber.u1_Pos_2 = 0x00;
	st_FreqNumber.u1_Pos_3 = 0x00;
	st_FreqNumber.u1_Pos_4 = 0x00;

	u1_UnderLine_Pos_x = 85;

	pu1_FreqNumber = (uint8_t *) &st_FreqNumber;

	u1_FreqMode_EncoderChange = U1_NO_ROTATE;
    // Init SSD1306 driver
	SSD1306_Init();
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_MainUpdate
//	Function:	    Update value in main User interface
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UI_MainUpdate( void )
{
	if ( u1_UI_MainModeFirst_F == U1_ON )
	{
		u1_UI_MainModeFirst_F = U1_OFF;

		SSD1306_Clear();

		char apu1_Freq_to_ASCII[10];
		char pu1_Duty_to_ASCII[10];

		sprintf((char *)apu1_Freq_to_ASCII, "%d Hz", (int)st_CH1_ProFile.u4_FreqValue);
		sprintf((char *)pu1_Duty_to_ASCII, "%d %%", (int)st_CH1_ProFile.u1_DutyValue);

		SSD1306_GotoXY (0,0);
		SSD1306_Puts ("Freq:", &Font_7x10, 1);
		SSD1306_GotoXY (40,5);
		SSD1306_Puts (apu1_Freq_to_ASCII, &Font_7x10, 1);

		SSD1306_GotoXY (0,25);
		SSD1306_Puts ("Duty:", &Font_7x10, 1);
		SSD1306_GotoXY (50,40);
		SSD1306_Puts (pu1_Duty_to_ASCII, &Font_11x18, 1);

		SSD1306_UpdateScreen(); //display
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_SelectMode
//	Function:	    UI when User select mode (Include Freq and Duty Cycle Mode)
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UI_SelectMode( void )
{
	if ( u1_UI_SeleModeFirst_F == U1_ON )
	{
		u1_UI_SeleModeFirst_F = U1_OFF;

		SSD1306_Clear();
		SSD1306_GotoXY (22,0);
		SSD1306_Puts ("Choose Mode:", &Font_7x10, 1);
		SSD1306_GotoXY (8,25);
		SSD1306_Puts ("Freq", &Font_11x18, 1);
		SSD1306_GotoXY (72,25);
		SSD1306_Puts ("Duty", &Font_11x18, 1);
		SSD1306_DrawLine(8,50,50,50,1);
		SSD1306_UpdateScreen(); //display
	}
	else{
		if ( st_ModeSel.u1_ModeSel_Cur != st_ModeSel.u1_ModeSel_Change )
		{
			st_ModeSel.u1_ModeSel_Cur = st_ModeSel.u1_ModeSel_Change;
			if ( st_ModeSel.u1_ModeSel_Cur == U1_FREQ_MODE )
			{
				SSD1306_DrawLine(72,50,115,50,0);
				SSD1306_DrawLine(8,50,50,50,1);
			}
			else if ( st_ModeSel.u1_ModeSel_Cur == U1_DUTY_MODE )
			{
				SSD1306_DrawLine(72,50,115,50,1);
				SSD1306_DrawLine(8,50,50,50,0);
			}
			SSD1306_UpdateScreen();
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_FreqModeSelected
//	Function:	    Return when User choose Freq mode
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UI_FreqModeSelected( void )
{
	char apu1_Freq_to_ASCII[5];

	if ( u1_UI_FreqModeFirst_F == U1_ON )
	{
		u1_UI_FreqModeFirst_F = U1_OFF;										// Turn off clear UI
		sprintf((char *)apu1_Freq_to_ASCII, "%05d", (int)st_CH1_ProFile.u4_FreqValue);	// Convert number to ASCII with 5 digits
		SSD1306_Clear();																// Update UI
		SSD1306_GotoXY (30,5);
		SSD1306_Puts ("Freq Mode:", &Font_7x10, 1);
		SSD1306_GotoXY (20,30);
		SSD1306_Puts (apu1_Freq_to_ASCII, &Font_16x26, 1);
		SSD1306_GotoXY (110,42);
		SSD1306_Puts ("Hz", &Font_7x10, 1);
		// Under line
		SSD1306_DrawLine(u1_UnderLine_Pos_x,57,(u1_UnderLine_Pos_x+14),57,1);						// Draw Under line 
		SSD1306_UpdateScreen(); 
	}
	else
	{
		if ( u1_Freq_Number_Change == U1_ON )														// Change number (0-9)
		{
			u1_Freq_Number_Change = U1_OFF;

			sprintf((char *)apu1_Freq_to_ASCII, "%d", (int)pu1_FreqNumber[u1_FreqPosi_Pointer]);	// Copy data

			SSD1306_DrawFilledRectangle((84 - (16*u1_FreqPosi_Pointer)) ,30, 16, 26, 0);			// Clear
			//SSD1306_UpdateScreen();	
			SSD1306_GotoXY ((84 - (16*u1_FreqPosi_Pointer)),30);									
			SSD1306_Puts (apu1_Freq_to_ASCII, &Font_16x26, 1);
			SSD1306_UpdateScreen();
		}
		else if ( u1_Freq_UnderLine_Change == U1_ON )												// Change the position of under line	
		{
			u1_Freq_UnderLine_Change = U1_OFF;
			// Clear
			SSD1306_DrawLine(u1_UnderLine_Pos_x,57,(u1_UnderLine_Pos_x + 14),57,0);
			//SSD1306_UpdateScreen(); //display
			if ( u1_FreqMode_EncoderChange == U1_INCREASE )
			{
				u1_UnderLine_Pos_x += 16;
				if ( u1_UnderLine_Pos_x > 84 )
				{
					u1_UnderLine_Pos_x = 84;
				}
			}
			else if ( u1_FreqMode_EncoderChange == U1_DECREASE )
			{
				u1_UnderLine_Pos_x -= 16;
				if ( u1_UnderLine_Pos_x < 20 )
				{
					u1_UnderLine_Pos_x = 20;
				}
			}
			u1_FreqMode_EncoderChange = U1_NO_ROTATE;

			// Update under line
			SSD1306_DrawLine(u1_UnderLine_Pos_x,57,(u1_UnderLine_Pos_x + 14),57,1);
			SSD1306_UpdateScreen(); //display
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_main_update
//	Function:	    Udpate duty cycle when User select Duty mode
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void UI_DutyModeSelected( void )
{
	char pu1_Duty_to_ASCII[4];
    sprintf(pu1_Duty_to_ASCII, "%d", (int)st_CH1_ProFile.u1_DutyValue);

	if ( u1_UI_DutyModeFirst_F == U1_ON )
	{
		u1_UI_DutyModeFirst_F = U1_OFF;
		SSD1306_Clear();
		SSD1306_GotoXY (30,5);
		SSD1306_Puts ("Duty Mode:", &Font_7x10, 1);
		SSD1306_GotoXY (40,30);
		SSD1306_Puts (pu1_Duty_to_ASCII, &Font_16x26, 1);
		SSD1306_GotoXY (80,33);
		SSD1306_Puts ("%", &Font_11x18, 1);
		SSD1306_UpdateScreen(); //display0
	}
	else{
		if ( u1_DutyValue_Change == U1_ON ){
			u1_DutyValue_Change = U1_OFF;

			// Clear the old value
			// SSD1306_GotoXY (40,30);
			SSD1306_DrawFilledRectangle(40,30,32,26,0);
			//SSD1306_UpdateScreen();
			// Render new value
			SSD1306_GotoXY (40,30);
			SSD1306_Puts (pu1_Duty_to_ASCII, &Font_16x26, 1);
			SSD1306_UpdateScreen();
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Change_BaseEncoder
//	Function:	    -
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void Change_BaseEncoder( uint8_t u1_En_SW_F, uint8_t u1_En_Rot_F )
{
	switch (em_ModeSeq)
	{
		case EM_UI_MAIN:	// If Encoder rotate change in UI main do nothing
			Main_Mode_Processing(u1_En_SW_F, u1_En_Rot_F);
			break;
		case EM_UI_SEL_MODE:	// If Encoder rotate change in Select UI Change from Frequency or duty cycle UI
			Select_Mode_Processing(u1_En_SW_F, u1_En_Rot_F);
			break;
		case EM_UI_FREQ_MODE:
			Freq_Mode_Processing(u1_En_SW_F, u1_En_Rot_F);
			break;
		case EM_UI_DUTY_MODE:
			Duty_Mode_Processing(u1_En_SW_F, u1_En_Rot_F);
			break;
		default:
			break;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        UI_Sequence_Update
//	Function:	    -
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
void UI_Sequence_Update( uint8_t u1_En_SW_F, uint8_t u1_En_Rot_F )
{
	// If Encode change or If button is switched
	if ( u1_En_Rot_F != U1_NO_ROTATE || u1_En_SW_F != U1_NO_PRESS)
	{
		Change_BaseEncoder(u1_En_SW_F, u1_En_Rot_F);
	}

	// UI Sequence processing
	switch (em_ModeSeq)
	{
		case EM_UI_MAIN:
			UI_MainUpdate();
			break;
		case EM_UI_SEL_MODE:
			UI_SelectMode();
			break;
		case EM_UI_FREQ_MODE:
			UI_FreqModeSelected();
			break;
		case EM_UI_DUTY_MODE:
			UI_DutyModeSelected();
			break;
		default:
			break;
	}

}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Freq_Mode_Processing
//	Function:	    Funtion change value of Freqency in Freq mode
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void Freq_Mode_Processing(uint8_t u1_En_SW_F, uint8_t u1_En_Rot_F)
{
	uint32_t au4_FreqValue_Temp;
	au4_FreqValue_Temp = (uint32_t)0;

	if ( u1_FreqPosi_Indicate == U1_ON )						// Change the value of number is selected
	{
		if ( u1_En_Rot_F == U1_INCREASE )
		{
			pu1_FreqNumber[u1_FreqPosi_Pointer] += 1;
			if ( pu1_FreqNumber[u1_FreqPosi_Pointer] > 9 )
			{
				pu1_FreqNumber[u1_FreqPosi_Pointer] = 9;
			}
			u1_Freq_Number_Change = U1_ON;							// Notify Freq number need change
		}
		else if ( u1_En_Rot_F == U1_DECREASE )
		{
			if ( pu1_FreqNumber[u1_FreqPosi_Pointer] != 0 )
			{
				pu1_FreqNumber[u1_FreqPosi_Pointer] -= 1;
			}
			u1_Freq_Number_Change = U1_ON;							// Notify Freq number need change
		}	
	}
	else
	{															// Change the pointer to number position
		u1_Freq_UnderLine_Change = U1_ON;
		if ( u1_En_Rot_F == U1_DECREASE )
		{
			u1_FreqMode_EncoderChange = U1_DECREASE;
			u1_FreqPosi_Pointer++;
			if ( u1_FreqPosi_Pointer >= 4 )
			{
				u1_FreqPosi_Pointer = 4;
			}
		}
		else if ( u1_En_Rot_F == U1_INCREASE )
		{
			u1_FreqMode_EncoderChange = U1_INCREASE;
			if ( u1_FreqPosi_Pointer != 0 )
			{
				u1_FreqPosi_Pointer--;
			}
		}
	}



	// Exit UI main and save value
	if ( u1_En_SW_F == U1_LONG_PRESS )
	{
		em_ModeSeq = EM_UI_MAIN;
		u1_UI_FreqModeFirst_F = U1_ON;
		u1_FreqPosi_Pointer = 0;
		u1_FreqPosi_Indicate = U1_OFF;

		// Copy temp data to UI main
		for ( uint8_t au1_ForC = 0; au1_ForC < 5; au1_ForC++ )
		{
			au4_FreqValue_Temp += pu1_FreqNumber[au1_ForC] * pow(10,au1_ForC); 
		}
		if (au4_FreqValue_Temp >= U1_FREQ_MAX_VAL)
		{
			st_CH1_ProFile.u4_FreqValue = U1_FREQ_MAX_VAL;
		}
		else if (au4_FreqValue_Temp <= U1_FREQ_MIN_VAL)
		{
			st_CH1_ProFile.u4_FreqValue = U1_FREQ_MIN_VAL;
		}
		else
		{
			st_CH1_ProFile.u4_FreqValue = au4_FreqValue_Temp;
		}
	}
	else if (u1_En_SW_F == U1_SHORT_PRESS)
	{
		u1_FreqPosi_Indicate = !u1_FreqPosi_Indicate;
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Duty_Mode_Processing
//	Function:	    Funtion to change duty cycle in DutyMode
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void Duty_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot)
{
		// Increase and Decrease Duty value
		if ( u1_En_Rot == U1_INCREASE )
		{
			if ( st_CH1_ProFile.u1_DutyValue < U1_DUTY_MAX_VAL )
			{
				u1_DutyValue_Change = U1_ON;
				st_CH1_ProFile.u1_DutyValue++;
			}
			if ( st_CH1_ProFile.u1_DutyValue >= U1_DUTY_MAX_VAL )
			{
				st_CH1_ProFile.u1_DutyValue = U1_DUTY_MAX_VAL;
			}
		}
		else if ( u1_En_Rot == U1_DECREASE )
		{
			if ( st_CH1_ProFile.u1_DutyValue > U1_DUTY_MIN_VAL )
			{
				u1_DutyValue_Change = U1_ON;
				st_CH1_ProFile.u1_DutyValue--;
			}
		}
		else
		{
			u1_DutyValue_Change = U1_OFF;
		}

		// Exit UI main and save value
		if ( u1_En_SW == U1_LONG_PRESS )
		{
			em_ModeSeq = EM_UI_MAIN;
			u1_UI_DutyModeFirst_F = U1_ON;
		}
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Select_Mode_Processing
//	Function:	    Funtion to change UI when User rotate encoder (Change under line from Freq to Duty mode and reverse)
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void Select_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot)
{
	// Change under line when User change encoder (Freq or Duty cycle mode)
	if (u1_En_Rot != U1_NO_ROTATE)
	{
		st_ModeSel.u1_ModeSel_Change = (!st_ModeSel.u1_ModeSel_Change);	
	}
	
	// If long press is deteced back to UI main
	if ( u1_En_SW == U1_LONG_PRESS )
	{
		em_ModeSeq = EM_UI_MAIN;
	}
	else // If short press is deted switch to UI conserdering
	{
		if ( u1_En_SW == U1_SHORT_PRESS )
		{
			if ( st_ModeSel.u1_ModeSel_Cur == U1_FREQ_MODE )
			{
				em_ModeSeq = EM_UI_FREQ_MODE;
			}
			else
			{
				em_ModeSeq = EM_UI_DUTY_MODE;
			}
			u1_UI_SeleModeFirst_F = U1_ON;
			st_ModeSel.u1_ModeSel_Change = U1_FREQ_MODE;
			st_ModeSel.u1_ModeSel_Cur    = U1_FREQ_MODE;
		}
	}
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//	Name:	        Select_Mode_Processing
//	Function:	    Funtion to change UI when User rotate encoder (Change under line from Freq to Duty mode and reverse)
//	Argument:	    -
//	Return value:	-
//	Create:	        2024.11.27 V.Vu	New
//  Change:	        -
//	Remarks:	    -
//
////////////////////////////////////////////////////////////////////////////////////////////////////
static void Main_Mode_Processing(uint8_t u1_En_SW, uint8_t u1_En_Rot)
{
	UNUSED(u1_En_Rot);
	// If SW of Encoder press longer 3s -> switch to Select mode
	if (u1_En_SW == U1_LONG_PRESS)	
	{
		em_ModeSeq = EM_UI_SEL_MODE;
		u1_UI_MainModeFirst_F = U1_ON;
	}
}
/* ************************************* End of File ******************************************** */

