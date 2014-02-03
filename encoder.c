/*******************************************************************************
*
*	TITLE:		encoder.c 
*
*	VERSION:	0.5 (Beta)                           
*
*	DATE:		17-Dec-2005
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or
*				elsewhere without permission. Thanks.
*
*				Copyright ©2003-2006 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	20-Dec-2003  0.1  RKW - Original code.
*	18-Feb-2004  0.2  RKW - Reassigned the encoder digital inputs to run
*	                  on the FRC robot controller too.
*	01-Jan-2005  0.3  RKW - Get_Left_Encoder_Count(), Get_Right_Encoder_Count(),
*	                  Set_Left_Encoder_Count() and Set_Right_Encoder_Count()
*	                  functions added.
*	01-Jan-2005  0.3  RKW - Renamed Int_1_Handler() and Int_2_Handler() to
*	                  Left_Encoder_Int_Handler() and Right_Encoder_Int_Handler
*	                  respectively.
*	01-Jan-2005  0.3  RKW - Altered the interrupt service routines to easily
*	                  flip the direction the encoders count by altering the
*	                  RIGHT_ENCODER_TICK_DELTA and LEFT_ENCODER_TICK_DELTA
*	                  #defines found in encoder.h
*	06-Jan-2005  0.4  RKW - Rebuilt with C18 version 2.40.
*	17-Dec-2005  0.5  RKW - Added code to accommodate four more encoders on
*	                  interrupts 3 through 6. These additional encoder inputs
*	                  are optimized for position control.
*
*******************************************************************************/

#include "p18f8722.h"
#include "ifi_aliases.h"
#include "encoder.h"

// These variables are used to keep track of the encoder position over time.
// Though these are global variables, they shouldn't be modified directly. 
// Functions to modify these variables are included below.
#ifdef ENABLE_ENCODER_1
volatile long Encoder_1_Count = 0;
#endif

#ifdef ENABLE_ENCODER_2
volatile long Encoder_2_Count = 0;
#endif

#ifdef ENABLE_ENCODER_3
unsigned char Encoder_3_State;
volatile long Encoder_3_Count = 0;
#endif

#ifdef ENABLE_ENCODER_4
unsigned char Encoder_4_State;
volatile long Encoder_4_Count = 0;
#endif

#ifdef ENABLE_ENCODER_5
unsigned char Encoder_5_State;
volatile long Encoder_5_Count = 0;
#endif

#ifdef ENABLE_ENCODER_6
unsigned char Encoder_6_State;
volatile long Encoder_6_Count = 0;
#endif

// So that we'll know which interrupt pin changed state the next time through,
// the state of port b is saved in this variable each time the interrupt
// handler for interrupts 3 through 6 is called. This variable should be
// initialized to the current state of port b just before enabling interrupts
// 3 through 6.
unsigned char Old_Port_B = 0xFF;

/*******************************************************************************
*
*	FUNCTION:		Initialize_Encoders()
*
*	PURPOSE:		Initializes the encoder software.
*
*	CALLED FROM:	user_routines.c/User_Initialization()
*
*	PARAMETERS:		None
*
*	RETURNS:		Nothing
*
*	COMMENTS:
*
*******************************************************************************/
void Initialize_Encoders(void)  
{
	// if enabled in encoder.h, configure encoder 1's interrupt input
	#ifdef ENABLE_ENCODER_1

	// make sure interrupt 1 is configured as an input
	TRISBbits.TRISB2 = 1;

	// interrupt 1 is low priority
	INTCON3bits.INT2IP = 0;

	// trigger on rising edge
	INTCON2bits.INTEDG2 = 1;

	// make sure the interrupt flag is reset before enabling
	INTCON3bits.INT2IF = 0;

	// enable interrupt 1
	INTCON3bits.INT2IE = 1;
	#endif

	// if enabled in encoder.h, configure encoder 2's interrupt input
	#ifdef ENABLE_ENCODER_2

	// make sure interrupt 2 is configured as an input
	TRISBbits.TRISB3 = 1;

	// interrupt 2 is low priority
	INTCON2bits.INT3IP = 0;

	// trigger on rising edge
	INTCON2bits.INTEDG3 = 1;

	// make sure the interrupt flag is reset before enabling
	INTCON3bits.INT3IF = 0;

	// enable interrupt 2
	INTCON3bits.INT3IE = 1;
	#endif

	// if enabled in encoder.h, configure the interrupt input for encoders 3-6
	#ifdef ENABLE_ENCODER_3_6
	// make sure interrupts 3 through 6 are configured as inputs
	TRISBbits.TRISB4 = 1;
	TRISBbits.TRISB5 = 1;
	TRISBbits.TRISB6 = 1;
	TRISBbits.TRISB7 = 1;

	// interrupts 3 through 6 are low priority
  	INTCON2bits.RBIP = 0;

	// before enabling interrupts 3 through 6, take a snapshot of port b
	Old_Port_B = PORTB;

	// make sure the interrupt flag is reset before enabling
	INTCONbits.RBIF = 0;

	// enable interrupts 3 through 6
	INTCONbits.RBIE = 1;
	#endif
}


#ifdef ENABLE_ENCODER_1
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_1_Count()
*
*	PURPOSE:		Gets the current number of encoder 1 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 1 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_1_Count(void)
{
	long count;

	// Since we're about to access the Encoder_1_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_1_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT2IE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_1_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCON3bits.INT2IE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_1_Count()
*
*	PURPOSE:		Resets the encoder 1 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_1_Count(long to)
{
	// Since we're about to access the Encoder_1_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_1_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT2IE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_1_Count = to;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCON3bits.INT2IE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_1_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 1 interrupt handler is called when
*					the interrupt 1 pin changes from a logic 0 to a logic 1.
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_1_Int_Handler(void)
{
	// Encoder 1's phase a signal just transitioned from zero to one, causing 
	// this interrupt service routine to be called. We know that the encoder 
	// just rotated one count or "tick" so now check the logical state of the 
	// phase b signal to determine which way the the encoder shaft rotated.
	if(ENCODER_1_PHASE_B_PIN == 0)
	{
		Encoder_1_Count -= ENCODER_1_TICK_DELTA;
	}
	else
	{
		Encoder_1_Count += ENCODER_1_TICK_DELTA;
	}
}
#endif

#ifdef ENABLE_ENCODER_2
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_2_Count()
*
*	PURPOSE:		Gets the current number of encoder 2 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 2 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_2_Count(void)
{
	long count;

	// Since we're about to access the Encoder_2_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_2_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT3IE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_2_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCON3bits.INT3IE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_2_Count()
*
*	PURPOSE:		Resets the encoder 2 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_2_Count(long to)
{
	// Since we're about to access the Encoder_2_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_2_Count variable doesn't get altered while
	// we're using it.
	INTCON3bits.INT3IE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_2_Count = to;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCON3bits.INT3IE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_2_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 2 interrupt handler is called when
*					the interrupt 2 pin changes from a logic 0 to a logic 1.
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_2_Int_Handler(void)
{
	// Encoder 2's phase a signal just transitioned from zero to one, causing 
	// this interrupt service routine to be called. We know that the encoder 
	// just rotated one count or "tick" so now check the logical state of the 
	// phase b signal to determine which way the the encoder shaft rotated.
	if(ENCODER_2_PHASE_B_PIN == 0)
	{
		Encoder_2_Count -= ENCODER_2_TICK_DELTA;
	}
	else
	{
		Encoder_2_Count += ENCODER_2_TICK_DELTA;
	}
}
#endif

#ifdef ENABLE_ENCODER_3
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_3_Count()
*
*	PURPOSE:		Gets the current number of encoder 3 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 3 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_3_Count(void)
{
	long count;

	// Since we're about to access the Encoder_3_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_3_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_3_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_3_Count()
*
*	PURPOSE:		Resets the encoder 3 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_3_Count(void)
{
	// Since we're about to access the Encoder_3_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_3_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_3_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_3_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 3 interrupt handler is called when
*					the interrupt 3 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_3_Int_Handler(unsigned char state)
{
	// Encoder 3's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_3_PHASE_B_PIN == 0)
		{
			// backward
			if(Encoder_3_State == 1)
			{
				Encoder_3_Count -= ENCODER_3_TICK_DELTA;
			}
		}
		else
		{
			// forward
			if(Encoder_3_State == 0)
			{
				Encoder_3_Count += ENCODER_3_TICK_DELTA;
			}
		}
	}
	else
	{
		// falling-edge interrupt
		//   phase b is zero if moving forward
		//   phase b is one if moving backward
		Encoder_3_State = ENCODER_3_PHASE_B_PIN;
	}
}
#endif

#ifdef ENABLE_ENCODER_4
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_4_Count()
*
*	PURPOSE:		Gets the current number of encoder 4 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 4 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_4_Count(void)
{
	long count;

	// Since we're about to access the Encoder_4_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_4_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_4_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_4_Count()
*
*	PURPOSE:		Resets the encoder 4 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_4_Count(void)
{
	// Since we're about to access the Encoder_4_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_4_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_4_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_4_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 4 interrupt handler is called when
*					the interrupt 4 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_4_Int_Handler(unsigned char state)
{
	// Encoder 4's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_4_PHASE_B_PIN == 0)
		{
			// backward
			if(Encoder_4_State == 1)
			{
				Encoder_4_Count -= ENCODER_4_TICK_DELTA;
			}
		}
		else
		{
			// forward
			if(Encoder_4_State == 0)
			{
				Encoder_4_Count += ENCODER_4_TICK_DELTA;
			}
		}
	}
	else
	{
		// falling-edge interrupt
		//   phase b is zero if moving forward
		//   phase b is one if moving backward
		Encoder_4_State = ENCODER_4_PHASE_B_PIN;
	}
}
#endif

#ifdef ENABLE_ENCODER_5
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_5_Count()
*
*	PURPOSE:		Gets the current number of encoder 5 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 5 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_5_Count(void)
{
	long count;

	// Since we're about to access the Encoder_5_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_5_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_5_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_5_Count()
*
*	PURPOSE:		Resets the encoder 5 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_5_Count(void)
{
	// Since we're about to access the Encoder_5_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_5_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_5_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_5_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 5 interrupt handler is called when
*					the interrupt 5 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_5_Int_Handler(unsigned char state)
{
	// Encoder 5's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_5_PHASE_B_PIN == 0)
		{
			Encoder_5_Count -= ENCODER_5_TICK_DELTA;
		}
		else
		{
			Encoder_5_Count += ENCODER_5_TICK_DELTA;
		}
	}
	else
	{
		// falling-edge interrupt
		if(ENCODER_5_PHASE_B_PIN == 0)
		{
			Encoder_5_Count += ENCODER_5_TICK_DELTA;
		}
		else
		{
			Encoder_5_Count -= ENCODER_5_TICK_DELTA;
		}
	}
}
#endif

#ifdef ENABLE_ENCODER_6
/*******************************************************************************
*
*	FUNCTION:		Get_Encoder_6_Count()
*
*	PURPOSE:		Gets the current number of encoder 6 counts.		
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Number of encoder 6 counts since the last reset.
*
*	COMMENTS:
*
*******************************************************************************/
long Get_Encoder_6_Count(void)
{
	long count;

	// Since we're about to access the Encoder_6_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_6_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can get a local copy of the encoder count without fear
	// that we'll get corrupted data.
	count = Encoder_6_Count;

	// Okay, we have a local copy of the encoder count, so turn the 
	// encoder's interrupt back on.
	INTCONbits.RBIE = 1;

	// Return the encoder count to the caller.
	return(count);
}

/*******************************************************************************
*
*	FUNCTION:		Reset_Encoder_6_Count()
*
*	PURPOSE:		Resets the encoder 6 count to zero	
*
*	CALLED FROM:
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Reset_Encoder_6_Count(void)
{
	// Since we're about to access the Encoder_6_Count variable,
	// which can also be modified in the interrupt service routine,
	// we'll briefly disable the encoder's interrupt to make sure
	// that the Encoder_6_Count variable doesn't get altered while
	// we're using it.
	INTCONbits.RBIE = 0;

	// Now we can set the value of the encoder count without fear
	// that we'll write corrupted data.
	Encoder_6_Count = 0;

	// Okay, we're done updating the encoder count, so turn the 
	// left encoder's interrupt back on.
	INTCONbits.RBIE = 1;
}

/*******************************************************************************
*
*	FUNCTION:		Encoder_6_Int_Handler()
*
*	PURPOSE:		If enabled, the encoder 6 interrupt handler is called when
*					the interrupt 6 pin changes logic state
*
*	CALLED FROM:	user_routines_fast.c/InterruptHandlerLow()
*
*	PARAMETERS:		None.
*
*	RETURNS:		Nothing.
*
*	COMMENTS:
*
*******************************************************************************/
void Encoder_6_Int_Handler(unsigned char state)
{
	// Encoder 6's phase a signal just changed logic level, causing this 
	// interrupt service routine to be called.
	if(state == 1)
	{
		// rising-edge interrupt
		if(ENCODER_6_PHASE_B_PIN == 0)
		{
			Encoder_6_Count -= ENCODER_6_TICK_DELTA;
		}
		else
		{
			Encoder_6_Count += ENCODER_6_TICK_DELTA;
		}
	}
	else
	{
		// falling-edge interrupt
		if(ENCODER_6_PHASE_B_PIN == 0)
		{
			Encoder_6_Count += ENCODER_6_TICK_DELTA;
		}
		else
		{
			Encoder_6_Count -= ENCODER_6_TICK_DELTA;
		}
	}
}
#endif
