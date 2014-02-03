/*******************************************************************************
*
*	TITLE:		camera.h 
*
*	VERSION:	0.2 (Beta)                           
*
*	DATE:		16-Jan-2007
*
*	AUTHOR:		R. Kevin Watson
*				kevinw@jpl.nasa.gov
*
*	COMMENTS:	This is the "streamlined" version of camera.h
*
*				You are free to use this source code for any non-commercial
*				use. Please do not make copies of this source code, modified
*				or un-modified, publicly available on the internet or elsewhere
*				without permission. Thanks.
*
*				Copyright ©2005-2007 R. Kevin Watson. All rights are reserved.
*
********************************************************************************
*
*	CHANGE LOG:
*
*	DATE         REV  DESCRIPTION
*	-----------  ---  ----------------------------------------------------------
*	01-Jan-2006  0.1  RKW - Original code.
*	16-Jan-2007  0.2  RKW - Added Virtual_Window() function.
*
*******************************************************************************/
#ifndef _CAMERA_H
#define _CAMERA_H

//#define _DEBUG

// If your camera's serial port is attached to the the robot controller's
// programming serial port, uncomment the "#define CAMERA_SERIAL_PORT_1" 
// line. Otherwise, if your camera is attached to the TTL serial port, 
// uncomment the "#define CAMERA_SERIAL_PORT_2" line.
// #define CAMERA_SERIAL_PORT_1
#define CAMERA_SERIAL_PORT_2

// Default camera initialization parameters that will be 
// used if Get_Camera_Configuration() cannot find valid
// parameters in EEPROM. Commented values in brackets are
// the camera module's power-on default value in decimal
// and hexadecimal notation.
/*
//2007: first number = ligotti tuning; second = sisson tuning
#define R_MIN_DEFAULT	37	// Rmin for call to Track_Color()					101
#define R_MAX_DEFAULT	57	// Rmax for call to Track_Color()					121
#define G_MIN_DEFAULT	220	// Gmin for call to Track_Color()					15
#define G_MAX_DEFAULT	240	// Gmax for call to Track_Color()					26
#define B_MIN_DEFAULT	190 // Bmin for call to Track_Color()						114
#define B_MAX_DEFAULT	210 // Bmax for call to Track_Color()						134
#define NF_DEFAULT		0	// value for call to Noise_Filter()						1
#define AGC_DEFAULT		0	// Automatic Gain Control Register [0/0x00]				0
#define BLU_DEFAULT		128	// Blue Gain Control Register [128/0x80]			128
#define RED_DEFAULT		128	// Red Gain Control Register [128/0x80]				128
#define SAT_DEFAULT		128	// Saturation Control Register [128/0x80]			128
#define BRT_DEFAULT		1	// Brightness Control Register [128/0x80]				1
#define AEC_DEFAULT		23	// Automatic Exposure Control Register [127/0x7F]	1
#define COMA_DEFAULT	32	// Common Control A Register [36/0x24]				32
#define COMB_DEFAULT	32	// Common Control B Register [1/0x01]				32
#define COMI_DEFAULT	128	// Common Control I Register [0/0x00]				128
#define EHSH_DEFAULT	128	// Frame Rate Adjust Register 1 [0/0x00]			128
#define EHSL_DEFAULT	32	// Frame Rate Adjust Register 2 [0/0x00]			32
#define COMJ_DEFAULT	132	// Common Control J Register [129/0x81]				132
*/

//2006 BACKUP!!!
#define R_MIN_DEFAULT	93	// Rmin for call to Track_Color()
#define R_MAX_DEFAULT	113	// Rmax for call to Track_Color()
#define G_MIN_DEFAULT	15	// Gmin for call to Track_Color()
#define G_MAX_DEFAULT	26	// Gmax for call to Track_Color()
#define B_MIN_DEFAULT	111 // Bmin for call to Track_Color()
#define B_MAX_DEFAULT	131 // Bmax for call to Track_Color()
#define NF_DEFAULT		1	// value for call to Noise_Filter()
#define AGC_DEFAULT		0	// Automatic Gain Control Register [0/0x00]
#define BLU_DEFAULT		128	// Blue Gain Control Register [128/0x80]
#define RED_DEFAULT		128	// Red Gain Control Register [128/0x80]
#define SAT_DEFAULT		123	// Saturation Control Register [128/0x80]  118
#define BRT_DEFAULT		1	// Brightness Control Register [128/0x80]
#define AEC_DEFAULT		1	// Automatic Exposure Control Register [127/0x7F]
#define COMA_DEFAULT	32	// Common Control A Register [36/0x24]
#define COMB_DEFAULT	32	// Common Control B Register [1/0x01]
#define COMI_DEFAULT	128	// Common Control I Register [0/0x00]
#define EHSH_DEFAULT	128	// Frame Rate Adjust Register 1 [0/0x00]
#define EHSL_DEFAULT	32	// Frame Rate Adjust Register 2 [0/0x00]
#define COMJ_DEFAULT	132	// Common Control J Register [129/0x81]

// Number of cycles Initialize_Camera() will wait for a ACK/NCK before 
// timing out.
#define MAX_ACK_LOOP_COUNT 10

// To view debugging information on the terminal screen, uncomment the
// "#define _DEBUG" line below.
// #define _DEBUG

// DEBUG() is a macro that can be used just like printf() to send 
// debugging information to the screen. It has the added benefit that 
// it can be turned off and removed from the entire project by 
// commenting out the #define _DEBUG line above. Use it just like 
// printf(), just make sure you use double parentheses. As an example: 
// DEBUG(("Error: I'm about to cause a red-light-of-death\r\n")).
// The use of double parentheses is a common method employed to send a 
// variable number of arguments to a macro. 
#ifdef _DEBUG
#define DEBUG(x) printf x
#else
#define DEBUG(x)
#endif

#define ACENT(x) ((120 - (int)x)*25)/240

// setup camera-related macros
#ifdef CAMERA_SERIAL_PORT_1
#define TERMINAL_SERIAL_PORT_2
#else
#define TERMINAL_SERIAL_PORT_1
#endif

// camera state machine states
#define UNSYNCHRONIZED 1
#define DETERMINING_PACKET_TYPE 2
#define RECEIVING_T_PACKET 3
#define RECEIVING_T_PACKET_BITMAP 4
#define RECEIVING_ACK 5
#define RECEIVING_NCK 6

// initialize_Camera() states
#define STATE_ONE		 1
#define STATE_TWO		 2
#define STATE_THREE		 3
#define STATE_FOUR		 4
#define STATE_FIVE		 5
#define STATE_SIX		 6
#define STATE_SEVEN		 7
#define STATE_EIGHT		 8
#define STATE_NINE		 9
#define STATE_TEN		10
#define STATE_ELEVEN	11
#define STATE_TWELVE	12
#define STATE_THIRTEEN	13
#define STATE_FOURTEEN	14
#define STATE_FIFTEEN	15
#define STATE_SIXTEEN	16
#define STATE_SEVENTEEN	17
#define STATE_EIGHTEEN	18

// camera module register addresses
#define AGC_ADDRESS		0x00 	//  0 - Automatic Gain Control Register
#define BLU_ADDRESS		0x01 	//  1 - Blue Gain Control Register
#define RED_ADDRESS		0x02 	//  2 - Red Gain Control Register
#define SAT_ADDRESS		0x03 	//  3 - Saturation Control Register
#define BRT_ADDRESS		0x06 	//  6 - Brightness Control Register
#define AEC_ADDRESS		0x10 	// 16 - Automatic Exposure Control Register
#define COMA_ADDRESS	0x12 	// 18 - Common Control A Register
#define COMB_ADDRESS	0x13 	// 19 - Common Control B Register
#define COMI_ADDRESS	0x29 	// 41 - Common Control I Register
#define EHSH_ADDRESS	0x2A 	// 42 - Frame Rate Adjust Register 1
#define EHSL_ADDRESS	0x2B 	// 43 - Frame Rate Adjust Register 2
#define COMJ_ADDRESS	0x2D 	// 45 - Common Control J Register

// camera t packet data structure
typedef struct
{
	unsigned char mx;			// The middle of mass x value
	unsigned char my;			// The middle of mass y value
	unsigned char x1;			// The left most corner’s x value
	unsigned char y1;			// The left most corner’s y value
	unsigned char x2;			// The right most corner’s x value
	unsigned char y2;			// The right most corner’s y value
	unsigned char pixels;		// Number of pixels in the tracked region, scaled and capped at 255: (pixels+4)/8
	unsigned char confidence;	// The (# of pixels/area)*256 of the bounded rectangle and capped at 255
}	T_Packet_Data_Type;

typedef struct
{
	unsigned char num_of_lights;
	signed char c_light_angle;
	signed char r_light_angle;
	signed char l_light_angle;
}	Cam_Target_Data_Structure;

#define dual_target_threshold 100

typedef struct
{
	char armed;
	char sploded;
	char name;
} T_Bomb;

// global variables
extern unsigned int camera_t_packets;
extern T_Packet_Data_Type T_Packet_Data;
extern Cam_Target_Data_Structure Targets;

// function prototypes
void Camera_Handler(void);
void Camera_State_Machine(unsigned char);
unsigned char Initialize_Camera(void);
void Track_Color(unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char);
void Camera_Idle(void);
void Restart_Camera(void);
unsigned char Get_Camera_State(void);
void Raw_Mode(unsigned char);
void Noise_Filter(unsigned char);
void Virtual_Window(unsigned char, unsigned char, unsigned char, unsigned char);
void Write_Camera_Module_Register(unsigned char, unsigned char);
unsigned char Camera_Serial_Port_Byte_Count(void);
unsigned char Read_Camera_Serial_Port(void);
void Write_Camera_Serial_Port(unsigned char);
unsigned char Terminal_Serial_Port_Byte_Count(void);
unsigned char Read_Terminal_Serial_Port(void);
void Write_Terminal_Serial_Port(unsigned char);
void update_targets(void);
char cam_hasTarget(void);

#endif
