#include "com.h"

extern float altitude;
extern float altitude_rate;
extern float testoutput;
extern float loiter_rate;
extern float change_x;

uint8_t Message_Buf[24] = 
{
	'A','T',
	'0','0','0','0',
	'0','0','0','0',
	'0','0','0','0',
	'0','0','0','0',
	'0','0','0','0',
	'0','0'
};

void Send_Messge(void)
{	
	float AR, AP, AY, EP, ER;

	AR = Attitude.Roll * 10;
	AP = Attitude.Pitch * 10;
	AY = camera.position_x * 10;
    //ER = testoutput * 100;
	//EP = camera.erzhi * 100;
	ER = camera.camera_x * 100;
	EP = camera.sine * 100;

	Message_Buf[2] = (uint8_t)( (int32_t)AR>>24 );
	Message_Buf[3] = (uint8_t)( (int32_t)AR>>16 );
	Message_Buf[4] = (uint8_t)( (int32_t)AR>>8 );
	Message_Buf[5] = (uint8_t)( AR );
	
	Message_Buf[6] = (uint8_t)( (int32_t)AP>>24 );
	Message_Buf[7] = (uint8_t)( (int32_t)AP>>16 );
	Message_Buf[8] = (uint8_t)( (int32_t)AP>>8 );
	Message_Buf[9] = (uint8_t)( AP );
	
	Message_Buf[10] = (uint8_t)( (int32_t)AY>>24 );
	Message_Buf[11] = (uint8_t)( (int32_t)AY>>16 );
	Message_Buf[12] = (uint8_t)( (int32_t)AY>>8 );
	Message_Buf[13] = (uint8_t)( AY );
	
	Message_Buf[14] = (uint8_t)( (int32_t)EP>>24 );
	Message_Buf[15] = (uint8_t)( (int32_t)EP>>16 );
	Message_Buf[16] = (uint8_t)( (int32_t)EP>>8 );
	Message_Buf[17] = (uint8_t)( EP );
	
	Message_Buf[18] = (uint8_t)( (int32_t)ER>>24 );
	Message_Buf[19] = (uint8_t)( (int32_t)ER>>16 );
	Message_Buf[20] = (uint8_t)( (int32_t)ER>>8 );
	Message_Buf[21] = (uint8_t)( ER );
	
	Message_Buf[22] = (uint8_t)((int32_t)altitude>>8);
	Message_Buf[23] = (uint8_t)((int32_t)altitude>>0);
	
	R_UART0_Send(Message_Buf, 24);
}