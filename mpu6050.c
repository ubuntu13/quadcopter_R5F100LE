#include "mpu6050.h"

uint8_t MPU6050_Data_Buf[14] =
{
	'G','Y'
};

uint8_t Offset_Buf[8] = 
{
	'O','F'
};

struct MPU MPU_Data;
struct int16_t_XYZ Accel_Offset, Gyro_Offset;

uint8_t MPU6050_Offset_OK = 1;

void MPU6050_Init(void)
{
	uint8_t temp;
	uint16_t i;
	
	MPU6050_Write_Byte(PWR_MGMT_1, 0x80);
	
	i = 10000;
	while(i--);
	i = 10000;
	while(i--);
	i = 10000;
	while(i--);
	
	MPU6050_Write_Byte(PWR_MGMT_1, 0x03);
	i = 100;
	while(i--);
	MPU6050_Write_Byte(PWR_MGMT_2, 0x00);
	i = 100;
	while(i--);
  	MPU6050_Write_Byte(SMPLRT_DIV, 0x04);
	i = 100;
	while(i--);
	MPU6050_Write_Byte(CONFIG, 0x04);
	i = 100;
	while(i--);
  	MPU6050_Write_Byte(GYRO_CONFIG, 0x18);
	i = 100;
	while(i--);
  	MPU6050_Write_Byte(ACCEL_CONFIG, 0x08);
	i = 100;
	while(i--);
	
	if(MPU6050_Offset_OK)
	{
		Accel_Offset.X = 9;
		Accel_Offset.Y = -259;
		Accel_Offset.Z = -969;
		
		Gyro_Offset.X = -33;
		Gyro_Offset.Y = 10;
		Gyro_Offset.Z = 14;
	}
}

void MPU6050_Write_Byte(uint8_t Reg_Adr, uint8_t Send_Data)
{
	uint8_t Buf[2];
	Buf[0] = Reg_Adr;
	Buf[1] = Send_Data;
	
	while( R_IICA0_Master_Send(MPU6050_SADR, Buf, 2, 200) != 0U );
	while( IICA0_Sendend == 0 );
	IICA0_Sendend = 0;
}

uint8_t MPU6050_Read_Byte(uint8_t Reg_Adr)
{
	uint8_t Recieve_Data = 0;
	
	while( R_IICA0_Master_Send(MPU6050_SADR, &Reg_Adr, 1, 200) != 0U);
	while( IICA0_Sendend == 0 );
	IICA0_Sendend = 0;
    
    while( R_IICA0_Master_Receive(MPU6050_SADR, &Recieve_Data, 1, 200) != 0U );
    while( IICA0_Receiveend == 0 );
	IICA0_Receiveend = 0;
	
	return Recieve_Data;
}

void MPU6050_Read_Double_6Bytes(uint8_t Start_Reg_Adr1, uint8_t Start_Reg_Adr2)
{
	while( R_IICA0_Master_Send(MPU6050_SADR, &Start_Reg_Adr1, 1, 200) != 0U);
	while( IICA0_Sendend == 0 );
	IICA0_Sendend = 0;
	
	while( R_IICA0_Master_Receive(MPU6050_SADR, MPU6050_Data_Buf + 2, 6, 200) != 0U );
    while( IICA0_Receiveend == 0 );
	IICA0_Receiveend = 0;
	
	while( R_IICA0_Master_Send(MPU6050_SADR, &Start_Reg_Adr2, 1, 200) != 0U);
	while( IICA0_Sendend == 0 );
	IICA0_Sendend = 0;
	
	while( R_IICA0_Master_Receive(MPU6050_SADR, MPU6050_Data_Buf + 8, 6, 200) != 0U );
    while( IICA0_Receiveend == 0 );
	IICA0_Receiveend = 0;
}

void MPU6050_Data_Update(void)
{
	static int32_t temp_AX=0, temp_AY=0, temp_AZ=0, temp_GX=0, temp_GY=0, temp_GZ=0;
	static uint8_t cnt=0;
	
	MPU6050_Read_Double_6Bytes(ACCEL_XOUT_H, GYRO_XOUT_H);
	
	MPU_Data.AX = ( ( MPU6050_Data_Buf[2]<<8 ) + MPU6050_Data_Buf[3] ) - Accel_Offset.X;
	MPU_Data.AY = ( ( MPU6050_Data_Buf[4]<<8 ) + MPU6050_Data_Buf[5] ) - Accel_Offset.Y;
	MPU_Data.AZ = ( ( MPU6050_Data_Buf[6]<<8 ) + MPU6050_Data_Buf[7] ) - Accel_Offset.Z;
	
	MPU_Data.GX = ( ( MPU6050_Data_Buf[8]<<8 ) + MPU6050_Data_Buf[9] ) - Gyro_Offset.X;
	MPU_Data.GY = ( ( MPU6050_Data_Buf[10]<<8 ) + MPU6050_Data_Buf[11] ) - Gyro_Offset.Y;
	MPU_Data.GZ = ( ( MPU6050_Data_Buf[12]<<8 ) + MPU6050_Data_Buf[13] ) - Gyro_Offset.Z;
	
	if(MPU6050_Offset_OK == 0)
	{
		temp_AX += MPU_Data.AX;
		temp_AY += MPU_Data.AY;
		temp_AZ += MPU_Data.AZ;
		temp_GX += MPU_Data.GX;
		temp_GY += MPU_Data.GY;
		temp_GZ += MPU_Data.GZ;
		
		cnt++;
		
		if(cnt==200)
		{
			Accel_Offset.X = (int16_t)(temp_AX / cnt) - 0;
			Accel_Offset.Y = (int16_t)(temp_AY / cnt) - 0;
			Accel_Offset.Z = (int16_t)(temp_AZ / cnt) - 8192;
			Gyro_Offset.X = (int16_t)(temp_GX / cnt) - 0;
			Gyro_Offset.Y = (int16_t)(temp_GY / cnt) - 0;
			Gyro_Offset.Z = (int16_t)(temp_GZ / cnt) - 0;
			
			Offset_Buf[2] = (uint8_t)( Accel_Offset.X >> 8 );
			Offset_Buf[3] = (uint8_t)Accel_Offset.X;
			Offset_Buf[4] = (uint8_t)( Accel_Offset.Y >> 8 );
			Offset_Buf[5] = (uint8_t)Accel_Offset.Y;
			Offset_Buf[6] = (uint8_t)( Accel_Offset.Z >> 8 );
			Offset_Buf[7] = (uint8_t)Accel_Offset.Z;
			
			R_UART0_Send(Offset_Buf, 8);
			
			MPU6050_Offset_OK = 1;
		}
	}
}











