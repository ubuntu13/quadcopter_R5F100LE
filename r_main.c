/***********************************************************************************************************************
* DISCLAIMER
* This software is supplied by Renesas Electronics Corporation and is only 
* intended for use with Renesas products. No other uses are authorized. This 
* software is owned by Renesas Electronics Corporation and is protected under 
* all applicable laws, including copyright laws.
* THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING 
* THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT 
* LIMITED TO WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE 
* AND NON-INFRINGEMENT.  ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED.
* TO THE MAXIMUM EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS 
* ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES SHALL BE LIABLE 
* FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR 
* ANY REASON RELATED TO THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE 
* BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
* Renesas reserves the right, without notice, to make changes to this software 
* and to discontinue the availability of this software.  By using this software, 
* you agree to the additional terms and conditions found by accessing the 
* following link:
* http://www.renesas.com/disclaimer
*
* Copyright (C) 2011, 2013 Renesas Electronics Corporation. All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* File Name    : r_main.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements main function.
* Creation Date: 2015/8/14
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_cgc.h"
#include "r_cg_port.h"
#include "r_cg_serial.h"
#include "r_cg_timer.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include "mpu6050.h"
#include "pid.h"
#include "imu.h"
#include "controller.h"
#include "senor.h"
#include "stabilizer.h"
#include "camera.h"
#include "key.h"
#include "oled.h"
#include "show.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
extern uint8_t UART0_Recieve_Buf[10];
extern uint8_t UART2_Recieve_Buf[64];
extern uint8_t UART0_Receiveend;
extern uint8_t UART2_RX_STA;
extern uint8_t time_imu;
extern uint8_t time_control;
extern uint8_t time_trig;
extern uint8_t time_pos;
extern uint8_t Timer2_En;
extern int16_t M_Thr;
extern struct F_XYZ Fil_Accel, Fil_Gyro;
extern uint8_t HIGH_CRL;
extern float altitude;
extern uint8_t flying_task;
extern uint8_t landing_task;
extern uint8_t Timer5_En, Timer6_En;
extern int16_t M_ALt;

uint8_t landing_finished = 0;

uint8_t PosHold_Enabled = 0;
uint8_t Show_Enabled = 1;

uint8_t tx_buf[10] = "dzb";
uint8_t key_temp = 0;
uint8_t flymode = 0;
float temp;

float testoutput = 0;

/* End user code. Do not edit comment generated here */
void R_MAIN_UserInit(void);

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void main(void)
{
    R_MAIN_UserInit();
    /* Start user code. Do not edit comment generated here */
	
    while (1U)
    {
		key_temp = key_scan();
		if(key_temp == 1)
		{
			oled_showstring(0 , 4, "unlock!");
			oled_showmode(flymode);
			
			Timer2_En = 1;
			R_TAU0_Channel6_Start();
			
			break;
		}
		
        if(UART0_Receiveend)
		{
			UART0_Receiveend = 0;
			
			if( UART0_Recieve_Buf[0] == 'G' && UART0_Recieve_Buf[1] == 'O' )
			{
				Timer2_En = 1;
				
				R_TAU0_Channel6_Start();
				
				break;
			}
		}
    }

	while (1U)
	{
		if(time_imu)
		{
			time_imu = 0;
			
			MPU6050_Data_Update();
			IMU_Prepare();
			IMU_Update(Fil_Gyro.X, Fil_Gyro.Y, Fil_Gyro.Z, Fil_Accel.X, Fil_Accel.Y, Fil_Accel.Z);
			
			imu_buffer();
			
			stabilize_task();
		}
		
		if(time_control)
		{
			time_control = 0;
			
			M_ALt = controllerAltHoldPID(altitude);
		}
		
		if(time_trig)
		{
			time_trig = 0;
			
			senor_trig();	//senor trig to get altitude
			
			if(Show_Enabled)
			{
				oled_showmessage((uint8_t)Attitude.Roll, (uint8_t)Attitude.Pitch, (uint8_t)(altitude/100));
			}
		}
		
		if(time_pos)
		{
			time_pos = 0;
			
			if(PosHold_Enabled)
			{
				testoutput = controllerPosPitHoldPID(camera.position_x);
				pidPitch.desired = testoutput;
			}
		}
		
		key_temp = key_scan();
		if(key_temp)
		{
			if(key_temp == 1)
			{
				if(flymode)
				{
					flymode = 0;
				}
				else
				{
					flymode = 1;
				}
				oled_showmode(flymode);
			}
			else if(key_temp == 2)
			{
				Show_Enabled = 0;
				if(flymode == 0)
				{
					Timer5_En = 1;
				}
				else
				{
					Timer6_En = 1;
				}
			}
		}
		
		if(UART0_Receiveend || landing_finished)
		{ 
			UART0_Receiveend = 0;
			
			if( (UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'T') || (landing_finished) )
			{
				MOTOR1 = 1000;
				MOTOR2 = 1000;
				MOTOR3 = 1000;
				MOTOR4 = 1000;
					
				M_Thr = 1000;
				landing_finished = 0;
					
				while (1U)
				{
					if(UART0_Receiveend)
					{
						UART0_Receiveend = 0;
							
						if( UART0_Recieve_Buf[0] == 'G' && UART0_Recieve_Buf[1] == 'O' )
						{
							break;
						}
						
						if(UART0_Recieve_Buf[0] == 'E' && UART0_Recieve_Buf[1] == 'H')
						{
							temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
							
							pidAlt.desired = temp;
						}
					}
				}
			}
			
			if( UART0_Recieve_Buf[0] == 'P' && UART0_Recieve_Buf[1] == 'W' )
			{
				temp = ( UART0_Recieve_Buf[2]<<8 | UART0_Recieve_Buf[3] );
				M_Thr = (MOTOR_Update( (int16_t)temp) );
			}
			
			if( UART0_Recieve_Buf[0] == 'P' && UART0_Recieve_Buf[1] == 'S' )
			{
				PosHold_Enabled = 1;
			}
			
			if( UART0_Recieve_Buf[0] == 'P' && UART0_Recieve_Buf[1] == 'T' )
			{
				PosHold_Enabled = 0;
			}
			
			if(UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'H')
			{
				HIGH_CRL = 0;
			}
			
			if(UART0_Recieve_Buf[0] == 'F' && UART0_Recieve_Buf[1] == 'L')
			{
				flying_task = 1;
			}
			
			if(UART0_Recieve_Buf[0] == 'L' && UART0_Recieve_Buf[1] == 'A')
			{
				landing_task = 1;
			}
			
			if(UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'P')
			{
				temp = (float)(UART0_Recieve_Buf[2]<<8 | UART0_Recieve_Buf[3]);
				
				pidPosRol.kp = temp * 0.001;
			}
			
			if(UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'I')
			{
				temp = (UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
				
				pidPosRol.ki = temp * 0.001;
			}
										
			if(UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'D')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);

				pidPosRol.kd = temp * 0.001;
			}
			
			if(UART0_Recieve_Buf[0] == 'S' && UART0_Recieve_Buf[1] == 'X')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);

				pidPosRol.kp = temp * 0.001;
			}
			
			if(UART0_Recieve_Buf[0] == 'E' && UART0_Recieve_Buf[1] == 'P')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
							
				pidPitch.desired = temp * 0.001;
				
				if(UART0_Recieve_Buf[4] == '-')
				{
					pidPitch.desired = 0 - pidPitch.desired;
				}
			}
			
			if(UART0_Recieve_Buf[0] == 'E' && UART0_Recieve_Buf[1] == 'R')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
							
				pidRoll.desired = temp * 0.001;
				
				if(UART0_Recieve_Buf[4] == '-')
				{
					pidRoll.desired = 0 - pidRoll.desired;
				}
			}
			
			if(UART0_Recieve_Buf[0] == 'E' && UART0_Recieve_Buf[1] == 'W')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
							
				pidYaw.desired = temp * 0.001;
				
				if(UART0_Recieve_Buf[4] == '-')
				{
					pidYaw.desired = 0 - pidYaw.desired;
				}
			}
			
			if(UART0_Recieve_Buf[0] == 'E' && UART0_Recieve_Buf[1] == 'H')
			{
				temp = ( UART0_Recieve_Buf[2]<<8)|(UART0_Recieve_Buf[3]);
				pidAlt.desired = temp;
			}
		}
	}
    /* End user code. Do not edit comment generated here */
}

/***********************************************************************************************************************
* Function Name: R_MAIN_UserInit
* Description  : This function adds user code before implementing main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void R_MAIN_UserInit(void)
{
    /* Start user code. Do not edit comment generated here */
	uint16_t i = 10000;
	uint8_t j = 10;
	
    EI();
	
	while(i--)
	{
		while(j--);
	}
	
	i = 10000;
	j = 10;
	
	P3.0 = 0;
	
	R_TAU0_Channel0_Start();
	R_TAU0_Channel5_Start();
	
	R_IT_Start();
	
	R_UART0_Start();
	R_UART0_Send(tx_buf, 4);
	R_UART0_Receive( UART0_Recieve_Buf, 6);
	
	R_UART2_Start();
	R_UART2_Receive( UART0_Recieve_Buf, 6);
	
	controller_init();
	
	Filter_Init();
	
	MPU6050_Init();
	
	oled_init();
	main_interface();
	
	while(i--)
	{
	   while(j--);
	}
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
