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
* File Name    : r_cg_it_user.c
* Version      : CodeGenerator for RL78/G13 V2.00.00.07 [22 Feb 2013]
* Device(s)    : R5F100LE
* Tool-Chain   : CA78K0R
* Description  : This file implements device driver for IT module.
* Creation Date: 2015/8/14
***********************************************************************************************************************/

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
#pragma interrupt INTIT r_it_interrupt
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "r_cg_macrodriver.h"
#include "r_cg_it.h"
/* Start user code for include. Do not edit comment generated here */
#include "controller.h"
#include "camera.h"
/* End user code. Do not edit comment generated here */
#include "r_cg_userdefine.h"

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
uint8_t Timer6_En = 0;
uint8_t Timer5_En = 0;
uint8_t Timer4_En = 1;
uint8_t Timer2_En = 0;

uint8_t time_imu = 0;
uint8_t time_control = 0;
uint8_t time_trig = 0;
uint8_t time_pos = 0;

uint16_t Int_Timer_Cnt = 0;
uint16_t Int_Task_Cnt = 0;
uint16_t Int_Fly_Cnt = 0;

uint8_t flying_task = 0;
uint8_t landing_task = 0;
uint8_t main_task = 1;

extern int16_t M_Thr;
extern float altitude;
extern uint8_t landing_finished;
extern uint8_t PosHold_Enabled;

uint8_t flying_finished;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: r_it_interrupt
* Description  : This function is INTIT interrupt service routine.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
__interrupt static void r_it_interrupt(void)
{
    /* Start user code. Do not edit comment generated here */
	if( Timer5_En ) //flymode0
	{
		Int_Fly_Cnt++;
		
		switch(main_task)
		{
			case 1:
				if(Int_Fly_Cnt >= 2000)
				{
					flying_task = 1;
					//PosHold_Enabled = 1;
					pidRoll.desired = -1.0;
					if(flying_finished)
					{
						Int_Fly_Cnt = 0;
						main_task = 2;
						pidRoll.desired = -1.5;
					}
				}
				
				break;
			case 2:
				if(Int_Fly_Cnt >= 5000)
				{
					if(camera.sine == 0)
					{
						Int_Fly_Cnt = 0;
						main_task = 3;
						pidRoll.desired = 3.0;//shache
					}
				}
				
				break;
			case 3:
				if(Int_Fly_Cnt >= 800)
				{
					pidRoll.desired = 0;
					pidPitch.desired = 0;
					landing_task = 1;
					Timer5_En = 0;
					PosHold_Enabled = 0;
				}
				
				break;
		}
	}
	
	if( Timer4_En )
	{
		if(flying_task)
		{
			Int_Task_Cnt++;
			
			if(Int_Task_Cnt == 2)
			{
				pidAlt.desired = 3;
				M_Thr = 1400; // %40 power
			}
			if(Int_Task_Cnt == 1500)
			{
				pidAlt.desired = 60;
			}
			
			if(altitude > 5000)
			{
				flying_finished = 1;
				flying_task = 0;
				PosHold_Enabled = 1;
			}
		}
		
		if(landing_task)
		{
			pidAlt.desired = 0;
			
			if(altitude <= 900)
			{
				landing_finished = 1;
				landing_task = 0;
			}
		}
	}
	
	if( Timer2_En )
	{
		Int_Timer_Cnt ++;
		
		if(Int_Timer_Cnt %4 == 0)
		{
			time_imu = 1;
		}
		
		if(Int_Timer_Cnt %10 == 0)
		{
			time_control = 1;
		}
			
		if(Int_Timer_Cnt %25 == 0)
		{
			time_trig = 1;
		}
		
		if(Int_Timer_Cnt %200 == 0)
		{
			time_pos = 1;
		}
		
		if(Int_Timer_Cnt >= 1000)
		{
			Int_Timer_Cnt = 0;
		}
	}
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
