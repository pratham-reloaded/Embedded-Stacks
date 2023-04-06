/*
 * DC_MOTOR_cfg.c
 *
 *  Created on: 10-Sep-2022
 *      Author: shashank
 */

#include "mdds30.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[]=
{
		//Steering Configurations PA6 TIM 3 CHANNEL 1

		{
				GPIOB,
				GPIO_PIN_4,
				TIM4,
				TIM_CHANNEL_1,
				36,
				DC_MOTOR_F_PWM,
				DC_MOTOR_PWM_RES

		},
		{
						GPIOB,
						GPIO_PIN_5,
						TIM4,
						TIM_CHANNEL_2,
						36,
						DC_MOTOR_F_PWM,
						DC_MOTOR_PWM_RES

				}

};

