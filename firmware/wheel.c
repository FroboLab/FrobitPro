/****************************************************************************
# Frobit FroboMind Controller interface
# Copyright (c) 2012-2014, Kjeld Jensen <kjeld@frobomind.org>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*****************************************************************************
# File: wheel.c
# Project: Frobit RoboCard interface
# Platform: FroboMind Controller Feb. 2014 http://www.frobomind.org
# Microcontroller: AT90CAN128
# Author: Kjeld Jensen <kjeld@frobomind.org>
# Created:  2012-08-15 Kjeld Jensen
# Modified: 2013-02-04 Kjeld Jensen, Migrated to the BSD license
# Modified: 2014-04-17 Kjeld Jensen, FrobitV2 updates
# Modified: 2014-08-16 Kjeld Jensen, Ported to FroboMind Controller
****************************************************************************/
/* includes */
#include <avr/interrupt.h>
#include "wheel.h"
#include "fmctrl_def.h"
#include "pid_ctrl_int.h"

/***************************************************************************/
/* defines */
#define STATE_NO_ERR		2 /* maximum system state value before shutting down motors */

/* Motor defines (left) */
#define ML_PWM_MAX			511 /* 9 bit PWM */
#define ML_ENABLE_DDR		DDRA
#define ML_ENABLE_PORT		PORTA
#define ML_ENABLE_PIN		PA0
#define ML_ENABLE			PB_HIGH(ML_ENABLE_PORT, ML_ENABLE_PIN)
#define ML_DISABLE			PB_LOW(ML_ENABLE_PORT, ML_ENABLE_PIN)

#define ML_PWM_A			OCR1A
#define ML_PWM_A_ON			TCCR1A |= (1<<COM1A1)
#define ML_PWM_A_OFF		TCCR1A &= ~(1<<COM1A1)
#define ML_PWM_A_DDR		DDRB
#define ML_PWM_A_PIN		PB5

#define ML_PWM_B			OCR1B
#define ML_PWM_B_ON			TCCR1A |= (1<<COM1B1)
#define ML_PWM_B_OFF		TCCR1A &= ~(1<<COM1B1)
#define ML_PWM_B_DDR		DDRB
#define ML_PWM_B_PIN		PB6

/* Motor defines (right) */
#define MR_PWM_MAX			511 /* 9 bit PWM */
#define MR_ENABLE_DDR		DDRA
#define MR_ENABLE_PORT		PORTA
#define MR_ENABLE_PIN		PA2
#define MR_ENABLE			PB_HIGH(MR_ENABLE_PORT, MR_ENABLE_PIN)
#define MR_DISABLE			PB_LOW(MR_ENABLE_PORT, MR_ENABLE_PIN)

#define MR_PWM_A			OCR3A
#define MR_PWM_A_ON			TCCR3A |= (1<<COM3A1)
#define MR_PWM_A_OFF		TCCR3A &= ~(1<<COM3A1)
#define MR_PWM_A_DDR		DDRE
#define MR_PWM_A_PIN		PE3

#define MR_PWM_B			OCR3B
#define MR_PWM_B_ON			TCCR3A |= (1<<COM3B1)
#define MR_PWM_B_OFF		TCCR3A &= ~(1<<COM3B1)
#define MR_PWM_B_DDR		DDRE
#define MR_PWM_B_PIN		PE4

/* Encoder defines (left) */
#define ENCL_A_PIN			PIND0		/* int0 */
#define ENCL_A_PORT			PIND
#define ENCL_B_PIN			PINA1
#define ENCL_B_PORT			PINA

/* Encoder defines (right) */
#define ENCR_A_PIN			PIND1		/* int1 */
#define ENCR_A_PORT			PIND
#define ENCR_B_PIN			PINA3
#define ENCR_B_PORT			PINA

#define TICKS_BUF_LEN		10

/* PID control */

/* Low level motor control */
#define PROP_STEP		1 /* max change per. motor_update() call */

/***************************************************************************/
/* global and static variables */
extern char state;

/* actuator variables */
unsigned char set_cmd;
short vel_set_l, vel_set_r; /* [ticks/s] */
long prop_l, prop_r;
short pwm_l, pwm_r;

/* PID variables */
char pid_enable;
short pid_rate; /* [Hz] */
short pid_interval; /* 1-1000 [ms] */
pid_int_t pid_l, pid_r;
static long feed_forward;

/* status variables */
volatile long ticks_l, ticks_r;
static long pid_ticks_l, pid_ticks_r;
static short ticks_l_buf[TICKS_BUF_LEN];
static short ticks_r_buf[TICKS_BUF_LEN];

/***************************************************************************/
void encoder_init(void)
{
	/* initialize interrupts for external quadrature encoders */
	EIMSK|= BV(INT0)|BV(INT1); /* enable INT0 (PD0) & INT1 (PD1) */
	EICRA |= BV(ISC00)|BV(ISC10); /* INT0 INT1 both rising and falling edges */

	PB_PULL_UP (ENCL_A_PORT, ENCL_A_PIN); /* pull-up required according to datasheet */
	PB_PULL_UP (ENCL_B_PORT, ENCL_B_PIN);
	PB_PULL_UP (ENCR_A_PORT, ENCR_A_PIN);
	PB_PULL_UP (ENCR_B_PORT, ENCR_B_PIN);

	/* reset variables */
	ticks_l = 0;
	pid_ticks_l = 0;
	ticks_r = 0;
	pid_ticks_r = 0;
}
/***************************************************************************/
ISR (SIG_INTERRUPT0)
{
	if (PB_IS_HIGH(ENCL_A_PORT, ENCL_A_PIN)) /* if ch A is high */
	{
		if (PB_IS_HIGH(ENCL_B_PORT, ENCL_B_PIN)) /* if ch B is high */
			ticks_l++;
		else /* if ch B is low */
			ticks_l--;
	}
	else /* if ch A is low */
	{
		if (PB_IS_HIGH(ENCL_B_PORT, ENCL_B_PIN)) /* if ch B is high */
			ticks_l--;
		else /* if ch B is low */
			ticks_l++;
	}
}
/***************************************************************************/
ISR (SIG_INTERRUPT1)
{
	if (PB_IS_HIGH(ENCR_A_PORT, ENCR_A_PIN)) /* if ch A is high */
	{
		if (PB_IS_HIGH(ENCR_B_PORT, ENCR_B_PIN)) /* if ch B is high */
			ticks_r--;
		else /* if ch B is low */
			ticks_r++;
	}
	else /* if ch A is low */
	{
		if (PB_IS_HIGH(ENCR_B_PORT, ENCR_B_PIN)) /* if ch B is high */
			ticks_r++;
		else /* if ch B is low */
			ticks_r--;
	}
}
/***************************************************************************/
void motor_set_param(long dT, long Kp, long Ki, long Kd, long i_max, long feed_fwd)
{
	feed_forward = feed_fwd;

	pid_l.dT = dT;
	pid_l.Kp = Kp;
	pid_l.Ki = Ki;
	pid_l.Kd = Kd;
	pid_l.integral_max = i_max;
	pid_l.integral_factor = 10000;
	pid_l.derivative_factor = 100;
	pid_int_init (&pid_l); /* initialize PID controller (left) */

	pid_r.dT = dT;
	pid_r.Kp = Kp;
	pid_r.Ki = Ki;
	pid_r.Kd = Kd;
	pid_r.integral_max = i_max;
	pid_r.integral_factor = 10000;
	pid_r.derivative_factor = 100;
	pid_int_init (&pid_r); /* initialize PID controller (right) */
}
/***************************************************************************/
void motor_init(void)
{
	/* left motor */
	PB_OUT (ML_ENABLE_DDR, ML_ENABLE_PIN); /* set pin connected to Simple-H EA (Enable) as output */
	PB_OUT (ML_PWM_A_DDR, ML_PWM_A_PIN); /* set pin connected to Simple-H PA (PWM) as output */
	PB_OUT (ML_PWM_B_DDR, ML_PWM_B_PIN); /* set pin connected to Simple-H PB (PWM) as output */
	TCCR1A = (1<<WGM11)|(1<<WGM12); /* fast pwm 9 bit */
	TCCR1B = (1<<CS10); /* clk_io no prescaling */
	pwm_l = 0;

	/* right motor */
	PB_OUT (MR_ENABLE_DDR, MR_ENABLE_PIN); /* set pin connected to Simple-H EA (Enable) as output */
	PB_OUT (MR_PWM_A_DDR, MR_PWM_A_PIN); /* set pin connected to Simple-H PA (PWM) as output */
	PB_OUT (MR_PWM_B_DDR, MR_PWM_B_PIN); /* set pin connected to Simple-H PB (PWM) as output */
	TCCR3A = (1<<WGM31)|(1<<WGM32); /* fast pwm 9 bit */
	TCCR3B = (1<<CS30); /* clk_io no prescaling */
	pwm_r = 0;
}
/***************************************************************************/
void wheel_init (void)
{
	pid_enable = TRUE; /* default enable PID */

	encoder_init(); /* initialize quadrature encoders */
	motor_init(); /* initialize PWM */
}
/***************************************************************************/
static void motor_update (void)
{
	if (state <= STATE_NO_ERR)
	{
		if (pwm_l < 0)
		{
			ML_PWM_A_OFF;
			ML_PWM_B = (-pwm_l); /* 9 bit PWM */
			ML_PWM_B_ON;
			ML_ENABLE;
		}
		else if (pwm_l > 0)
		{
			ML_PWM_B_OFF;
			ML_PWM_A = (pwm_l); /* 9 bit PWM */
			ML_PWM_A_ON;
			ML_ENABLE;
		}
		else
		{
			ML_PWM_A_OFF;
			ML_PWM_B_OFF;
			ML_DISABLE;
		}

		if (pwm_r < 0)
		{
			MR_PWM_A_OFF;
			MR_PWM_B = (-pwm_r); /* 9 bit PWM */
			MR_PWM_B_ON;
			MR_ENABLE;
		}
		else if (pwm_r > 0)
		{
			MR_PWM_B_OFF;
			MR_PWM_A = (pwm_r); /* 9 bit PWM */
			MR_PWM_A_ON;
			MR_ENABLE;
		}
		else
		{
			MR_PWM_A_OFF;
			MR_PWM_B_OFF;
			MR_DISABLE;
		}
	}
	else
	{
		ML_PWM_A_OFF;
		ML_PWM_B_OFF;
		ML_DISABLE;
		pwm_l = 0;

		MR_PWM_A_OFF;
		MR_PWM_B_OFF;
		MR_DISABLE;
		pwm_r = 0;
	}
}
/***************************************************************************/
void wheel_update_ticks_buffers (void) /* asways 50 hz */
{
	short i;

	for (i=TICKS_BUF_LEN-1; i>0; i--)
	{
		ticks_l_buf[i] = ticks_l_buf[i-1];
		ticks_r_buf[i] = ticks_r_buf[i-1];
	}

	ticks_l_buf[0] = ticks_l - pid_ticks_l;
	pid_ticks_l = ticks_l;

	ticks_r_buf[0] = ticks_r - pid_ticks_r;
	pid_ticks_r = ticks_r;		
}
/***************************************************************************/
short wheel_calc_vel (short *buf)
{
	short sum, ticks;

	sum = buf[0]+buf[1]+ buf[2];
	if (sum > 20 || sum < -20)
	{
		ticks = (15*buf[0]+10*buf[1]+5*buf[2]);
	}
	else
	{
		sum += (buf[3]+buf[4]);
		if (sum > 20 || sum < -20)
		{
			ticks = (10*buf[0]+8*buf[1]+6*buf[2]+4*buf[3]+2*buf[4]);
		}
		else
		{
			ticks = (7*buf[0]+6*buf[1]+5*buf[2]+4*buf[3]+3*buf[4]+2*buf[5]+1*buf[6]+1*buf[7]+1*buf[8]);
		}
	}
	return ticks;
}
/***************************************************************************/
void wheel_update_pid (void)
{
	if (vel_set_l != 0 && state <= STATE_NO_ERR) /* if the velocity is set to zero */
	{
		pid_l.setpoint = vel_set_l*30/50;
		pid_l.measured = wheel_calc_vel(ticks_l_buf); 
		pid_int_update (&pid_l); 

		prop_l += pid_l.output/50;

		if (vel_set_l > 0) 
			pwm_l = feed_forward + prop_l;
		else
			pwm_l = -feed_forward + prop_l;
		
		if (pwm_l > ML_PWM_MAX)
			pwm_l = ML_PWM_MAX;
		else if (pwm_l <-ML_PWM_MAX)
			pwm_l = -ML_PWM_MAX;
	}
	else
	{
		prop_l = 0;
		pwm_l = 0;
		pid_int_init (&pid_l); 		
	}


	if (vel_set_r != 0 && state <= STATE_NO_ERR) /* if the velocity is set to zero */
	{
		pid_r.setpoint = vel_set_r*30/50;
		pid_r.measured = wheel_calc_vel(ticks_r_buf); 
		pid_int_update (&pid_r); 

		prop_r += pid_r.output/50;

		if (vel_set_r > 0) 
			pwm_r = feed_forward + prop_r;
		else
			pwm_r = -feed_forward + prop_r;
		
		if (pwm_r > MR_PWM_MAX)
			pwm_r = MR_PWM_MAX;
		else if (pwm_r <-MR_PWM_MAX)
			pwm_r = -MR_PWM_MAX;
	}
	else
	{
		prop_r = 0;
		pwm_r = 0;
		pid_int_init (&pid_r); 		
	}

	motor_update(); 
}
/***************************************************************************/
void wheel_update_open_loop (void)
{
	pwm_l = vel_set_l; 
	pwm_r = vel_set_r;
	motor_update();
}
/***************************************************************************/
