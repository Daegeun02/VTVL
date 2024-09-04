/*
 * task.c
 *
 *  Created on: Jun 30, 2024
 *      Author: daegeun
 */

#include "task.h"


extern TIM_Handler tim1;
extern TIM_Handler tim3;
extern TIM_Handler tim8;

extern UART_Handler uart2;

extern st_Actuator   Actuator;
extern st_Rocket     Rocket;
extern st_Controller Controller;
extern st_Guidance   Guidance;
extern st_Taranis    Taranis;
extern st_Mapping    Mapping;

extern uint8_t Gauged;


// AuTo Pilot Count
uint32_t ATP_COUNT = 0;
uint16_t PWM_COUNT = 0;
uint32_t PLS_COUNT = 0;
uint32_t HLS_COUNT = 0;

uint8_t CLIMB = 1;



__inline void DO_TASK_00( void )
{
	return;
}


/* Manual Flight Mode */
__inline void DO_TASK_11( void )
{
	register uint32_t PWM = 0;

	double pitch = (double)((&Taranis)->CH[2]) * 0.05 * D2R;
	double yaw   = (double)((&Taranis)->CH[3]) * 0.05 * D2R;

	/* Handle Engine Thrust Start */
	// Read CUTOFF Signal
	(&Controller)->CUTOFF = (bool)( (&Taranis)->CH[6] - 100 );

	if ( !(&Controller)->CUTOFF ) PWM = (uint32_t)( 1500 + (&Taranis)->CH[1] * 5 );
	else                          PWM = 1000;

	(&Controller)->PWM = PWM;
	/* Handle Engine Thrust End */

	/* Handle pitch Command Start */
	PWM = (uint32_t)(pitch * R2D * 200. + 1500.);

	if ( !(PWM < 2500) ) PWM = 2499;
	if ( !(PWM >  500) ) PWM =  501;

	(&Controller)->TVC_1 = (uint16_t)( PWM & 0x0000FFFF );
	/* Handle pitch Command End */

	/* Handle yaw Command Start */
	PWM = (uint32_t)(yaw   * R2D * 200. + 1500.);

	if ( !(PWM < 2500) ) PWM = 2499;
	if ( !(PWM >  500) ) PWM =  501;

	(&Controller)->TVC_2 = (uint16_t)( PWM & 0x0000FFFF );
	/* Handle yaw Command End */

	TIM_Change_CCR( &tim1, CHANNEL_1, (&Controller)->TVC_1 );
	TIM_Change_CCR( &tim1, CHANNEL_2, (&Controller)->TVC_2 );

	TIM_Change_CCR( &tim8, CHANNEL_4, (&Controller)->PWM );
}


__inline uint8_t DO_TASK_12( void )
{
	uint8_t READY = 0;

	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 4] + 100 ) * 0.005 ) *  4;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 5] + 100 ) * 0.005 ) *  5;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 6] + 100 ) * 0.005 ) *  6;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 7] + 100 ) * 0.005 ) * 99;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[10] + 100 ) * 0.005 ) * 10;

	return READY;
}


/* Manual Engine Command */
__inline uint8_t DO_TASK_30( void )
{
	register uint8_t  READY = 0;
	register uint32_t PWM   = 0;

	// Read CUTOFF Signal
	(&Controller)->CUTOFF = (bool)( (&Taranis)->CH[6] + 100 );

	if ( !(&Controller)->CUTOFF ) PWM = (uint32_t)( 1500 + (&Taranis)->CH[1] * 5 );
	else                          PWM = 1000;

	(&Controller)->PWM = PWM;

	TIM_Change_CCR( &tim8, CHANNEL_4, ( PWM & 0x0000FFFF ) );

	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 4] + 100 ) * 0.005 ) *  4;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 5] + 100 ) * 0.005 ) *  5;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 6] + 100 ) * 0.005 ) *  6;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[ 7] + 100 ) * 0.005 ) * 99;
	READY += (uint8_t)( ( (double)(&Taranis)->CH[10] + 100 ) * 0.005 ) * 10;

	return READY;
}


/* Step Thrust Test */
__inline void DO_TASK_31( void )
{
	register uint16_t PWM   = (&Controller)->PWM;

	// Read CUTOFF Signal
	(&Controller)->CUTOFF = (bool)( (&Taranis)->CH[6] + 100 );

	if ( ( (&Taranis)->CH[7] == -100 ) && ( (&Taranis)->CH[10] == -100 ) )
	{
		CLIMB = 1;
		PWM   = 1200;
	}
	else
	{
		if ( !(&Controller)->CUTOFF )
		{
			if ( ( PWM_COUNT % 400 ) == 0 )
			{
				if ( CLIMB )
				{
					if      ( (&Taranis)->CH[ 7] == 100 ) PWM += 100;
					else if ( (&Taranis)->CH[10] == 100 ) PWM +=  50;
				}
				else
				{
					if      ( (&Taranis)->CH[ 7] == 100 ) PWM -= 100;
					else if ( (&Taranis)->CH[10] == 100 ) PWM -=  50;
				}

				if      ( PWM == 2000 ) CLIMB = 0;
				else if ( PWM == 1200 ) CLIMB = 1;

				PWM_COUNT = 0;
			}
		}
		else
		{
			PWM = 1000;
		}

		PWM_COUNT++;
	}

	(&Controller)->PWM = PWM;

	TIM_Change_CCR( &tim8, CHANNEL_4, ( PWM & 0x0000FFFF ) );
}


__inline void DO_TASK_32( void )
{
	double pitch = (double)((&Taranis)->CH[2]) * 0.05 * D2R;
	double yaw   = (double)((&Taranis)->CH[3]) * 0.05 * D2R;

	register uint32_t tvc_x = (uint32_t)(pitch * R2D * 200. + 1500.);
	register uint32_t tvc_y = (uint32_t)(yaw   * R2D * 200. + 1500.);

	if ( !(tvc_x < 2500) ) tvc_x = 2499;
	if ( !(tvc_x >  500) ) tvc_x =  501;

	if ( !(tvc_y < 2500) ) tvc_y = 2499;
	if ( !(tvc_y >  500) ) tvc_y =  501;

	(&Controller)->TVC_1 = tvc_x;
	(&Controller)->TVC_2 = tvc_y;

	TIM_Change_CCR( &tim1, CHANNEL_1, ( tvc_x & 0x0000FFFF ) );
	TIM_Change_CCR( &tim1, CHANNEL_2, ( tvc_y & 0x0000FFFF ) );
}


__inline void DO_TASK_33( void )
{
	register uint32_t PWM;

	(&Controller)->CUTOFF = (bool)( (&Taranis)->CH[6] + 100 );

	if ( !(&Controller)->CUTOFF )
	{
		PWM = (uint32_t)( 1500 + (&Taranis)->CH[1] * 5 );

		TIM_Change_CCR( &tim3, CHANNEL_1, ( PWM ) );
	}
	else
	{
		TIM_Change_CCR( &tim3, CHANNEL_1, ( 1000 ) );
	}
}


__inline void DO_TASK_4X( void )
{
	if ( ( ATP_COUNT % 20 ) == 0 )
	{
		Do_Guidance( &Guidance, &Rocket );
	}

	if ( ( ATP_COUNT % 2 ) == 0 )
	{
		Do_Control( &Controller, &Rocket, (&Guidance)->Command );
	}

	*(int*)(&tim3)->CCR1 = (&Controller)->TVC_1;

	TIM_Change_CCR( &tim1, CHANNEL_1, (&Controller)->TVC_1 );
	TIM_Change_CCR( &tim1, CHANNEL_2, (&Controller)->TVC_2 );

	TIM_Change_CCR( &tim8, CHANNEL_4, (&Controller)->PWM );

	ATP_COUNT++;
}


__inline void DO_TASK_50( void )
{
	if ( ( PLS_COUNT % 20 ) == 0 )
	{
		Do_Guidance( &Guidance, &Rocket );
	}

	if ( ( PLS_COUNT % 2 ) == 0 )
	{
		Do_Control( &Controller, &Rocket, (&Guidance)->Command );

		Transmit_V_TO_P( &uart2 );
	}

	PLS_COUNT++;
}


__inline void DO_TASK_51( void )
{
	if ( ( HLS_COUNT % 20 ) == 0 )
	{
		Do_Guidance( &Guidance, &Rocket );
	}

	if ( ( HLS_COUNT % 2 ) == 0 )
	{
		Do_Control( &Controller, &Rocket, (&Guidance)->Command );

		TIM_Change_CCR( &tim1, CHANNEL_1, (&Controller)->TVC_1 );
		TIM_Change_CCR( &tim1, CHANNEL_2, (&Controller)->TVC_2 );
	}

	HLS_COUNT++;
}


__inline void DO_TASK_52( void )
{
	return;
}


__inline void DO_TASK_53( void )
{
	int8_t READY = 0;
	bool   Gauging = ( (&Taranis)->CH[11] + 100 );

	/* TRIMMING */
	if      ( (&Taranis)->CH[ 6] == -100 ) (&Mapping)->MODE = MOVING;
	else if ( (&Taranis)->CH[ 6] ==    0 ) (&Mapping)->MODE = SERVO1;
	else if ( (&Taranis)->CH[ 6] ==  100 ) (&Mapping)->MODE = SERVO2;

	switch ( (&Mapping)->MODE ) {

	case MOVING:
		break;

	case SERVO1:
		if ( !(&Mapping)->LOCK )
		{
			if ( Gauging )
			{
				READY += (int8_t)( (double)( (&Taranis)->CH[4] + 100 ) * 0.01 ) * 4;
				READY += (int8_t)( (double)( (&Taranis)->CH[5] + 100 ) * 0.01 ) * 5;

				switch ( READY ) {

				case  4: if ( Gauging ) (&Mapping)->trim1 += 10; (&Mapping)->LOCK = 1; break;
				case  8: if ( Gauging ) (&Mapping)->trim1 -= 10; (&Mapping)->LOCK = 1; break;
				case  5: if ( Gauging ) (&Mapping)->trim1 +=  3; (&Mapping)->LOCK = 1; break;
				case 10: if ( Gauging ) (&Mapping)->trim1 -=  3; (&Mapping)->LOCK = 1; break;
				}
			}

			(&Mapping)->Gain1 = (float)(&Taranis)->CH[8] * 0.02 + 3.0;
		}
		else
		{
			if ( !Gauging )
			{
				(&Mapping)->LOCK = 0;
			}
		}
		break;

	case SERVO2:
		if ( !(&Mapping)->LOCK )
		{
			if ( Gauging )
			{
				READY += (int8_t)( (double)( (&Taranis)->CH[4] + 100 ) * 0.01 ) * 4;
				READY += (int8_t)( (double)( (&Taranis)->CH[5] + 100 ) * 0.01 ) * 5;

				switch ( READY ) {

				case  4: if ( Gauging ) (&Mapping)->trim2 += 10; (&Mapping)->LOCK = 1; break;
				case  8: if ( Gauging ) (&Mapping)->trim2 -= 10; (&Mapping)->LOCK = 1; break;
				case  5: if ( Gauging ) (&Mapping)->trim2 +=  3; (&Mapping)->LOCK = 1; break;
				case 10: if ( Gauging ) (&Mapping)->trim2 -=  3; (&Mapping)->LOCK = 1; break;
				}
			}

			(&Mapping)->Gain2 = (float)(&Taranis)->CH[9] * 0.02 + 3.0;
		}
		else
		{
			if ( !Gauging )
			{
				(&Mapping)->LOCK = 0;
			}
		}
		break;
	}

	(&Mapping)->TVC1 = (uint32_t)( ( (&Taranis)->CH[2] ) * (&Mapping)->Gain1 + (&Mapping)->trim1 );
	(&Mapping)->TVC2 = (uint32_t)( ( (&Taranis)->CH[3] ) * (&Mapping)->Gain2 + (&Mapping)->trim2 );

	if ( (&Mapping)->TVC1 > 2000 ) (&Mapping)->TVC1 = 2000;
	if ( (&Mapping)->TVC1 <  800 ) (&Mapping)->TVC1 =  800;
	if ( (&Mapping)->TVC2 > 2000 ) (&Mapping)->TVC2 = 2000;
	if ( (&Mapping)->TVC2 <  800 ) (&Mapping)->TVC2 =  800;

	TIM_Change_CCR( &tim1, CHANNEL_1, (&Mapping)->TVC1 );
	TIM_Change_CCR( &tim1, CHANNEL_2, (&Mapping)->TVC2 );
}
