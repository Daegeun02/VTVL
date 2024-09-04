/*
 * uart4_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include <uart4_packet.h>


#define CHAR( X ) (char)X[0]

#define MSG_SIZE ( 27 )


extern st_Actuator   Actuator;
extern st_Rocket     Rocket;
extern st_Controller Controller;

extern st_GPS GPS;
extern st_FSM FSM;

uint8_t txh0_V_TO_G = 0x44;
uint8_t txh1_V_TO_G = 0x47;

uint8_t rxh0_G_TO_V = 0x47;
uint8_t rxh1_G_TO_V = 0x44;

uint8_t hdrf_G_TO_V = 0;
uint8_t idxn_G_TO_V = 0;
uint8_t _sum_G_TO_V = 0;
uint8_t Tsum_G_TO_V = 0;
uint8_t Rsum_G_TO_V = 0;
uint8_t eCnt_G_TO_V = 0;

uint8_t rxbf_G_TO_V[2];

uint8_t ready_G_TO_V;

uint8_t txData_V_TO_G[bfsz_V_TO_G];
uint8_t rxData_G_TO_V[bfsz_G_TO_V];


char TASK_11[] = "GROUP1, MANUAL FLIGHT"   ; uint8_t _len_11 = sizeof( TASK_11 );
char TASK_12[] = "GROUP1, AUTO PILOT"      ; uint8_t _len_12 = sizeof( TASK_12 );
char TASK_19[] = "GROUP1, SUSPENDING"      ; uint8_t _len_19 = sizeof( TASK_19 );

char TASK_20[] = "GROUP2, GUIDANCE GAIN"   ; uint8_t _len_20 = sizeof( TASK_20 );
char TASK_21[] = "GROUP2, CONTROL GAIN"    ; uint8_t _len_21 = sizeof( TASK_21 );
char TASK_29[] = "GROUP2, SUSPENDING"      ; uint8_t _len_29 = sizeof( TASK_29 );

char TASK_30[] = "GROUP3, MANUAL COMMAND"  ; uint8_t _len_30 = sizeof( TASK_30 );
char TASK_31[] = "GROUP3, STEP THRUST TEST"; uint8_t _len_31 = sizeof( TASK_31 );
char TASK_32[] = "GROUP3, TVC SERVO MANUAL"; uint8_t _len_32 = sizeof( TASK_32 );
char TASK_33[] = "GROUP3, RCS BLDC MANUAL" ; uint8_t _len_33 = sizeof( TASK_33 );
char TASK_39[] = "GROUP3, SUSPENDING"      ; uint8_t _len_39 = sizeof( TASK_39 );

char TASK_40[] = "GROUP4, TAKE-OFF"        ; uint8_t _len_40 = sizeof( TASK_40 );
char TASK_41[] = "GROUP4, LANDING"         ; uint8_t _len_41 = sizeof( TASK_41 );
char TASK_42[] = "GROUP4, HOVERING"        ; uint8_t _len_42 = sizeof( TASK_42 );
char TASK_43[] = "GROUP4, HORIZONTAL MOVE" ; uint8_t _len_43 = sizeof( TASK_43 );
char TASK_44[] = "GROUP4, VERTICAL MOVE"   ; uint8_t _len_44 = sizeof( TASK_44 );
char TASK_49[] = "GROUP4, SUSPENDING"      ; uint8_t _len_49 = sizeof( TASK_49 );

char TASK_50[] = "GROUP5, PILS"            ; uint8_t _len_50 = sizeof( TASK_50 );
char TASK_51[] = "GROUP5, HILS"            ; uint8_t _len_51 = sizeof( TASK_51 );
char TASK_52[] = "GROUP5, TVC Mapping"     ; uint8_t _len_52 = sizeof( TASK_52 );
char TASK_53[] = "GROUP5, TVC Trimming"    ; uint8_t _len_53 = sizeof( TASK_53 );
char TASK_59[] = "GROUP5, SUSPENDING"      ; uint8_t _len_59 = sizeof( TASK_59 );

char TASK_NN[] = "GROUP NONE, SUSPEND"     ; uint8_t _len_NN = sizeof( TASK_NN );
char TASK_NA[] = "N / A"                   ; uint8_t _len_NA = sizeof( TASK_NA );



void Transmit_V_TO_G( UART_Handler *uart, uint32_t GLOBALTIME )
{
	uint8_t checksum = 0x00;
	char*   TASK;
	uint8_t _len;
	double  temp;

	switch ( FSM.TASK ) {

	case FSM_TASK_MANUAL        : TASK = TASK_11; _len = _len_11; break;
	case FSM_TASK_AUTOPILOT     : TASK = TASK_12; _len = _len_12; break;
	case FSM_TASK_GROUP1_SUSPEND: TASK = TASK_19; _len = _len_19; break;

	case FSM_TASK_GuidanceGain  : TASK = TASK_20; _len = _len_20; break;
	case FSM_TASK_ControlGain   : TASK = TASK_21; _len = _len_21; break;
	case FSM_TASK_GROUP2_SUSPEND: TASK = TASK_29; _len = _len_29; break;

	case FSM_TASK_MANUALCOMMAND   : TASK = TASK_30; _len = _len_30; break;
	case FSM_TASK_STEP_THRUST_TEST: TASK = TASK_31; _len = _len_31; break;
	case FSM_TASK_TVC_SERVO_MANUAL: TASK = TASK_32; _len = _len_32; break;
	case FSM_TASK_RCS_BLDC_MANUAL : TASK = TASK_33; _len = _len_33; break;
	case FSM_TASK_GROUP3_SUSPEND  : TASK = TASK_39; _len = _len_39; break;

	case FSM_TASK_TAKEOFF       : TASK = TASK_40; _len = _len_40; break;
	case FSM_TASK_LANDING       : TASK = TASK_41; _len = _len_41; break;
	case FSM_TASK_HOVER         : TASK = TASK_42; _len = _len_42; break;
	case FSM_TASK_HORIZON       : TASK = TASK_43; _len = _len_43; break;
	case FSM_TASK_VERTICAL      : TASK = TASK_44; _len = _len_44; break;
	case FSM_TASK_GROUP4_SUSPEND: TASK = TASK_49; _len = _len_49; break;

	case FSM_TASK_PILS          : TASK = TASK_50; _len = _len_50; break;
	case FSM_TASK_HILS          : TASK = TASK_51; _len = _len_51; break;
	case FSM_TASK_TVC_Mapping   : TASK = TASK_52; _len = _len_52; break;
	case FSM_TASK_TVC_TRIMMING  : TASK = TASK_53; _len = _len_53; break;
	case FSM_TASK_GROUP5_SUSPEND: TASK = TASK_59; _len = _len_59; break;

	case FSM_TASK_SUSPEND: TASK = TASK_NN; _len = _len_NN; break;
	default              : TASK = TASK_NA; _len = _len_NA; break;
	}

	txData_V_TO_G[0] = txh0_V_TO_G;
	txData_V_TO_G[1] = txh1_V_TO_G;

	// FSM
	for ( int i = 0; i < _len; i++ )
	{
		txData_V_TO_G[i+2] = TASK[i];
	}
	for ( int i = _len; i < MSG_SIZE; i++ )
	{
		txData_V_TO_G[i+2] = CHAR(" ");
	}

	txData_V_TO_G[29] = FSM.TASK_LOCKER;

	// VELOCITY
	Compression_2_U08( &Rocket.VEL[0], &txData_V_TO_G[30], _CONVERT_I16_2_U08, VEL_RES_TX );
	Compression_2_U08( &Rocket.VEL[1], &txData_V_TO_G[32], _CONVERT_I16_2_U08, VEL_RES_TX );
	Compression_2_U08( &Rocket.VEL[2], &txData_V_TO_G[34], _CONVERT_I16_2_U08, VEL_RES_TX );

	// ATTITUDE
	Compression_2_U08( &Rocket.ATT[0], &txData_V_TO_G[36], _CONVERT_I16_2_U08, ATT_RES_TX );
	Compression_2_U08( &Rocket.ATT[1], &txData_V_TO_G[38], _CONVERT_I16_2_U08, ATT_RES_TX );
	Compression_2_U08( &Rocket.ATT[2], &txData_V_TO_G[40], _CONVERT_I16_2_U08, ATT_RES_TX );

	// LATITUDE
	temp = Rocket.LLA[0] * R2D; Compression_2_U08( &temp, &txData_V_TO_G[42], _CONVERT_I32_2_U08, LAT_RES_TX );
	// LONGITUDE
	temp = Rocket.LLA[1] * R2D; Compression_2_U08( &temp, &txData_V_TO_G[46], _CONVERT_I32_2_U08, LON_RES_TX );
	// ALTITUDE
	Compression_2_U08( &Rocket.LLA[2], &txData_V_TO_G[50], _CONVERT_I16_2_U08, ALT_RES_TX );

	// GPS
	Compression_2_U08( &GPS.SPD, &txData_V_TO_G[52], _CONVERT_I16_2_U08, VEL_RES_TX );

	txData_V_TO_G[54] = (uint8_t)GPS.SatNum;

	// TIME
	UNION_U08_U32.value = GLOBALTIME;

	txData_V_TO_G[55] = UNION_U08_U32.input[0];
	txData_V_TO_G[56] = UNION_U08_U32.input[1];
	txData_V_TO_G[57] = UNION_U08_U32.input[2];
	txData_V_TO_G[58] = UNION_U08_U32.input[3];

	// Controller Response
	txData_V_TO_G[59] = (uint8_t)( Taranis.CH[ 0] + 100 );
	txData_V_TO_G[60] = (uint8_t)( Taranis.CH[ 1] + 100 );
	txData_V_TO_G[61] = (uint8_t)( Taranis.CH[ 2] + 100 );
	txData_V_TO_G[62] = (uint8_t)( Taranis.CH[ 3] + 100 );
	txData_V_TO_G[63] = (uint8_t)( Taranis.CH[ 4] + 100 );
	txData_V_TO_G[64] = (uint8_t)( Taranis.CH[ 5] + 100 );
	txData_V_TO_G[65] = (uint8_t)( Taranis.CH[ 6] + 100 );
	txData_V_TO_G[66] = (uint8_t)( Taranis.CH[ 7] + 100 );
	txData_V_TO_G[67] = (uint8_t)( Taranis.CH[ 8] + 100 );
	txData_V_TO_G[68] = (uint8_t)( Taranis.CH[ 9] + 100 );
	txData_V_TO_G[69] = (uint8_t)( Taranis.CH[10] + 100 );
	txData_V_TO_G[70] = (uint8_t)( Taranis.CH[11] + 100 );

	// Engine
	temp = (double)Actuator.RPM * 0.1;
	Compression_2_U08( &temp, &txData_V_TO_G[71], _CONVERT_U16_2_U08, 1. );

	txData_V_TO_G[73] = Actuator.ThrustLevel;
	txData_V_TO_G[74] = (uint8_t)( Actuator.Vb * 21.2500000000000 );
	txData_V_TO_G[75] = (uint8_t)( Actuator.Vr * 21.2500000000000 );

	temp = (double)Actuator.Power * 0.01;
	Compression_2_U08( &Actuator.Power      , &txData_V_TO_G[76], _CONVERT_U16_2_U08, 1. );
	Compression_2_U08( &Actuator.Temperature, &txData_V_TO_G[78], _CONVERT_U16_2_U08, 65.5350000000000 );

	txData_V_TO_G[80] = (uint8_t)( Actuator.EngineStatus[ 0] );
	txData_V_TO_G[81] = (uint8_t)( Actuator.EngineStatus[ 1] );
	txData_V_TO_G[82] = (uint8_t)( Actuator.EngineStatus[ 2] );
	txData_V_TO_G[83] = (uint8_t)( Actuator.EngineStatus[ 3] );
	txData_V_TO_G[84] = (uint8_t)( Actuator.EngineStatus[ 4] );
	txData_V_TO_G[85] = (uint8_t)( Actuator.EngineStatus[ 5] );
	txData_V_TO_G[86] = (uint8_t)( Actuator.EngineStatus[ 6] );
	txData_V_TO_G[87] = (uint8_t)( Actuator.EngineStatus[ 7] );
	txData_V_TO_G[88] = (uint8_t)( Actuator.EngineStatus[ 8] );
	txData_V_TO_G[89] = (uint8_t)( Actuator.EngineStatus[ 9] );
	txData_V_TO_G[90] = (uint8_t)( Actuator.EngineStatus[10] );
	txData_V_TO_G[91] = (uint8_t)( Actuator.EngineStatus[11] );
	txData_V_TO_G[92] = (uint8_t)( Actuator.EngineStatus[12] );
	txData_V_TO_G[93] = (uint8_t)( Actuator.EngineStatus[13] );

	Compress_U16_2_U08( Controller.PWM, &txData_V_TO_G[94] );

	txData_V_TO_G[96] = Controller.CUTOFF;

	Compression_2_U08( &Controller.Thrust, &txData_V_TO_G[97], _CONVERT_I16_2_U08, 10. );

	Compress_U16_2_U08( ( Controller.TVC_1 ), &txData_V_TO_G[ 99] );
	Compress_U16_2_U08( ( Controller.TVC_2 ), &txData_V_TO_G[101] );

	Compress_U16_2_U08( ( Controller.RCS1 ), &txData_V_TO_G[103] );
	Compress_U16_2_U08( ( Controller.RCS2 ), &txData_V_TO_G[105] );
	Compress_U16_2_U08( ( Controller.RCS3 ), &txData_V_TO_G[107] );
	Compress_U16_2_U08( ( Controller.RCS4 ), &txData_V_TO_G[109] );

	// CHECKSUM
	for ( int i = 0; i < bfsz_V_TO_G-1; i++ )
	{
		checksum ^= txData_V_TO_G[i];
	}

	txData_V_TO_G[bfsz_V_TO_G-1] = checksum;

	UART_Transmit( uart, txData_V_TO_G, bfsz_V_TO_G );
}


__inline void Read_G_TO_V( UART_Handler *uart )
{
	if ( UART_Receive_1B( uart, &rxbf_G_TO_V[0] ) )
	{
		if ( hdrf_G_TO_V == 1 )
		{
			rxData_G_TO_V[idxn_G_TO_V] = rxbf_G_TO_V[0];

			_sum_G_TO_V ^= rxbf_G_TO_V[0];
			idxn_G_TO_V++;

			if ( idxn_G_TO_V == bfsz_G_TO_V - 1 )
			{
				hdrf_G_TO_V++;
			}
		}

		else if ( hdrf_G_TO_V == 0 )
		{
			if ( rxbf_G_TO_V[0] == rxh0_G_TO_V )
			{
				hdrf_G_TO_V++;
				idxn_G_TO_V++;
			}
			else
			{
				hdrf_G_TO_V = 0;
				idxn_G_TO_V = 0;
			}
		}

		else if ( hdrf_G_TO_V == 1 )
		{
			if ( rxbf_G_TO_V[0] == rxh1_G_TO_V )
			{
				hdrf_G_TO_V++;
				idxn_G_TO_V++;
			}
			else
			{
				hdrf_G_TO_V = 0;
				idxn_G_TO_V = 0;
			}
		}

		else if ( hdrf_G_TO_V == 3 )
		{
			Tsum_G_TO_V = _sum_G_TO_V;
			Rsum_G_TO_V = rxbf_G_TO_V[0];

			if ( Tsum_G_TO_V == Rsum_G_TO_V )
			{
				hdrf_G_TO_V = 0;
				idxn_G_TO_V = 0;
				_sum_G_TO_V = 0;

				ready_G_TO_V = 1;
			}
			else
			{
				hdrf_G_TO_V = 0;
				idxn_G_TO_V = 0;
				_sum_G_TO_V = 0;

				eCnt_G_TO_V++;
			}
		}

		else
		{
			hdrf_G_TO_V = 0;
			idxn_G_TO_V = 0;
			_sum_G_TO_V = 0;

			ready_G_TO_V = 0;
		}
	}
}
