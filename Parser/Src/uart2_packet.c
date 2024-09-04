/*
 * uart6_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include <uart2_packet.h>



uint8_t pIMU[256] = {0};
uint8_t pINS[256] = {0};
uint8_t pGPS[256] = {0};

uint8_t ready_I_TO_V = 0;

uint8_t dtyp_I_TO_V = 0;
uint8_t hdrf_I_TO_V = 0;
uint8_t plen_I_TO_V = 0;

uint8_t checkMSB = 0;
uint8_t checkLSB = 0;

uint8_t rxBuff_I_TO_V[ 2 ] = {0};
uint8_t rxData_I_TO_V[256] = {0};

st_INS INS;
st_IMU IMU;
st_GPS GPS;

uint32_t errCOUNT = 0;

uint8_t txData_V_TO_P[bfsz_V_TO_P];

uint8_t txh0_V_TO_P = 0x44;
uint8_t txh1_V_TO_P = 0x47;


uint32_t CNT_COUNT = 0;

extern st_Rocket Rocket;

extern st_Controller Controller;


__inline void Read_I_TO_V( UART_Handler *uart )
{
	if ( UART_Receive_1B( uart, &rxBuff_I_TO_V[0] ) )
	{
		if ( hdrf_I_TO_V >= 4 )
		{
			if      ( dtyp_I_TO_V == 1 ) pIMU[hdrf_I_TO_V] = rxBuff_I_TO_V[0];
			else if ( dtyp_I_TO_V == 2 ) pINS[hdrf_I_TO_V] = rxBuff_I_TO_V[0];
			else if ( dtyp_I_TO_V == 3 ) pGPS[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

			rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

			hdrf_I_TO_V++;
			plen_I_TO_V--;

			if ( plen_I_TO_V >= 2 )
			{
				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}

			else if ( plen_I_TO_V == 0 )
			{
				if ( ( checkMSB == rxData_I_TO_V[hdrf_I_TO_V-2] ) && ( checkLSB == rxData_I_TO_V[hdrf_I_TO_V-1] ) )
				{
					ready_I_TO_V = 1;

					hdrf_I_TO_V = 0;

					checkMSB = 0;
					checkLSB = 0;
				}
			}
		}

		else if ( hdrf_I_TO_V == 3 )
		{
			if      ( dtyp_I_TO_V == 1 ) pIMU[hdrf_I_TO_V] = rxBuff_I_TO_V[0];
			else if ( dtyp_I_TO_V == 2 ) pINS[hdrf_I_TO_V] = rxBuff_I_TO_V[0];
			else if ( dtyp_I_TO_V == 3 ) pGPS[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

			rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

			plen_I_TO_V = rxBuff_I_TO_V[0] + 2;

			hdrf_I_TO_V++;

			checkMSB += rxBuff_I_TO_V[0];
			checkLSB += checkMSB;
		}

		else if ( hdrf_I_TO_V == 0 )
		{
			// SYNC1
			if ( rxBuff_I_TO_V[0] == 0x75 )
			{
				rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

				hdrf_I_TO_V++;

				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}
			else
			{
				hdrf_I_TO_V = 0;

				checkMSB = 0;
				checkLSB = 0;
			}
		}

		else if ( hdrf_I_TO_V == 1 )
		{
			// SYNC2
			if ( rxBuff_I_TO_V[0] == 0x65 )
			{
				rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

				hdrf_I_TO_V++;

				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}
			else
			{
				hdrf_I_TO_V = 0;

				checkMSB = 0;
				checkLSB = 0;
			}
		}

		else if ( hdrf_I_TO_V == 2 )
		{
			// Description -> IMU
			if ( rxBuff_I_TO_V[0] == 0x80 )
			{
				rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

				dtyp_I_TO_V = 1;

				hdrf_I_TO_V++;

				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}
			// Description -> INS
			else if ( rxBuff_I_TO_V[0] == 0x82 )
			{
				rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

				dtyp_I_TO_V = 2;

				hdrf_I_TO_V++;

				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}
			// Description -> GPS
			else if ( rxBuff_I_TO_V[0] == 0x91 )
			{
				rxData_I_TO_V[hdrf_I_TO_V] = rxBuff_I_TO_V[0];

				dtyp_I_TO_V = 3;

				hdrf_I_TO_V++;

				checkMSB += rxBuff_I_TO_V[0];
				checkLSB += checkMSB;
			}
			else
			{
				hdrf_I_TO_V = 0;
				dtyp_I_TO_V = 0;

				checkMSB = 0;
				checkLSB = 0;
			}
		}

		else
		{
			hdrf_I_TO_V = 0;
			dtyp_I_TO_V = 0;

			checkMSB = 0;
			checkLSB = 0;
		}
	}
}


__inline void parse_IMU( uint8_t *rxData_I_TO_V )
{
	if ( rxData_I_TO_V[3] == IMU_PAYLOAD )
	{
		if ( rxData_I_TO_V[19] == IMU_ACC )
		{
			UNION_U08_F32.input[0] = rxData_I_TO_V[23]; UNION_U08_F32.input[1] = rxData_I_TO_V[22]; UNION_U08_F32.input[2] = rxData_I_TO_V[21]; UNION_U08_F32.input[3] = rxData_I_TO_V[20];
			IMU.ACC[0]     = (double)UNION_U08_F32.value * 9.81;

			UNION_U08_F32.input[0] = rxData_I_TO_V[27]; UNION_U08_F32.input[1] = rxData_I_TO_V[26]; UNION_U08_F32.input[2] = rxData_I_TO_V[25]; UNION_U08_F32.input[3] = rxData_I_TO_V[24];
			IMU.ACC[1]     = (double)UNION_U08_F32.value * 9.81;

			UNION_U08_F32.input[0] = rxData_I_TO_V[31]; UNION_U08_F32.input[1] = rxData_I_TO_V[30]; UNION_U08_F32.input[2] = rxData_I_TO_V[29]; UNION_U08_F32.input[3] = rxData_I_TO_V[28];
			IMU.ACC[2]     = (double)UNION_U08_F32.value * 9.81;
		}

		if ( rxData_I_TO_V[33] == IMU_ANV )
		{
			UNION_U08_F32.input[0] = rxData_I_TO_V[37]; UNION_U08_F32.input[1] = rxData_I_TO_V[36]; UNION_U08_F32.input[2] = rxData_I_TO_V[35]; UNION_U08_F32.input[3] = rxData_I_TO_V[34];
			IMU.ANV[0]     = (double)UNION_U08_F32.value * R2D;

			UNION_U08_F32.input[0] = rxData_I_TO_V[41]; UNION_U08_F32.input[1] = rxData_I_TO_V[40]; UNION_U08_F32.input[2] = rxData_I_TO_V[39]; UNION_U08_F32.input[3] = rxData_I_TO_V[38];
			IMU.ANV[1]     = (double)UNION_U08_F32.value * R2D;

			UNION_U08_F32.input[0] = rxData_I_TO_V[45]; UNION_U08_F32.input[1] = rxData_I_TO_V[44]; UNION_U08_F32.input[2] = rxData_I_TO_V[43]; UNION_U08_F32.input[3] = rxData_I_TO_V[42];
			IMU.ANV[2]     = (double)UNION_U08_F32.value * R2D;
		}

		if ( rxData_I_TO_V[47] == IMU_MAG )
		{
			UNION_U08_F32.input[0] = rxData_I_TO_V[51]; UNION_U08_F32.input[1] = rxData_I_TO_V[50]; UNION_U08_F32.input[2] = rxData_I_TO_V[49]; UNION_U08_F32.input[3] = rxData_I_TO_V[48];
			IMU.MAG[0]     = (double)UNION_U08_F32.value;

			UNION_U08_F32.input[0] = rxData_I_TO_V[55]; UNION_U08_F32.input[1] = rxData_I_TO_V[54]; UNION_U08_F32.input[2] = rxData_I_TO_V[53]; UNION_U08_F32.input[3] = rxData_I_TO_V[52];
			IMU.MAG[1]     = (double)UNION_U08_F32.value;

			UNION_U08_F32.input[0] = rxData_I_TO_V[59]; UNION_U08_F32.input[1] = rxData_I_TO_V[58]; UNION_U08_F32.input[2] = rxData_I_TO_V[57]; UNION_U08_F32.input[3] = rxData_I_TO_V[56];
			IMU.MAG[2]     = (double)UNION_U08_F32.value;
		}

		if ( rxData_I_TO_V[61] == IMU_ATT )
		{
			UNION_U08_F32.input[0] = rxData_I_TO_V[65]; UNION_U08_F32.input[1] = rxData_I_TO_V[64]; UNION_U08_F32.input[2] = rxData_I_TO_V[63]; UNION_U08_F32.input[3] = rxData_I_TO_V[62];
			IMU.ATT[0]     = (double)UNION_U08_F32.value * R2D;

			UNION_U08_F32.input[0] = rxData_I_TO_V[69]; UNION_U08_F32.input[1] = rxData_I_TO_V[68]; UNION_U08_F32.input[2] = rxData_I_TO_V[67]; UNION_U08_F32.input[3] = rxData_I_TO_V[66];
			IMU.ATT[1]     = (double)UNION_U08_F32.value * R2D;

			UNION_U08_F32.input[0] = rxData_I_TO_V[73]; UNION_U08_F32.input[1] = rxData_I_TO_V[72]; UNION_U08_F32.input[2] = rxData_I_TO_V[71]; UNION_U08_F32.input[3] = rxData_I_TO_V[70];
			IMU.ATT[2]     = (double)UNION_U08_F32.value * R2D;
		}

		IMU.COUNT++;
	}
}


__inline void parse_INS( uint8_t *rxData_I_TO_V )
{
	if ( rxData_I_TO_V[3] == INS_PAYLOAD )
	{
		if ( rxData_I_TO_V[19] == INS_LLH )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[44];
			UNION_U08_U16.input[1] = rxData_I_TO_V[45];

			if ( UNION_U08_U16.value )
			{
				UNION_U08_F64.input[0] = rxData_I_TO_V[27]; UNION_U08_F64.input[1] = rxData_I_TO_V[26]; UNION_U08_F64.input[2] = rxData_I_TO_V[25]; UNION_U08_F64.input[3] = rxData_I_TO_V[24];
				UNION_U08_F64.input[4] = rxData_I_TO_V[23]; UNION_U08_F64.input[5] = rxData_I_TO_V[22]; UNION_U08_F64.input[6] = rxData_I_TO_V[21]; UNION_U08_F64.input[7] = rxData_I_TO_V[20];
				Rocket.LLA[0]   = UNION_U08_F64.value * D2R;

				UNION_U08_F64.input[0] = rxData_I_TO_V[35]; UNION_U08_F64.input[1] = rxData_I_TO_V[34]; UNION_U08_F64.input[2] = rxData_I_TO_V[33]; UNION_U08_F64.input[3] = rxData_I_TO_V[32];
				UNION_U08_F64.input[4] = rxData_I_TO_V[31]; UNION_U08_F64.input[5] = rxData_I_TO_V[30]; UNION_U08_F64.input[6] = rxData_I_TO_V[29]; UNION_U08_F64.input[7] = rxData_I_TO_V[28];
				Rocket.LLA[1]   = UNION_U08_F64.value * D2R;

				UNION_U08_F64.input[0] = rxData_I_TO_V[43]; UNION_U08_F64.input[1] = rxData_I_TO_V[42]; UNION_U08_F64.input[2] = rxData_I_TO_V[41]; UNION_U08_F64.input[3] = rxData_I_TO_V[40];
				UNION_U08_F64.input[4] = rxData_I_TO_V[39]; UNION_U08_F64.input[5] = rxData_I_TO_V[38]; UNION_U08_F64.input[6] = rxData_I_TO_V[37]; UNION_U08_F64.input[7] = rxData_I_TO_V[36];
				Rocket.LLA[2]   = UNION_U08_F64.value;
			}
		}

		if ( rxData_I_TO_V[47] == INS_VEL )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[60];
			UNION_U08_U16.input[1] = rxData_I_TO_V[61];

			if ( UNION_U08_U16.value )
			{
				UNION_U08_F32.input[0] = rxData_I_TO_V[51]; UNION_U08_F32.input[1] = rxData_I_TO_V[50]; UNION_U08_F32.input[2] = rxData_I_TO_V[49]; UNION_U08_F32.input[3] = rxData_I_TO_V[48];
				Rocket.VEL[0]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[55]; UNION_U08_F32.input[1] = rxData_I_TO_V[54]; UNION_U08_F32.input[2] = rxData_I_TO_V[53]; UNION_U08_F32.input[3] = rxData_I_TO_V[52];
				Rocket.VEL[1]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[59]; UNION_U08_F32.input[1] = rxData_I_TO_V[58]; UNION_U08_F32.input[2] = rxData_I_TO_V[57]; UNION_U08_F32.input[3] = rxData_I_TO_V[56];
				Rocket.VEL[2]  = (double)UNION_U08_F32.value;
			}
		}

		if ( rxData_I_TO_V[63] == INS_ATT )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[76];
			UNION_U08_U16.input[1] = rxData_I_TO_V[77];

			if ( UNION_U08_U16.value )
			{
				UNION_U08_F32.input[0] = rxData_I_TO_V[67]; UNION_U08_F32.input[1] = rxData_I_TO_V[66]; UNION_U08_F32.input[2] = rxData_I_TO_V[65]; UNION_U08_F32.input[3] = rxData_I_TO_V[64];
				Rocket.ATT[0]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[71]; UNION_U08_F32.input[1] = rxData_I_TO_V[70]; UNION_U08_F32.input[2] = rxData_I_TO_V[69]; UNION_U08_F32.input[3] = rxData_I_TO_V[68];
				Rocket.ATT[1]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[75]; UNION_U08_F32.input[1] = rxData_I_TO_V[74]; UNION_U08_F32.input[2] = rxData_I_TO_V[73]; UNION_U08_F32.input[3] = rxData_I_TO_V[72];
				Rocket.ATT[2]  = (double)UNION_U08_F32.value;
			}
		}

		if ( rxData_I_TO_V[79] == INS_ACC )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[92];
			UNION_U08_U16.input[1] = rxData_I_TO_V[93];

			if ( UNION_U08_U16.value )
			{
				UNION_U08_F32.input[0] = rxData_I_TO_V[83]; UNION_U08_F32.input[1] = rxData_I_TO_V[82]; UNION_U08_F32.input[2] = rxData_I_TO_V[81]; UNION_U08_F32.input[3] = rxData_I_TO_V[80];
				Rocket.ACC[0]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[87]; UNION_U08_F32.input[1] = rxData_I_TO_V[86]; UNION_U08_F32.input[2] = rxData_I_TO_V[85]; UNION_U08_F32.input[3] = rxData_I_TO_V[84];
				Rocket.ACC[1]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[91]; UNION_U08_F32.input[1] = rxData_I_TO_V[90]; UNION_U08_F32.input[2] = rxData_I_TO_V[89]; UNION_U08_F32.input[3] = rxData_I_TO_V[88];
				Rocket.ACC[2]  = (double)UNION_U08_F32.value;
			}
		}

		if ( rxData_I_TO_V[95] == INS_ANV )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[108];
			UNION_U08_U16.input[1] = rxData_I_TO_V[109];

			if ( UNION_U08_U16.value )
			{
				UNION_U08_F32.input[0] = rxData_I_TO_V[ 99]; UNION_U08_F32.input[1] = rxData_I_TO_V[ 98]; UNION_U08_F32.input[2] = rxData_I_TO_V[ 97]; UNION_U08_F32.input[3] = rxData_I_TO_V[ 96];
				Rocket.ANV[0]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[103]; UNION_U08_F32.input[1] = rxData_I_TO_V[102]; UNION_U08_F32.input[2] = rxData_I_TO_V[101]; UNION_U08_F32.input[3] = rxData_I_TO_V[100];
				Rocket.ANV[1]  = (double)UNION_U08_F32.value;

				UNION_U08_F32.input[0] = rxData_I_TO_V[107]; UNION_U08_F32.input[1] = rxData_I_TO_V[106]; UNION_U08_F32.input[2] = rxData_I_TO_V[105]; UNION_U08_F32.input[3] = rxData_I_TO_V[104];
				Rocket.ANV[2]  = (double)UNION_U08_F32.value;
			}
		}

		INS.COUNT++;
	}
}


__inline void parse_GPS( uint8_t *rxData_I_TO_V )
{
	if ( rxData_I_TO_V[3] == GPS_PAYLOAD )
	{
		if ( rxData_I_TO_V[19] == GPS_FIX )
		{
			UNION_U08_U16.input[0] = rxData_I_TO_V[24];
			UNION_U08_U16.input[1] = rxData_I_TO_V[25];

			const uint16_t FIX_Validity = UNION_U08_U16.value;

			if ( FIX_Validity == 0x0007 )
			{
				GPS.Fix    = rxData_I_TO_V[20];
				GPS.SatNum = rxData_I_TO_V[21];
				GPS.Fix    = rxData_I_TO_V[23];
			}
		}

		GPS.COUNT++;
	}
}


void Transmit_V_TO_P( UART_Handler *uart )
{
	uint8_t checksum = 0b00000000;
	int32_t  value1;
	uint32_t value2;

	txData_V_TO_P[0] = txh0_V_TO_P;
	txData_V_TO_P[1] = txh1_V_TO_P;
//
//	Compression_2_U08( &Rocket.POS[0], &txData_V_TO_G[ 2], _CONVERT_I16_2_U08, POS_RES_TX );
//	Compression_2_U08( &Rocket.POS[1], &txData_V_TO_G[ 4], _CONVERT_I16_2_U08, POS_RES_TX );
//	Compression_2_U08( &Rocket.POS[2], &txData_V_TO_G[ 6], _CONVERT_I16_2_U08, POS_RES_TX );
//
//	Compression_2_U08( &Rocket.VEL[0], &txData_V_TO_G[ 8], _CONVERT_I16_2_U08, VEL_RES_TX );
//	Compression_2_U08( &Rocket.VEL[1], &txData_V_TO_G[10], _CONVERT_I16_2_U08, VEL_RES_TX );
//	Compression_2_U08( &Rocket.VEL[2], &txData_V_TO_G[12], _CONVERT_I16_2_U08, VEL_RES_TX );
//
//	Compression_2_U08( &Rocket.ATT[0], &txData_V_TO_G[14], _CONVERT_I16_2_U08, ATT_RES_TX );
//	Compression_2_U08( &Rocket.ATT[1], &txData_V_TO_G[16], _CONVERT_I16_2_U08, ATT_RES_TX );
//	Compression_2_U08( &Rocket.ATT[2], &txData_V_TO_G[18], _CONVERT_I16_2_U08, ATT_RES_TX );
//
//	Compression_2_U08( &Rocket.ANV[0], &txData_V_TO_G[20], _CONVERT_I16_2_U08, ANV_RES_TX );
//	Compression_2_U08( &Rocket.ANV[1], &txData_V_TO_G[22], _CONVERT_I16_2_U08, ANV_RES_TX );
//	Compression_2_U08( &Rocket.ANV[2], &txData_V_TO_G[24], _CONVERT_I16_2_U08, ANV_RES_TX );

	Compress_U16_2_U08( (&Controller)->RCS1, &txData_V_TO_P[2] );
	Compress_U16_2_U08( (&Controller)->RCS2, &txData_V_TO_P[4] );
	Compress_U16_2_U08( (&Controller)->RCS3, &txData_V_TO_P[6] );
	Compress_U16_2_U08( (&Controller)->RCS4, &txData_V_TO_P[8] );

	value1 = ( int32_t)( (&Controller)->theta[0] * 2.4608349915022000e10 ); Compress_I32_2_U08( value1, &txData_V_TO_P[10] );
	value1 = ( int32_t)( (&Controller)->theta[1] * 2.4608349915022000e10 ); Compress_I32_2_U08( value1, &txData_V_TO_P[14] );
	value2 = (uint32_t)( (&Controller)->Thrust   * 9.5443717666666700e06 ); Compress_U32_2_U08( value2, &txData_V_TO_P[18] );

	Compress_U32_2_U08( CNT_COUNT, &txData_V_TO_P[22] );

	CNT_COUNT++;

	for ( int i = 2; i < bfsz_V_TO_P-1; i++ )
	{
		checksum += txData_V_TO_P[i];
	}

	txData_V_TO_P[bfsz_V_TO_P-1] = checksum;

	UART_Transmit( uart, txData_V_TO_P, bfsz_V_TO_P );
}
