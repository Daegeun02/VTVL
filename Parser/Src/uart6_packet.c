/*
 * uart7_packet.c
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#include <uart6_packet.h>


uint8_t rxBuff_E_TO_V[2];
uint8_t rxData_E_TO_V[ENG_PAYLOAD];

uint8_t ready_E_TO_V = 0;

uint8_t rxh0_E_TO_V = 0xFC;
uint8_t rxh1_E_TO_V = 0xC5;
uint8_t rxln_E_TO_V = 0x52;
uint8_t rxft_E_TO_V = 0x5A;

uint8_t hdrf_E_TO_V = 0;
uint8_t idxn_E_TO_V = 0;

uint16_t _sum_E_TO_V = 0;
uint16_t Rsum_E_TO_V = 0;
uint16_t Tsum_E_TO_V = 0;

extern st_Actuator Actuator;



void Read_E_TO_V( UART_Handler *uart )
{
	if ( UART_Receive_1B( uart, &rxBuff_E_TO_V[0] ) )
	{
		if ( hdrf_E_TO_V == 3 )
		{
			rxData_E_TO_V[idxn_E_TO_V] = rxBuff_E_TO_V[0];

			idxn_E_TO_V++;

			_sum_E_TO_V += rxBuff_E_TO_V[0];

			if ( idxn_E_TO_V == ENG_PAYLOAD-3 )
			{
				hdrf_E_TO_V++;
			}
		}

		else if ( hdrf_E_TO_V == 0 )
		{
			if ( rxBuff_E_TO_V[0] == rxh0_E_TO_V )
			{
				rxData_E_TO_V[idxn_E_TO_V] = rxh0_E_TO_V;

				hdrf_E_TO_V++;
				idxn_E_TO_V++;
			}
			else
			{
				hdrf_E_TO_V = 0;
				idxn_E_TO_V = 0;
			}
		}

		else if ( hdrf_E_TO_V == 1 )
		{
			if ( rxBuff_E_TO_V[0] == rxh1_E_TO_V )
			{
				rxData_E_TO_V[idxn_E_TO_V] = rxh1_E_TO_V;

				hdrf_E_TO_V++;
				idxn_E_TO_V++;
			}
			else
			{
				hdrf_E_TO_V = 0;
				idxn_E_TO_V = 0;
			}
		}

		else if ( hdrf_E_TO_V == 2 )
		{
			if ( rxBuff_E_TO_V[0] == rxln_E_TO_V )
			{
				rxData_E_TO_V[idxn_E_TO_V] = rxln_E_TO_V;

				_sum_E_TO_V += rxln_E_TO_V;

				hdrf_E_TO_V++;
				idxn_E_TO_V++;
			}
			else
			{
				hdrf_E_TO_V = 0;
				idxn_E_TO_V = 0;
			}
		}

		else if ( hdrf_E_TO_V == 4 )
		{
			rxData_E_TO_V[idxn_E_TO_V] = rxBuff_E_TO_V[0];

			idxn_E_TO_V++;

			if ( idxn_E_TO_V == ENG_PAYLOAD-1 )
			{
				hdrf_E_TO_V++;
			}
		}

		else if ( hdrf_E_TO_V == 5 )
		{
			rxData_E_TO_V[idxn_E_TO_V] = rxBuff_E_TO_V[0];

			if ( rxBuff_E_TO_V[0] == 0x5A )
			{
				UNION_U08_U16.input[0] = rxData_E_TO_V[idxn_E_TO_V-2];
				UNION_U08_U16.input[1] = rxData_E_TO_V[idxn_E_TO_V-1];

				Rsum_E_TO_V = _sum_E_TO_V;
				Tsum_E_TO_V = UNION_U08_U16.value;

				if ( Rsum_E_TO_V == Tsum_E_TO_V )
				{
					ready_E_TO_V = 1;

					hdrf_E_TO_V = 0;
					idxn_E_TO_V = 0;
					_sum_E_TO_V = 0;
				}
			}

			else
			{
				ready_E_TO_V = 0;

				hdrf_E_TO_V = 0;
				idxn_E_TO_V = 0;
				_sum_E_TO_V = 0;
			}
		}

		else
		{
			hdrf_E_TO_V = 0;
			idxn_E_TO_V = 0;
			_sum_E_TO_V = 0;

			ready_E_TO_V = 0;
		}
	}
}


void parse_ENG( uint8_t *rxbf )
{
	char arr1[5] = {0,};
	char arr2[4] = {0,};

	Actuator.RPM = ( rxbf[15] << 8 | rxbf[14] << 0 ) * 100;

	Actuator.EngineStatus[ 0] = (char)rxbf[30];
	Actuator.EngineStatus[ 1] = (char)rxbf[31];
	Actuator.EngineStatus[ 2] = (char)rxbf[32];
	Actuator.EngineStatus[ 3] = (char)rxbf[33];
	Actuator.EngineStatus[ 4] = (char)rxbf[34];
	Actuator.EngineStatus[ 5] = (char)rxbf[35];
	Actuator.EngineStatus[ 6] = (char)rxbf[36];
	Actuator.EngineStatus[ 7] = (char)rxbf[37];
	Actuator.EngineStatus[ 8] = (char)rxbf[38];
	Actuator.EngineStatus[ 9] = (char)rxbf[39];
	Actuator.EngineStatus[10] = (char)rxbf[40];
	Actuator.EngineStatus[11] = (char)rxbf[41];
	Actuator.EngineStatus[12] = (char)rxbf[42];
	Actuator.EngineStatus[13] = (char)rxbf[43];

	if ( isdigit( rxbf[49] ) ) Actuator.ThrustLevel = atoi( (char*)(&rxbf[49]) );
	else                       Actuator.ThrustLevel = 0;

	memcpy( arr1, &rxbf[54], sizeof(float) ); Actuator.Vb = atof( arr1 );
	memcpy( arr1, &rxbf[62], sizeof(float) ); Actuator.Vr = atof( arr1 );

	memcpy( arr2, &rxbf[70], sizeof(float) ); Actuator.Power       = atoi( arr2 );
	memcpy( arr2, &rxbf[78], sizeof(float) ); Actuator.Temperature = atoi( arr2 );

	Actuator.COUNT++;
}
