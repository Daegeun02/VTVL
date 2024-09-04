/*
 * uart8_packet.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART3_PACKET_H_
#define INC_UART3_PACKET_H_

#include <stdint.h>

#include <uart.h>


#define BUS_PAYLOAD ( 25 )

#define SBUS_HEADER ( 15 )
#define SBUS_FOOTER ( 0 )

#define SBUS_MIN ( 172 )
#define SBUS_MAX ( 1811 )
#define SBUS_NEU ( 992 )

#define SBUS_DIF     ( 0.12210012210012211 )
#define SBUS_DIF_MIN ( 0.12195121951219513 )
#define SBUS_DIF_MAX ( 0.12210012210012211 )


typedef struct {
	int8_t CH[18];
	int8_t LP[18];		// For Low Pass Filter

	uint8_t FrameLost;
	uint8_t Fail_Safe;

	uint32_t COUNT;

	float  w;
	float dt;
	float  G;
} st_Taranis;


extern uint8_t rxData_T_TO_V[BUS_PAYLOAD];

extern uint8_t ready_T_TO_V;

extern st_Taranis Taranis;

void Init_XMP( st_Taranis *Taranis );

void Read_T_TO_V( UART_Handler *uart );

void parse_XMP( uint8_t *rxbf );

#endif /* INC_UART3_PACKET_H_ */
