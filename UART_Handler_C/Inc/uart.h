/*
 * uart_c.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include <stdint.h>
#include <stdbool.h>

#define UART_BUF_SIZE ( 500 )



typedef enum {
    _UART1 = 0x40011000,
    _UART2 = 0x40004400,
    _UART3 = 0x40004800,
	_UART4 = 0x40004C00,
	_UART5 = 0x40005000,
    _UART6 = 0x40011400,
	_UART7 = 0x40007800,
	_UART8 = 0x40007C00
} UARTx;


typedef enum {
	_CR1 = 0x00,
	_CR3 = 0x08,
    _RQR = 0x18,
	_ISR = 0x1C,
	_ICR = 0x20,
	_RDR = 0x24,
	_TDR = 0x28
} UART_Offset_REG;


typedef enum {
	RXNEIE = 0x00000020,
	TXEIE  = 0x00000080,
	EIE    = 0x00000001,
    RXNE   = 0x00000020,
    TXE    = 0x00000080,
	ORE	   = 0x00000008,
	ORECF  = 0x00000008,
    RXFRQ  = 0x00000008
} Def_REG;


typedef struct {
    uint8_t data[UART_BUF_SIZE];
    int32_t r;
    int32_t w;
} RingBuffer;


typedef struct {
    int Adr;
    int CR1;
    int CR3;
    int RQR;
    int ISR;
    int ICR;
    int RDR;
    int TDR;

    uint8_t txbf[UART_BUF_SIZE];
    int32_t txIr;
    int32_t txIw;

    uint8_t rxbf[UART_BUF_SIZE];
    int32_t rxIr;
    int32_t rxIw;

    int error_cnt;
} UART_Handler;


void Init_UART( UARTx UARTn, UART_Handler *uart );

void UART_Start_Rx_IT( UART_Handler *uart );
void UART_Stop_Rx_IT( UART_Handler *uart );

void UART_Transmit( UART_Handler *uart, uint8_t *pdata, uint8_t length );
bool UART_Receive_1B( UART_Handler *uart, uint8_t *pdata );

void IRQ_Handler( UART_Handler *uart );

bool __Get_ISR__( int ISR, Def_REG reg );

void __Transmit__( UART_Handler *uart );
void __Receive__( UART_Handler *uart );

void __TxEnable__( UART_Handler *uart );
void __TxDisable__( UART_Handler *uart );
void __RxEnable__( UART_Handler *uart );
void __RxDisable__( UART_Handler *uart );

#endif /* INC_UART_H_ */
