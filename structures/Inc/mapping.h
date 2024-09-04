/*
 * mapping.h
 *
 *  Created on: Aug 27, 2024
 *      Author: daegeun
 */

#ifndef INC_MAPPING_H_
#define INC_MAPPING_H_

#include <stdint.h>

#include <timer.h>

#include <uart3_packet.h>


typedef enum {
	MOVING = 0,
	SERVO1 = 1,
	SERVO2 = 2
} MappingMode;


typedef struct {
	MappingMode MODE;

	uint32_t TVC1;
	uint32_t TVC2;

	float Gain1;
	float Gain2;

	uint32_t trim1;
	uint32_t trim2;

	uint8_t CUTOFF;

	uint8_t LOCK;
} st_Mapping;


extern st_Mapping Mapping;


void Init_Mapping( st_Mapping *map );

#endif /* INC_MAPPING_H_ */
