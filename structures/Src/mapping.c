/*
 * mapping.c
 *
 *  Created on: Aug 27, 2024
 *      Author: daegeun
 */

#include <mapping.h>


extern TIM_Handler tim3;

st_Mapping Mapping;


void Init_Mapping( st_Mapping *map )
{
	map->MODE = MOVING;

	map->Gain1 = 3.;
	map->Gain2 = 3.;

	map->TVC1 = 1500;
	map->TVC2 = 1500;

	map->trim1 = 1750;
	map->trim2 = 1500;

	map->CUTOFF = 1;
}
