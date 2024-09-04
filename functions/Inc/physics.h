/*
 * physics.h
 *
 *  Created on: Jun 12, 2024
 *      Author: daegeun
 */

#ifndef INC_PHYSICS_H_
#define INC_PHYSICS_H_

#include "actuator.h"
#include "rocket.h"

#include "dcm.h"

#include "navigation.h"


void Do_PhysicalUpdate( st_Rocket *Rocket, st_Actuator *Actuator );

#endif /* INC_PHYSICS_H_ */
