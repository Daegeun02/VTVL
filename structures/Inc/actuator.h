/*
 * actuator.h
 *
 *  Created on: Jun 9, 2024
 *      Author: daegeun
 */

#ifndef INC_ACTUATOR_H_
#define INC_ACTUATOR_H_

#include <stdint.h>

#include <control.h>


#define TVC_NEUT_1 ( 1500 )
#define TVC_NEUT_2 ( 1500 )


typedef struct {
	double w;
	double j;

	double dY0[2];
	double  Y0[2];

	double dY1[2];
	double  Y1[2];

	double Hz;
	double dt;

	double Kp;
	double Kd;

	double theta[2];

	double tau;
	double tau_inv;

	double  Thrust;
	double dThrust;

	double MaxThrust;

	/* Response Data Start */
	uint16_t TVC_1;		// E
	uint16_t TVC_2;		// N

	uint16_t PWM;
	/* Response Data End */

	/* Data Relay Module Data Start */
	char     EngineStatus[14];

	uint32_t RPM;

	double   ThrustLevel;
	double   Vb;
	double   Vr;
	double   Power;
	double   Temperature;

	uint32_t COUNT;
	/* Data Relay Module Data End */
} st_Actuator;


extern st_Actuator Actuator;


void Init_Actuator( st_Actuator *Actuator );

void Do_Actuator( st_Actuator *Actuator, const st_Controller *Controller );

#endif /* INC_ACTUATOR_H_ */
