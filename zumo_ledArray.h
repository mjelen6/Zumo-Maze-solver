#ifndef ZUMO_LEDARRAY_H_
#define ZUMO_LEDARRAY_H_

#include "MKL46Z4.h"

#define LA_LPTMR_DELAY_CAP_CHARGE	5
#define LA_LPTMR_DELAY_CAP_MAX_DISCHARGE 0xffff

#define LA_PERCENTAGE_SWITCHING_LEVEL 50

typedef struct{
	uint16_t value;
	uint16_t min;
	uint16_t max;
} la_sensor_t;

void la_init(void);
void la_startCal(void);
void la_stopCal(void);
char la_getSensorState(void);
void la_getPercentageReflectance( int16_t * output_array );

void la_pins_init(void);
void la_pins_as_inputs(void);
void la_pins_as_outputs_and_high(void);
void lptimer_reload( uint16_t time_ms );
uint16_t la_getLptmrCNR(void);
void la_calibrateMinMax( volatile la_sensor_t * sensor_array );
char la_calculateSensorState( volatile la_sensor_t * sensor_array );

#endif
