#include "MKL46Z4.h"
#include "zumo_ledArray.h"


volatile la_sensor_t ledArr[6];		// tablica 6-ciu czasow czujnikow
volatile char la_state;
volatile uint8_t measured = 0;
volatile uint8_t cal_flag = 0;
volatile uint8_t valid_data = 0;

void la_init(void){
	
	la_pins_init();
	la_pins_as_outputs_and_high();
	
	SIM->SCGC5 |= SIM_SCGC5_LPTMR_MASK; 	/*Turn on ADC Low Power Timer (LPTMR) registers clock gate*/ 	 
	
		/** Configure LPTMR as timer in 'clear CNR in compare' mode*/ 
	LPTMR0->CSR = (	LPTMR_CSR_TCF_MASK | LPTMR_CSR_TIE_MASK );
	LPTMR0->PSR = ( LPTMR_PSR_PCS( 0 ) | LPTMR_PSR_PBYP_MASK );			/* Set 32kHz MCGIRCLK clock source. No prescaler selected */
	LPTMR0->CMR = LPTMR_CMR_COMPARE( LA_LPTMR_DELAY_CAP_CHARGE );

	/** Enable interrupt*/
	NVIC_ClearPendingIRQ(LPTimer_IRQn); 	/* Clear any pending interrupt */
	NVIC_EnableIRQ(LPTimer_IRQn);
		
	LPTMR0->CSR |=  LPTMR_CSR_TEN_MASK;
}

void la_startCal(void){
	cal_flag = 1;
}
void la_stopCal(void){
	cal_flag = 0;
}

void la_pins_init(void){
	
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTC_MASK | SIM_SCGC5_PORTD_MASK;
	PORTA->PCR[4] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[6] &= ~PORT_PCR_MUX_MASK;
	PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK;
	PORTD->PCR[3] &= ~PORT_PCR_MUX_MASK;
	PORTA->PCR[5] &= ~PORT_PCR_MUX_MASK;

	PORTA->PCR[4] |= PORT_PCR_MUX(1);
	PORTC->PCR[1] |= PORT_PCR_MUX(1);
	PORTD->PCR[6] |= PORT_PCR_MUX(1);
	PORTC->PCR[2] |= PORT_PCR_MUX(1);
	PORTD->PCR[3] |= PORT_PCR_MUX(1);
	PORTA->PCR[5] |= PORT_PCR_MUX(1);	

	PORTA->PCR[4] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTC->PCR[1] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTD->PCR[6] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTC->PCR[2] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTD->PCR[3] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
	PORTA->PCR[5] &= ~(PORT_PCR_PE_MASK | PORT_PCR_PS_MASK);
		
	NVIC_ClearPendingIRQ(PORTC_PORTD_IRQn);				/* Clear NVIC any pending interrupts on PORTC_D */
	NVIC_ClearPendingIRQ(PORTA_IRQn);				/* Clear NVIC any pending interrupts on PORTC_A */
	NVIC_EnableIRQ(PORTC_PORTD_IRQn);
	NVIC_EnableIRQ(PORTA_IRQn);
}

void la_pins_as_inputs(void){
	
	FPTA->PDDR &= ~( (1ul<<4) | (1ul<<5) );
	FPTC->PDDR &= ~( (1ul<<1) | (1ul<<2) );
	FPTD->PDDR &= ~( (1ul<<3) | (1ul<<6) );
	
	PORTA->PCR[4] |= PORT_PCR_IRQC(10);   // Interrupt on falling edge
	PORTC->PCR[1] |= PORT_PCR_IRQC(10);
	PORTD->PCR[6] |= PORT_PCR_IRQC(10);
	PORTC->PCR[2] |= PORT_PCR_IRQC(10);
	PORTD->PCR[3] |= PORT_PCR_IRQC(10);
	PORTA->PCR[5] |= PORT_PCR_IRQC(10);		
}

void la_pins_as_outputs_and_high(void){
	
	PORTA->PCR[4] &= ~PORT_PCR_IRQC_MASK;		// left side (top view)
	PORTC->PCR[1] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[6] &= ~PORT_PCR_IRQC_MASK;		
	PORTC->PCR[2] &= ~PORT_PCR_IRQC_MASK;
	PORTD->PCR[3] &= ~PORT_PCR_IRQC_MASK;
	PORTA->PCR[5] &= ~PORT_PCR_IRQC_MASK;		// right side (top view)
		
	FPTA->PDDR |= (1ul<<4) | (1ul<<5);
	FPTC->PDDR |= (1ul<<1) | (1ul<<2);
	FPTD->PDDR |= (1ul<<3) | (1ul<<6);
	
	FPTA->PSOR |= (1ul<<4) | (1ul<<5);
	FPTC->PSOR |= (1ul<<1) | (1ul<<2);
	FPTD->PSOR |= (1ul<<3) | (1ul<<6);
}

void lptimer_reload( uint16_t time_ms ){
	
	LPTMR0->CSR &= ~( LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK );			/* Disable timer to clear timer register*/
	LPTMR0->CMR = LPTMR_CMR_COMPARE( time_ms );	
	LPTMR0->CSR |=  LPTMR_CSR_TEN_MASK | LPTMR_CSR_TIE_MASK; /* Enable LPTMR timer and interupt */	
}

uint16_t la_getLptmrCNR(void){
	
	uint32_t * point = (uint32_t*)0x4004000Cu;
	*point = 0;
	return LPTMR0->CNR;
}

void la_calibrateMinMax( volatile la_sensor_t * sensor_array ){
	
	uint8_t i;
	for(i=0; i<6; i++){
		if( (sensor_array+i)->value > (sensor_array+i)->max ) (sensor_array+i)->max = (sensor_array+i)->value;
		if( (sensor_array+i)->value < (sensor_array+i)->min ) (sensor_array+i)->min = (sensor_array+i)->value;
	}
}

void la_getPercentageReflectance( int16_t * output_array ){
	
	uint8_t i;
	for(i=0; i<6; i++){
		*(output_array+i) = 100 - (100 * ((ledArr+i)->value - (ledArr+i)->min) / ((ledArr+i)->max - (ledArr+i)->min));
	}
}

char la_calculateSensorState( volatile la_sensor_t * sensor_array ){
	
	char state = 0;
	uint8_t i;
	int16_t temp;
	
	for(i=0; i<6; i++){
		state <<= 1;
		temp = 100 - (100 * ((sensor_array+i)->value - (sensor_array+i)->min) / ((sensor_array+i)->max - (sensor_array+i)->min));
		if( temp < LA_PERCENTAGE_SWITCHING_LEVEL ) state |= 0x01;
	}
	return state;
}


char la_getSensorState( void ){

	while( valid_data == 0 );
	return la_state;
}

void LPTimer_IRQHandler(void){		

	la_pins_as_outputs_and_high();
	LPTMR0->CSR |=  LPTMR_CSR_TCF_MASK;
	lptimer_reload( LA_LPTMR_DELAY_CAP_MAX_DISCHARGE );
	la_pins_as_inputs();
}


void PORTA_IRQHandler(void){
	
	if( PORTA->PCR[4] & PORT_PCR_ISF_MASK ){

		(ledArr+0)->value = la_getLptmrCNR();
		PORTA->PCR[4] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTA->PCR[5] & PORT_PCR_ISF_MASK ){
		
		(ledArr+5)->value = la_getLptmrCNR();
		PORTA->PCR[5] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	
	if( measured == 6 ){
		
		measured = 0;
		la_pins_as_outputs_and_high();
		lptimer_reload( LA_LPTMR_DELAY_CAP_CHARGE );
		
		if( cal_flag == 1 ) la_calibrateMinMax( ledArr );
		
		valid_data = 0;
		la_state = la_calculateSensorState( ledArr );
		valid_data = 1;
	}
}

void PORTC_PORTD_IRQHandler(void){
	
	if( PORTD->PCR[6] & PORT_PCR_ISF_MASK ){
		
		(ledArr+2)->value = la_getLptmrCNR();
		PORTD->PCR[6] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTC->PCR[2] & PORT_PCR_ISF_MASK ){
		
		(ledArr+3)->value = la_getLptmrCNR();
		PORTC->PCR[2] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTC->PCR[1] & PORT_PCR_ISF_MASK ){
		
		(ledArr+1)->value = la_getLptmrCNR();
		PORTC->PCR[1] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	else if( PORTD->PCR[3] & PORT_PCR_ISF_MASK ){
		
		(ledArr+4)->value = la_getLptmrCNR();
		PORTD->PCR[3] |= PORT_PCR_ISF_MASK;
		measured++;
	}
	
	if( measured == 6 ){
		
		measured = 0;
		la_pins_as_outputs_and_high();
		lptimer_reload( LA_LPTMR_DELAY_CAP_CHARGE );
		
		if( cal_flag == 1 ) la_calibrateMinMax( ledArr );
		
		valid_data = 0;
		la_state = la_calculateSensorState( ledArr );
		valid_data = 1;
	}
}
