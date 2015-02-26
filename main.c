/**
	@file	main.c
	@brief	Zumo maze solver main function.
	@details	It uses bluetooth transmitter to send led sensors state, type of node,
						performed reaction and whole track.
*/
#include "MKL46Z4.h"
#include "bluetooth.h"
#include "leds.h"
#include "motorDriver.h"
#include "zumo_button.h"
#include "zumo_buzzer.h"
#include "zumo_ledArray.h"
#include "zumo_maze.h"


/**
	@brief	Function sends via Bluetooth (UART2) state of LED sensor. '1' means black.
	@param	state Binary coded sensor state.
*/
void sendArrayState( char state ){
	int8_t i;
	
	for( i=5; i>=0; i--){
		bt_sendChar( (state & (1<<i)) ? '1' : '0' );
	}
	bt_sendChar( '\r' );
	bt_sendChar( '\n' );
}

/**
	@brief	Function sends via Bluetooth (UART2) command set.
	@param	route Pointer to string.
	@warning	Input string has to be ended with a '\0' (NULL) character.
*/
void sendRoute( char * route ){
	
	while( *route != '\0' ){
		bt_sendChar( *route );
		route++;
	}
	bt_sendChar( '\r' );
}





/*
enum Node_type{
	DEAD_END						= '0',
	FULL_CROSS,						//1
	STRAIGHT_LEFT_CROSS,	//2
	STRAIGHT_RIGHT_CROSS,	//3
	LEFT_RIGHT_CROSS,			//4
	LEFT_TURN,						//5
	RIGHT_TURN,						//6
	MAZE_END							//7
};
*/


/**
	@brief	Zumo maze solver main function.
	@details	This function is built from three parts: calibration, solving the maze, driving to end by orders. 
						It uses bluetooth transmitter to send led sensors state, type of node, performed reaction and whole track.
*/
int main(void){
	
	uint8_t node_type;
	char reaction;
	
	// Initialize everything
	zumo_button_init();
	ledsInitialize();
	zumo_buzzer_init();
	bt_init( BAUD_RATE );
	motorDriverInit();
	la_init();
	
	// Wait for user reaction
	while( !zumo_button_pressed() );
	_delay_ms( 1000 );

	// Calibrate itself
	zm_calibration( 35 );

	
	
	// infinite loop
	while(1){													
			
		// Turn on orange diode on Zumo. Zumo will search for exit.
		ledGreenOff();
		// Wait for user reaction
		while( !zumo_button_pressed() );
		_delay_ms( 1000 );
		
		// Prepare both arrays for incoming data
		zm_clearArray( &nodeArr );
		zm_clearArray( &optimizedNodeArr );
		
		// Get to the end of the maze
		do{		
			zm_driveToNode( 35 );
			sendArrayState( la_getSensorState() );
			
			node_type =  zm_checkNode( 35 );
			sendArrayState( la_getSensorState() );
			bt_sendChar( node_type );
			bt_sendChar( '\r' );
						
			reaction = zm_nodeReaction( node_type, 35 );
			bt_sendChar( reaction );
			bt_sendChar( '\r' );
			
		}while( reaction != 'F' );
		
		// Play some sound
		zb_WRC_start();
		
		
		
		// Optimize route
		zm_routeOptimizer( nodeArr.tab , optimizedNodeArr.tab );
		bt_sendChar( '\r' );
		bt_sendChar( '\r' );
		sendRoute( nodeArr.tab );
		sendRoute( optimizedNodeArr.tab );
		bt_sendChar( '\r' );
		bt_sendChar( '\r' );
		
		// Turn off orange diode on Zumo. Zumo knows where is end.
		ledGreenOn();
		
		// Wait for user reaction
		while( !zumo_button_pressed() );
		_delay_ms( 1000 );
			
		// Get to the end without mistakes
		do{
			zm_driveToNode( 35 );
			sendArrayState( la_getSensorState() );
			
			node_type =  zm_checkNode( 35 );
			sendArrayState( la_getSensorState() );
			bt_sendChar( node_type );
			bt_sendChar( '\r' );
						
			reaction = zm_strictNodeReaction( &optimizedNodeArr, node_type, 35);
			bt_sendChar( reaction );
			bt_sendChar( '\r' );
			
		}while( reaction != 'F' );
		
		// Play some sound
		zb_WRC_start();
	}
}
