/* 
 * qut-brushless-controller, an open-source Brushless DC motor controller
 * Copyright (C) 2011 Toby Lockley <tobylockley@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//===================================//
//      Main Controller File         //
//===================================//
//Initialisation and main program code
#include <avr/io.h>
#include <avr/interrupt.h>

//Pin definitions and config files
#include "config/BLDC_pindefs_breadboard.h"
#include "config/BLDC_config.h"
//#include "config/BLDC_motor_A2208.h"
//#include "config/BLDC_motor_smallpink.h"
#include "config/BLDC_motor_smallsilver.h"
//#include "config/BLDC_motor_HDD.h"

//Controller is broken up into easy to manage stubs
#include "rc_signal.h"
#include "debug.h"

void setPWM(uint8_t val);
void startMotor(void);
void stopMotor(void);
void nextCommutation(void);
void detectRisingBEMF(void);
void detectFallingBEMF(void);
void startZCDetection(void);
void stopZCDetection(void);
void clrAllOutputs(void);
void init_mosfets(void);
void init_registers(void);

extern volatile	uint16_t 	signalBuffer; //See rc_signal.h

volatile uint8_t 	zcActive; //Whether ZC detection is active or not
volatile uint8_t 	zcDetected; //Sets flag when ZC occurs, clears if it was just a spike
volatile uint8_t 	targetACO; //What the ACO reading should be after a ZC (either 1 or 0)
volatile uint16_t 	zcTime; //Timer ticks when ZC was detected
volatile uint16_t 	nextComm; //Timer ticks when next commutation will occur
volatile uint16_t 	prevCommDiv2; //Half of previous commutation time
volatile uint8_t 	t1_ovfs;
volatile uint8_t 	startupState;
volatile uint8_t 	pwmVal; //0->255
volatile uint8_t 	pwmPhase; //0=NONE, 1=A, 2=B, 3=C
volatile uint8_t 	motorON; //0=off, 1=on
volatile uint8_t 	commState; //1->6, then loops

int main(void)
{
	
	init_mosfets();
	init_registers();
	init_debug();
	
	switch (INPUT_SIGNAL_MODE) {
	case 1:
		init_rc();
		break;
	case 2:
		//init_twi();
		break;
	case 3:
		//init_uart();
		break;
	default:
		//NO INPUT SELECTED//
		break;
	}
	
	//Initialise variables
	motorON = 0;
	pwmVal = 0;
	zcActive = 0;
	
	while(1) {
		if (signalBuffer) { //When signal buffer is populated, update PWM
			pwmVal = processRCSignal(signalBuffer);
			signalBuffer = 0; //Clear signal
			
			if (pwmVal > PWM_ON_THR) {
				startMotor();
				setLED();
				motorON = 1; //Enter 'motor on' loop below
			}
		}
		
		while (motorON) {
			if (signalBuffer) { //When signal buffer is populated, update PWM
				pwmVal = processRCSignal(signalBuffer);
				signalBuffer = 0; //Clear signal
				setPWM(pwmVal);
				
				if (pwmVal < PWM_OFF_THR) {
					//turn off motor (allow coast to a stop), disable pwm
					stopMotor();
				}
			}
		}
	}
}

ISR(TIMER1_COMPA_vect)
{
	if (startupState) {
		OCR1A *= (1 - STARTUP_RAMP);
	}
	else {
		DISABLE_OC1A(); //disable commutation interrupt
	}
	nextCommutation();
	TCNT1 = 0;
}

ISR(TIMER1_COMPB_vect)
{
	if (zcActive) {
		//Input capture interrupt is set, so zero crossing was detected
		if (zcDetected) {
			//Analog comparator output has remained the same since interrupt occured, zero crossing has been detected
			stopZCDetection();
			TIFR |= (1 << ICF1); //Make sure input capture flag is cleared by writing logic 1 to it
			
			nextComm = zcTime + prevCommDiv2;
			OCR1A = nextComm; //Set commutation time to half last commutation period
			ENABLE_OC1A(); //Enable commutation to occur
			prevCommDiv2 = (nextComm)/2; //Adjusts new comm time based on early/late ZC
			
			//TODO: SET FLAG FOR MAIN LOOP TO CALCULATE VOLTAGE/CURRENT ADC
		}
	}
	else {
		//End of blanking period. Start detecting ZC
		if (zcDetected) {
			//ZC may have happened already, start spike detection
			zcTime = OCR1B;
			OCR1B += ZC_SPIKE_CHECK;
		}
		startZCDetection();
	}
}

ISR(TIMER1_OVF_vect)
{
	if (zcActive) {
		//Timer has overflowed during ZC detection, motor is stalled
		stopMotor();
	}
	t1_ovfs++; //Keep track of how many times timer1 overflows
}

ISR(TIMER2_COMP_vect)
{
	CLR_A_LOW();
	CLR_B_LOW();
	CLR_C_LOW();
}

ISR(TIMER2_OVF_vect)
{
	switch (pwmPhase) {
	case 1:
		SET_A_LOW();
		break;
	case 2:
		SET_B_LOW();
		break;
	case 3:
		SET_C_LOW();
		break;
	}
}

ISR(TIMER1_CAPT_vect)
{
	zcTime = ICR1;
	OCR1B = zcTime + ZC_SPIKE_CHECK;
	zcDetected = 1; //ZC flag to detect spikes, it is cleared when ACO changes back to original reading
	PORTB |= (1 << PB1);
}

ISR(ANA_COMP_vect)
{
	//Analog comparator trigger is always the opposite of a ZC, it detects when a spike/false ZC occurs
	zcDetected = 0;
	PORTB &= ~(1 << PB1);
}

void setPWM(uint8_t val)
{
	OCR2 = val;
}

void startMotor(void)
{
	uint8_t i; //used for "for" loops

	//TIMSK &= ~(1 << OCIE1B);
	
	//Initialise PWM & commutation variables
	pwmPhase = 1;
	TCNT1 = 0;
	commState = 1;
	startupState = 1;
	
	stopZCDetection(); //Make sure ZC detection is off
	
	setPWM(STARTUP_LOCK_PWM);
	PWM_START(); //enable PWM
	
	DISABLE_OC1A();
	DISABLE_OC1B();
	TIMER1_START();
	
	for (i=0; i<2; i++) {
		t1_ovfs = 0;
		nextCommutation();
		while (t1_ovfs < STARTUP_RLOCK) {} //wait until defined overflows occurs
	}
	
	//Commutations now handled in interrupts   
	setPWM(STARTUP_RAMP_PWM);
	OCR1A = STARTUP_TICKS_BEGIN;
	ENABLE_OC1A();
	while (OCR1A > STARTUP_TICKS_END) {} //Wait until ramp has finished
	
	//Getting ready to detect ZC using analog comparator
	DISABLE_OC1A();
	prevCommDiv2 = OCR1A / 2;
	startupState = 0;
	OCR1B = ZC_BLANKING_TICKS; //Set blanking period to avoid inductive spikes
	ENABLE_OC1B();
}

void stopMotor(void)
{
	stopZCDetection();
	TIMER1_STOP(); //Commutation timer
	PWM_STOP();
	setPWM(0);
	clrAllOutputs();
	clrLED(); //for debugging
	motorON = 0; //Leave 'motor on' loop
}

void nextCommutation(void)
{
	//When switching low side, extra logic tests are used to make sure if PWM was in an 'on' state,
	//	it will remain that way and visa versa
	
	//To make code more efficient, it is assumed that commState always starts at 0 after a MCU reset,
	//	and direction of rotation can not change during operation.
	
	//TODO: ADD IF STATEMENT FOR DIRECTION OF SPIN
	
	if (commState % 2 == 0) {
		//Even number, rising BEMF
		detectRisingBEMF();
		targetACO = 0;
	}
	else {
		//Odd number, falling BEMF
		detectFallingBEMF();
		targetACO = 1;
	}
	
	switch (commState) {

	case 1:
		//Do comparator stuff here before switching occurs
		ADMUX = ADC_C;
		
		SET_B_HIGH();
		CLR_C_HIGH();
		break;
	case 2:
		ADMUX = ADC_A;
		
		//Change pwmPhase 1->3, check current state
		if (GET_A_LOW()) {
			SET_C_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
			CLR_A_LOW();
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 3; //Set pwm to C_LOW
	  	break;
	case 3:
		ADMUX = ADC_B;
		
		SET_A_HIGH();
		CLR_B_HIGH();
	  	break;
	case 4:
		ADMUX = ADC_C;
		
		//Change pwmPhase 3->2, check current state
		if (GET_C_LOW()) {
			SET_B_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
			CLR_C_LOW();
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 2; //Set pwm to B_LOW
	  	break;
	case 5:
		ADMUX = ADC_A;
		
		SET_C_HIGH();
		CLR_A_HIGH();
	  	break;
	case 6:
		ADMUX = ADC_B;
		
		//Change pwmPhase 2->1, check current state
		if (GET_B_LOW()) {
			SET_A_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
			CLR_B_LOW();
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 1; //Set pwm to A_LOW
		commState = 0; //Start from beginning
	  	break;
	}
	commState++;
		
	if (!startupState) OCR1B = ZC_BLANKING_TICKS; //Set blanking period
}

void detectRisingBEMF(void)
{
	TCCR1B &= ~(1 << ICES1); //A falling comparator is a rising B-EMF
	ACSR |= (1 << ACIS0); //Detects when ACO returns to its former state (i.e. when a spike happens)
}

void detectFallingBEMF(void)
{
	TCCR1B |= (1 << ICES1); //A rising comparator is a falling B-EMF
	ACSR &= ~(1 << ACIS0); //Detects when ACO returns to its former state (i.e. when a spike happens)
}

void startZCDetection(void)
{
	TIMSK |= (1 << TICIE1);
	ACSR |= (1 << ACIE);
	zcActive = 1;
}

void stopZCDetection(void)
{
	TIMSK &= ~(1 << TICIE1);
	ACSR &= ~(1 << ACIE);
	zcActive = 0;
}

void clrAllOutputs(void)
{
	CLR_A_HIGH();
	CLR_B_HIGH();
	CLR_C_HIGH();
	CLR_A_LOW();
	CLR_B_LOW();
	CLR_C_LOW();
}

void init_mosfets(void)
{
	//Set mosfet drive pins as outputs
    HIGH_A_DDR |= (1 << HIGH_A);
    HIGH_B_DDR |= (1 << HIGH_B);
    HIGH_C_DDR |= (1 << HIGH_C);
    LOW_A_DDR |= (1 << LOW_A);
    LOW_B_DDR |= (1 << LOW_B);
    LOW_C_DDR |= (1 << LOW_C);
		
	//Turn all mosfets off
	clrAllOutputs();
}

void init_registers(void)
{
	//Set up registers, timers and interrupts

	//Edit the following registers in the header file
	
	DDRB |= (1 << PB1);
	
	//Timer1 - commutation timer
	TIMSK |= TIMER1_TIMSK;
	
	//Timer2 - PWM timer
	TCCR2 = TIMER2_TCCR2;
	TIMSK |= TIMER2_TIMSK;
	
	//Analog comparator settings
	SFIOR |= (1 << ACME); //Set comparator -ve input to ADMUX
	ACSR |= (1 << ACIC) | (1 << ACIS1);
	
	//CAPT_RISING();
	//ACO_FALLING();
	//ADMUX = ADC_A;
	
	//ZC_START_DETECT();
	//ACO_ON();
	
	sei(); //Enable interrupts
}