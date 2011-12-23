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

//Main config file
#include "config/BLDC_config.h"

//Only include modules that are used
#include "rc_signal.h"
#include "debug.h"

void setPWM(uint8_t val);
void setCommTime(uint16_t val);
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
volatile uint16_t 	startZcLowTime;
volatile uint16_t 	prevZCTime;
volatile uint16_t 	newZCTime;
volatile uint16_t 	newCommTime;
volatile uint16_t 	currCommTime; //Half of previous commutation time
volatile uint16_t	startupRampElapsed; //elapsed time during startup ramp
volatile uint16_t 	stabilisingCounts; //Used to track how many commutations have occured during stabilisation
volatile uint16_t 	t1_ovfs;
volatile uint8_t 	startupState;
volatile uint8_t 	pwmVal; //0->255
volatile uint8_t 	pwmPhase; //0=NONE, 1=A, 2=B, 3=C
volatile uint8_t 	motorON; //0=off, 1=on
volatile uint8_t 	commState; //1->6, then loops
volatile uint16_t 	testcount;
volatile uint16_t 	testoutput;

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
	prevZCTime = 0;
	newZCTime = 0;
	newCommTime = 0;
	
	while(1) {
		if (signalBuffer) { //When signal buffer is populated, update PWM
			pwmVal = processRCSignal(signalBuffer);
			signalBuffer = 0; //Clear signal
			
			if (pwmVal > PWM_ON_THR) {
				startMotor();
			}
		}
		
		while (motorON) {
			if (signalBuffer) { //When signal buffer is populated, update PWM
				pwmVal = processRCSignal(signalBuffer);
				signalBuffer = 0; //Clear signal
				setPWM(pwmVal);
				//USART_SendInt(pwmVal);
				//USART_NewLine();
				cli();
				uint16_t temp = newCommTime;
				sei();
				USART_SendInt(temp);
				USART_NewLine();
				
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
	PORTB &= ~(1 << PB1);
	clrLED();
	testoutput = testcount;
	if (!zcDetected) { newZCTime += TCNT1 - startZcLowTime; } //add last section of timer to ZC_low
	newCommTime = (prevZCTime + newZCTime); //averages new commutation period based on last 2 zero crossings
	
	if (newCommTime < (prevZCTime *2 - 10)){
		newCommTime = prevZCTime *2 -10;
	}
	if (newCommTime > (prevZCTime *2 + 10)){
		newCommTime = prevZCTime *2 +10;
	}
	
	OCR1B = newZCTime;
	
		/*
		//Clip newCommTime between half and double of oldCommTime
		if (newCommTime < (currCommTime / 2)) {
			newCommTime = (uint16_t)(currCommTime / 2);
		}
		if (newCommTime > (currCommTime * 2)) {
			newCommTime = (uint16_t)(currCommTime * 2);
		}
		*/
		

	if (!startupState) prevZCTime = newCommTime / 2; //Reset calculation for new cycle
	
	switch (startupState) {
	case 1:
		//calculate next timer period
		//OCR1A *= (1 - STARTUP_RAMP_FACTOR);
		startupRampElapsed += OCR1A / 256;
		double frac = startupRampElapsed / (double)STARTUP_DURATION; //Fraction of elapsed time to total startup duration
		//uint16_t temp = (uint16_t)(STARTUP_TICKS_BEGIN - (STARTUP_TICKS_BEGIN - STARTUP_TICKS_END)*((3-2*frac)*frac*frac));
		setCommTime((uint16_t)(STARTUP_TICKS_BEGIN - (STARTUP_TICKS_BEGIN - STARTUP_TICKS_END)*((3-2*frac)*frac*frac))); //Calculate new wait period based on S-curve
		//OCR1A = (uint16_t)(STARTUP_TICKS_BEGIN - (STARTUP_TICKS_BEGIN - STARTUP_TICKS_END)*((3-2*frac)*frac*frac));
		break;
	case 2:
		//stabilisation state, just continue commutating
		stabilisingCounts++;
		break;
	}
	
	nextCommutation();
	//Only update commTime if not in startup state
	if (!startupState) {
		//OCR1A = currCommTime - (currCommTime - newCommTime)/64;
		setCommTime(newCommTime);
	}
	testcount = 0;
}

ISR(TIMER1_COMPB_vect)
{
	//PORTB |= (1 << PB1);
	setLED();
}

ISR(TIMER1_OVF_vect)
{
	if (zcActive && !startupState) {
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
	//testcount++;
	//if (testcount % 5 == 0) nextCommutation();
	//if (testcount >= 30) stopMotor();
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
	//TCNT2 = 156;
}

ISR(TIMER1_CAPT_vect)
{
	//if (zcActive) {
		//Only execute when in ZC detection mode
	//	zcTime = ICR1;
	//	OCR1B = zcTime + ZC_SPIKE_CHECK;
	//}
	//newZCTime += TCNT1 - startZcLowTime;
	//zcDetected = 1; //ZC flag to detect spikes, it is cleared when ACO changes back to original reading
	//PORTB |= (1 << PB1);
	//setLED();
}

ISR(ANA_COMP_vect)
{
	//Analog comparator trigger is always the opposite of a ZC, it detects when a spike/false ZC occurs
	//startZcLowTime = TCNT1;
	//zcDetected = 0;
	//PORTB &= ~(1 << PB1);
	
	
	uint8_t acoReading = (ACSR & (1 << ACO)) >> ACO;
	if (acoReading != targetACO) {
		//ZC_high
		newZCTime += TCNT1 - startZcLowTime;
		testcount++;
		zcDetected = 1; //ZC flag to detect spikes, it is cleared when ACO changes back to original reading
		PORTB |= (1 << PB1);
	}
	else {
		//ZC_low
		startZcLowTime = TCNT1;
		zcDetected = 0;
		PORTB &= ~(1 << PB1);
	}
	
}

void setPWM(uint8_t val)
{
	pwmVal = val; //store current pwm value here
	OCR2 = pwmVal;
}

void setCommTime(uint16_t val)
{
	currCommTime = val;
	OCR1A = val;
}

void startMotor(void)
{
	uint8_t i; //used for "for" loops
	
	//Initialise PWM & commutation variables
	pwmPhase = 1;
	commState = 1;
	startupState = 1;
	TCNT1 = 0;
	startupRampElapsed = 0;
	prevZCTime = STARTUP_TICKS_END / 2;
	
	//stopZCDetection(); //Make sure ZC detection is off
	
	setPWM(STARTUP_LOCK_PWM);
	START_PWM(); //enable PWM
	
	DISABLE_COMMUTATION();
	DISABLE_OC1B();
	TIMER1_START();
	
	for (i=0; i<STARTUP_RLOCK_LOOPS; i++) {
		t1_ovfs = 0;
		nextCommutation();
		while (t1_ovfs < STARTUP_RLOCK) {} //wait until defined overflows occurs
	}
	
	//Commutations now handled in interrupts
	setPWM(STARTUP_RAMP_PWM);
	setCommTime(STARTUP_TICKS_BEGIN); //OCR1A = STARTUP_TICKS_BEGIN;
	ENABLE_COMMUTATION();
	OCR1B = STARTUP_TICKS_BEGIN;
	ENABLE_OC1B();
	while (currCommTime > STARTUP_TICKS_END) {} //Wait until ramp has finished
	
	//continue commutating at the same timer period to stabilise rotor speed
	startupState = 2;
	//setPWM(120);
	pwmVal = processRCSignal(signalBuffer);
	signalBuffer = 0; //Clear signal
	setPWM(pwmVal);
	setLED();
	stabilisingCounts = 0;
	while (stabilisingCounts < 500){}//STARTUP_STABILISE) {}
	clrLED(); //for debugging
	/*
	while (pwmVal > 40) {
		pwmVal--;
		setPWM(pwmVal);
		stabilisingCounts = 0;
		while (stabilisingCounts < 50) {
				USART_SendInt(pwmVal);
				USART_NewLine(); }
	}
	while (pwmVal > 32) {
		pwmVal--;
		setPWM(pwmVal);
		stabilisingCounts = 0;
		while (stabilisingCounts < 500) {
				USART_SendInt(pwmVal);
				USART_NewLine(); }
	}
	*/
	//TODO: sense back-emf before passing to closed loop
	
	//Getting ready to detect ZC using analog comparator
	startupState = 0;
	motorON = 1; //Enter 'motor on' state
}

void stopMotor(void)
{
	//stopZCDetection();
	TIMER1_STOP(); //Commutation timer
	STOP_PWM();
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
	
	//Initialise zero crossing detection variables before commutation
	TCNT1 = 0;
	startZcLowTime = 0;
	newZCTime = 0;
	zcDetected = 0;
	if (commState % 2 == 0) {
		//Even number, Floating voltage going low -> high (rising)
		detectRisingBEMF();
	}
	else {
		//Odd number, Floating voltage going high -> low (falling)
		detectFallingBEMF();
	}
	
	switch (commState) {
	case 1:
		//Do comparator stuff here before switching occurs
		ADMUX = ADC_C;
		
		CLR_C_HIGH();
		SET_B_HIGH();
		break;
	case 2:
		ADMUX = ADC_A;
		
		//Change pwmPhase 1->3, check current state
		if (GET_A_LOW()) {
			CLR_A_LOW();
			SET_C_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 3; //Set pwm to C_LOW
	  	break;
	case 3:
		ADMUX = ADC_B;
		
		CLR_B_HIGH();
		SET_A_HIGH();
	  	break;
	case 4:
		ADMUX = ADC_C;
		
		//Change pwmPhase 3->2, check current state
		if (GET_C_LOW()) {
			CLR_C_LOW();
			SET_B_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 2; //Set pwm to B_LOW
	  	break;
	case 5:
		ADMUX = ADC_A;
		
		CLR_A_HIGH();
		SET_C_HIGH();
	  	break;
	case 6:
		ADMUX = ADC_B;
		
		//Change pwmPhase 2->1, check current state
		if (GET_B_LOW()) {
			CLR_B_LOW();
			SET_A_LOW(); //if previous pwmPhase mosfet was on, set new phase to on
		}
		//If mosfet was off, then the new phase will be activated on the next pwm interrupt,
		//	no need to do anything extra
		pwmPhase = 1; //Set pwm to A_LOW
		commState = 0; //Start from beginning
	  	break;
	}
	commState++;
}

void detectRisingBEMF(void)
{
	//TCCR1B &= ~(1 << ICES1); //A falling comparator is a rising B-EMF
	//ACSR |= (1 << ACIS0); //Detects when ACO returns to its former state (i.e. when a spike happens)
	targetACO = 0;
}

void detectFallingBEMF(void)
{
	//TCCR1B |= (1 << ICES1); //A rising comparator is a falling B-EMF
	//ACSR &= ~(1 << ACIS0); //Detects when ACO returns to its former state (i.e. when a spike happens)
	targetACO = 1;
}

void startZCDetection(void)
{
	//TIMSK |= (1 << TICIE1);
	//ACSR |= (1 << ACIE);
	zcActive = 1;
}

void stopZCDetection(void)
{
	//TIMSK &= ~(1 << TICIE1);
	//ACSR &= ~(1 << ACIE);
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
	//ACSR |= (1 << ACIC) | (1 << ACIS1);
	
	//TIMSK |= (1 << TICIE1); //enable input capture interrupt
	ACSR |= (1 << ACIE); //enable ACO interrupt
	startZCDetection();
	
	sei(); //Enable interrupts
}