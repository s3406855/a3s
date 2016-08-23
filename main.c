/*	TO-DO
*	1. Come up with logic for the main process: outputting pulses while within bounds of shock
*	2. Write interrupt routine to read new values of K_P, K_I, K_D and Reference
*	3. Add LCD functionality to display K_P, K_I, K_D, Ref during setup and testing
*/

/*****	ATMEGA32 SETUP ******
*		Inputs / ADC (Port A)
*		0 = Accelerometer (Top)
*		1 = Accelerometer (Bottom)
*		2 = Not used
*		3 = K_P
*		4 = K_I
*		5 = K_D
*		6 = Reference
*
*		Output
*		Main PWM signal is PWM1A (PD5), Pin 19
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "pid.h"
#include "arduino_libraries/Arduino.h"

/*
*	prototypes
*/
void Init(void);
void Set_Input(int16_t inputValue);
int16_t Get_Measurement(uint8_t ch);
int16_t Get_Reference(void);
void TIMER0_OVF_ISR(void);
float get_from_eeprom(unsigned char a);
void write_to_eeprom(unsigned char a, int16_t val);

/* P, I and D parameter values (P, I and D gains)
 *
 * Controlled by trimpots
 * Volatiles so that they can be seen by Linux for debugging
 */

volatile float K_P = 1.00;
volatile float K_I = 0.00;
volatile float K_D = 0.00;
volatile float REF_VAL = 0.0;

/*
#define K_P     1.00
#define K_I     0.00
#define K_D     0.00
*/

//! Parameters for regulator
struct PID_DATA pidData;

/* Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
#define TIME_INTERVAL   157

/* Flags for status information */
struct GLOBAL_FLAGS
{
	//! True when PID control loop should run one time
	uint8_t pidTimer:1;
	uint8_t dummy:7;
}	gFlags = {0, 0};

int main(void)
{
	int16_t referenceValue, measurementValue, inputValue, lowMeas, hiMeas;
	
	Init();

	sei();	// set global interrupt flag
	
	while(1)
	{
		// Run PID calculations once every PID timer timeout
		if(gFlags.pidTimer)
		{
			referenceValue = Get_Reference();
			lowMeas = Get_Measurement(0);	// read accelerometer attached to wheel
			hiMeas = Get_Measurement(1);	// read accelerometer attached to chassis
			
			measurementValue = hiMeas - lowMeas; //Get_Measurement(0);

			inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

			Set_Input(inputValue);

			gFlags.pidTimer = FALSE;
		}
	}
}

/* Init */
void Init(void)
{
	/* PID init */
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

	// Set up timer, enable timer/counter 0 overflow interrupt
	TCCR0 = (1<<CS00);
	TIMSK = (1<<TOIE0);
	TCNT0 = 0;
	
	/* Load stored K values */
	/* Get_Measurement() returns a 16-bit int between 0-1023, so this must be scaled to a float between 0-1 */
	K_P = (float)Get_Measurement(3) / 1024;
	K_I = (float)Get_Measurement(4) / 1024;
	K_D = (float)Get_Measurement(5) / 1024;
	REF_VAL = (float)Get_Measurement(6) / 1024;
	
	/* ADC */
	// set micro to use VCC with external decoupling cap as reference voltage
	ADMUX = (1<<REFS0);
	
	// PWM -- initialize PWM1A
	pinMode(19, OUTPUT); // arduino library function
	pwmFreq(19, 2000); // Testing
	pwmDuty(19, 128); // Testing

	// set to approx 93.75kHz (with a 12 meg crystal on ousb): division factor of 128
	ADCSRA= (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);

	// This could be re-thought - decided to throw port inits in here
	PORTA= 0x00;	// turn off PORTA pull-ups
	DDRA = 0x00;	// all port A inputs
	PORTB= 0x00;	// all PORTB outputs low
	DDRB = 0xFF;	// all port B outputs
	PORTC= 0xFF;	// all PORTC pull-ups on
	DDRC = 0x00;	// all port C inputs
	
	TCCR0 |= (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS00); // x1101001

	// OC0 shares function with PB3, so it must be set to output to get a result
	//DDRB |= (1<<PB3); // set OC0 to output
}

/* Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void Set_Input(int16_t inputValue)
{
	/*
	* 	Since speed of the motor is proportional to PWM frequency, speed specified in Hz
	* 	Arbitrary values used here
	*/

	int16_t maxSpeed = 9000;
	int16_t minSpeed = 0;
	int16_t maxPID = 128;	// no idea what the max correction value is yet
	int16_t freqRange, newFreq;

	// Find the available range between low and high speed
	freqRange = maxSpeed - minSpeed;

	// scale PID_input so it fits in the frequency range, and add the value of MIN_SPEED to place it in range
	newFreq = ( ( (float)inputValue / (float)maxPID ) * freqRange ) + minSpeed;

	//set_PWM_freq( new_Freq );
	// NEED TO SET FREQUENCY HERE

	OCR0 = 128; // set duty cycle, 0-255 (255 = 100%)
}

/* Read system process value
 *
 * This function must return the measured data
 */
int16_t Get_Measurement(uint8_t ch)
{
	// select adc channel (0-7)
	ch = ch & 7;	// bit masking just ensures that channel can never be > 7
	ADMUX |= ch;

	// start a conversion by setting ADSC bit in ADCSRA
	ADCSRA |= (1<<ADSC);

	// wait for it to complete: ADIF bit gets set when conversion is complete
	// ASM equiv: sbis	ADCSR, ADIF
	while (!(ADCSRA & (1<<ADIF))) {};

	// clear ADIF
	// From the datasheet i thought this happened automatically, but perhaps not...
	ADCSRA |= (1<<ADIF);
          
	return ADC;
	//return 4;
}

/* Read reference value.
 *
 * This will be set by trimpot or button
 * 
 */
int16_t Get_Reference(void)
{
	return REF_VAL * SCALING_FACTOR;
}

/* Timer interrupt to control the sampling interval */
//#pragma vector = TIMER0_OVF_vect

void TIMER0_OVF_ISR( void )
{
	static uint16_t i = 0;
	
	if (i < TIME_INTERVAL)
	{
		i++;
	}
	else
	{
		gFlags.pidTimer = TRUE;
		i = 0;
	}
}