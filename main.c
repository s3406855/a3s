/*	TO-DO
*	1. Write interrupt routine to read new values of K_P, K_I, K_D and Reference
*	2. Complete get_from_eeprom() - retrieves stored values
*	3. Complete write_to_eeprom() - stores new values (called during interrupt)
*	4. Add LCD functionality to display K_P, K_I, K_D, Ref during setup and testing
*	5. Investigate how to vary the PWM frequency
*/


/*****	ATMEGA32 SETUP ******
*		ADC (Port A)
*		0 = Accelerometer (Top)
*		1 = Accelerometer (Bottom)
*		2 = K_P
*		3 = K_I
*		4 = K_D
*		5 = Reference
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <avr/eeprom.h>
#include "pid.h"

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
 * Values to be stored in eeprom so that values persist between uses
 * Will be controlled by buttons (ideally) or trimpots, with an interrupt to catch changes in value and store the result
 */

float K_P = 1.00;
float K_I = 0.00;
float K_D = 0.00;
float REF_VAL = 0.0;

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
	K_P = get_from_eeprom('P');
	K_I = get_from_eeprom('I');
	K_D = get_from_eeprom('D');
	REF_VAL = get_from_eeprom('R');
	
	/* ADC */
	// set micro to use VCC with external decoupling cap as reference voltage
	ADMUX = (1<<REFS0);

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

/*
*	Fetches the four variables, P-I-D-REF from flash memory when the program is first run
*
*		P exists at address
*		I exists at address
*		D exists at address
*		R exists at address
*/
float get_from_eeprom(unsigned char a)
{
	//float read = 0.00;
	//const float addr = 0.00;
	
	switch (a)
	{
		case 'P':
			return 1.00;
			//addr = 0xAABB;
			break;
		case 'I':
			return 0.00;
			//addr = 0xAABB;
			break;
		case 'D':
			return 0.00;
			//addr = 0xAABB;
			break;
		case 'R':
			return 0.00;
			//addr = 0xAABB;
			break;
	}
	
	/*
	if (eeprom_is_ready())
	{
		read = eeprom_read_float(addr);
	}*/
	
	return 0.00;
}

void write_to_eeprom(unsigned char a, int16_t val)
{
	;
}