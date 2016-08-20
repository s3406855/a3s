#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "pid.h"

/* P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 *
 * For Now - hard coded. Ideally these would be stored in flash and modified 
 * by way of buttons or trimpots
 */

#define K_P     1.00
#define K_I     0.00
#define K_D     0.00

/* Flags for status information */
struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t dummy:7;
} gFlags = {0, 0};

//! Parameters for regulator
struct PID_DATA pidData;

/* Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz] ) / 255
 */
#define TIME_INTERVAL   157

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

/* Init of PID controller demo */
void Init(void)
{
	/* PID Related */
	pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);

	// Set up timer, enable timer/counter 0 overflow interrupt
	TCCR0 = (1<<CS00);
	TIMSK = (1<<TOIE0);
	TCNT0 = 0;
	
	/*
	* ADC
	*/
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
	DDRB |= (1<<PB3); // set OC0 to output
}

void Init_ADC()
{
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
}

void Init_PWM()
{
	TCCR0 |= (1<<WGM00) | (1<<WGM01) | (1<<COM01) | (1<<CS00); // x1101001

	// OC0 shares function with PB3, so it must be set to output to get a result
	DDRB |= (1<<PB3); // set OC0 to output
}

/* Read reference value.
 *
 * This function must return the reference value.
 * May be constant or varying
 */
int16_t Get_Reference(void)
{
  return 0;
}

/*! \brief Read system process value
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

/* Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void Set_Input(int16_t inputValue)
{
	uint8_t duty_cycle = 0x00;
	float scaled = 0.0;
	uint8_t adjust = 46; // adjust hard-coded for now - should be made adjustable by trimpot or something

	scaled = ((float)inputValue / (float)1023) * 255;
	duty_cycle = (uint8_t)scaled; // scale to 8 bits
	OCR0 = duty_cycle + adjust; // set duty cycle, 0-255 (255 = 100%)
}

int main(void)
{
	int16_t referenceValue, measurementValue, inputValue;
	Init();
	Init_ADC();
	Init_PWM();

	sei();	// re-enable global interrupts

	while(1)
	{
		// Run PID calculations once every PID timer timeout
		if(gFlags.pidTimer)
		{
			referenceValue = Get_Reference();
			measurementValue = Get_Measurement(0);

			inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

			Set_Input(inputValue);

			gFlags.pidTimer = FALSE;
		}
	}
}

