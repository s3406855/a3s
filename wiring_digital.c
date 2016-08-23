/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  Modified 28 September 2010 by Mark Sproul

  $Id: wiring.c 248 2007-02-03 15:36:30Z mellis $
*/

#define ARDUINO_MAIN
#include <util/delay.h>
#include "wiring_private.h"
#include "pins_arduino.h"


void pinMode(uint8_t pin, uint8_t mode) //====================================
{       --pin ; // make zero numbered.
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *reg, *out;

	if (port == NOT_A_PIN) return;

	// JWS: can I let the optimizer do this?
	reg = portModeRegister(port);
	out = portOutputRegister(port);

	if (mode == INPUT) { 
		uint8_t oldSREG = SREG;
                //cli();
		*reg &= ~bit;
		*out &= ~bit;
		SREG = oldSREG;
	} else if (mode == INPUT_PULLUP) {
		uint8_t oldSREG = SREG;
                //cli();
		*reg &= ~bit;
		*out |= bit;
		SREG = oldSREG;
	} else {
		uint8_t oldSREG = SREG;
                //cli();
		*reg |= bit;
		SREG = oldSREG;
	}
}

// Forcing this inline keeps the callers from having to push their own stuff
// on the stack. It is a good performance win and only takes 1 more byte per
// user than calling. (It will take more bytes on the 168.)
//
// But shouldn't this be moved into pinMode? Seems silly to check and do on
// each digitalread or write.
//
// Mark Sproul:
// - Removed inline. Save 170 bytes on atmega1280
// - changed to a switch statment; added 32 bytes but much easier to read and maintain.
// - Added more #ifdefs, now compiles for atmega645
//
//static inline void turnOffPWM(uint8_t timer) __attribute__ ((always_inline));
//static inline void turnOffPWM(uint8_t timer)
/*
static void turnOffPWM(uint8_t timer) //======================================
{
	switch (timer)
	{
		#if defined(TCCR1A) && defined(COM1A1)
		case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
		#endif
		#if defined(TCCR1A) && defined(COM1B1)
		case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
		#endif
		
		#if defined(TCCR2) && defined(COM21)
		case  TIMER2:   cbi(TCCR2, COM21);      break;
		#endif
		
		#if defined(TCCR0A) && defined(COM0A1)
		case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
		#endif
		
		#if defined(TIMER0B) && defined(COM0B1)
		case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2A1)
		case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
		#endif
		#if defined(TCCR2A) && defined(COM2B1)
		case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
		#endif
		
		#if defined(TCCR3A) && defined(COM3A1)
		case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3B1)
		case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
		#endif
		#if defined(TCCR3A) && defined(COM3C1)
		case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
		#endif

		#if defined(TCCR4A) && defined(COM4A1)
		case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
		#endif					
		#if defined(TCCR4A) && defined(COM4B1)
		case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
		#endif
		#if defined(TCCR4A) && defined(COM4C1)
		case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
		#endif			
		#if defined(TCCR4C) && defined(COM4D1)
		case TIMER4D:	cbi(TCCR4C, COM4D1);	break;
		#endif			
			
		#if defined(TCCR5A)
		case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
		case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
		case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
		#endif
	}
}
*/

void digitalWrite(uint8_t pin, uint8_t val)  //===============================
{       --pin ; // make zero numbered.
	//uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	//if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	uint8_t oldSREG = SREG;
	//cli();

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	SREG = oldSREG;
}

uint8_t digitalRead(uint8_t pin) //===============================================
{       --pin ; // make zero numbered.
	//uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	//if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}


void portWrite( uint8_t port, uint8_t value) //==============================
{ 
   volatile uint8_t *out, *reg;
   reg = portModeRegister(port);
   *reg = 255 ;                     // make all pins output.
   out = portOutputRegister(port);
   *out = value ;                   // do the output.  
}


uint8_t portRead( uint8_t port) //==========================================
{ 
   volatile uint8_t *out, *reg;
   reg = portModeRegister(port);
   *reg = 0 ;                     // make all pins input.
   reg = portOutputRegister(port) ;
   *reg = 0 ;                     // set pullups off.
   out = portInputRegister(port);
   return(*out) ;                 // do the input.
}

uint8_t portReadPullup( uint8_t port) //====================================
{ 
   volatile uint8_t *out, *reg;
   reg = portModeRegister(port);
   *reg = 0 ;                     // make all pins input.
   reg = portOutputRegister(port) ;
   *reg = 0xFF ;                  // set pullups on.
   out = portInputRegister(port);
   return(*out) ;                 // do the input.
}


void pwmFreq(int pin, uint32_t frequency) //=============================
{//--- 
   uint32_t fPCK = F_CPU ;
 //--- abort if not a timer pin.       
   if (   (pin != PIN_PWM0) 
       && (pin != PIN_PWM1A) 
       && (pin != PIN_PWM1B)
       && (pin != PIN_PWM2)
      )  
      return ;
           
 //--- frequency of zero means disable and stop.   
   if(frequency == 0)
     {  if(pin == PIN_PWM0)
                TCCR0 = 0;
        else if( pin==PIN_PWM2)
                TCCR2 = 0;
        else {  TCCR1A = 0; // must be pwm1
                TCCR1B = 0;
             }
        return ;
     }
          
 //--- want to set the frequency, try for pwm0 or 2.
   if( (pin == PIN_PWM0) || (pin == PIN_PWM2) )
     {// pwm 0 and 2 can only take on fixed frequencies, round to the nearest.
        uint8_t  divider = 0; 
        uint16_t fdivider;
        if(frequency > 26367){
                divider = 1; //46875
                fdivider = 1;}
        else if(frequency > 3296){
                divider = 2; //5859
                fdivider = 8;
        }
        else if(frequency > 458){
                divider = 3; //732
                fdivider = 64;
        }
        else if(frequency > 114){
                divider = 4; //183
                fdivider = 256;
        }
        else{
                divider = 5; //46
                fdivider = 1024;
        }
        if(pin == PIN_PWM0)
              TCCR0 = 0x48 | divider; //0x48 = WGM1 + WGM0
        else  TCCR2 = 0x48 | divider; //0x48 = WGM1 + WGM0

        return ; //( fPCK/256)/fdivider;
     }
   
 //--- must be setting freq for pwm1  
   // get expected values into regs assuming /1 prescalar.
   uint32_t top ;
   uint8_t  tmp_TCCR1A, tmp_TCCR1B;
   tmp_TCCR1A = TCCR1A ;
   tmp_TCCR1B = TCCR1B ;
   TCCR1A = (tmp_TCCR1A & 0xF0) | 0x02;  //set WGM11
   TCCR1B = (tmp_TCCR1B & 0xE0) | 0x19 ; //set WGM13, WGM12 and CS10

   if (frequency > fPCK/4) // if freq too high limit to top value.
     frequency = fPCK/4 ;

   //--- a fixed divider of 1 gives limits from 183Hz to 4MHz on 12 MHz xtal
   if (frequency < fPCK/65536)
    {//-- try for /256 prescalar not 1.
       TCCR1B = (tmp_TCCR1B & 0xE0) | 0x1C ; //set WGM13, WGM12 and CS12
       if ( frequency < fPCK/65536/256)  // set limit to lowest freq if under that.
         frequency = fPCK/65536/256 ;
       top  = (fPCK/frequency/256 - 1);
       ICR1 = top;
       // do not set COM1A or COM1B bits yet, do that when define pulse width.
       return ; //fPCK/(top+1)/256;  // actual frequency
    }
   else //  /1 prescalar OK.
    {  top  = (fPCK/frequency - 1);
       ICR1 = top ;
       // do not set COM1A or COM1B bits yet, do that when define pulse width.
       return ; //fPCK/(top+1);  // actual frequency
    }   
    
}


void pwmDuty(uint8_t pin, uint8_t duty_cycle) //==========================
{//--- abort if not a timer pin.       
   if (   (pin != PIN_PWM0) 
       && (pin != PIN_PWM1A) 
       && (pin != PIN_PWM1B)
       && (pin != PIN_PWM2)
      )  
      return ;
           
 //--- Enable PWM outputs
   if(pin == PIN_PWM0)
     { uint8_t tmp_TCCR0 = TCCR0 & 0xCF; //xx00xxxx
       if (duty_cycle > 0)
               tmp_TCCR0 |= 0x20;        //xx10xxxx
       TCCR0 = tmp_TCCR0;
       OCR0  = duty_cycle ;
       pinMode(PIN_PWM0, OUTPUT) ;
       return ;
     }
     
    if(pin == PIN_PWM2)
     { uint8_t tmp_TCCR2 = TCCR2 & 0xCF;         //xx00xxxx
       if (duty_cycle > 0)
               tmp_TCCR2 |= 0x20;        //xx10xxxx
       TCCR2 = tmp_TCCR2;
       OCR2  = duty_cycle ;
       pinMode(PIN_PWM2, OUTPUT) ;
       return ;
     }
     
  //--- must be pwm1a or pwm1b which can have different duty cycles.
    uint8_t tmp_TCCR1A = TCCR1A ;
    if(pin == PIN_PWM1A)
         {  tmp_TCCR1A &= 0x3F;       //00xxxxxx
            if(duty_cycle > 0)
                tmp_TCCR1A |= 0x80;   //10xxxxxx
            pinMode(PIN_PWM1A, OUTPUT) ;
         }
    else {//pin == PIN_PWM1B
            tmp_TCCR1A &= 0xCF;       //xx00xxxx
            if(duty_cycle > 0)
                tmp_TCCR1A |= 0x20;   //xx10xxxx
            pinMode(PIN_PWM1B, OUTPUT) ;
         }
    TCCR1A = tmp_TCCR1A;

    uint32_t tmp_OCR = (ICR1*(uint32_t)duty_cycle)/255 ; 
    if(pin == PIN_PWM1A)
         OCR1A = tmp_OCR ;     
    else OCR1B = tmp_OCR ;

}


//************************ POLLED SERIAL TX RX *******************************

//------ Links between stdio system and the uart tx/rx
void uart_putchar( uint8_t tx_byte) ;              // prototypes.
uint8_t uart_getchar() ;

int usart_putchar_printf(char var, FILE *stream)   // routine that gets called
{ uart_putchar(var);                               //   by printf
  return 0;
}

int usart_getchar_scanf(FILE *stream)              // routine called by scanf
{  uint8_t rx_char = uart_getchar() ;
   uart_putchar( rx_char) ;                        // echo back to terminal.
   return rx_char ;
}

// structure for printf to use to get to uart transmit.
static FILE mystdio = FDEV_SETUP_STREAM(usart_putchar_printf, usart_getchar_scanf, _FDEV_SETUP_RW);


void uartInit(uint16_t baud) //==============================================
{  DDRD = (DDRD & 0xFE) | 2 ;                 // RX and TX set to I,O,
   UBRRH =  0 ;                                
   UBRRL =  (750000/ (uint32_t)baud) - 1;        
   UCSRB = (UCSRB | _BV(RXEN) | _BV(TXEN) );  // Enable UART RX and TX. 
   _delay_ms(10) ; // delay to ensure connected receiver recovers from serial break.
   
   stdin = stdout = &mystdio; // setup stdio
}


void uart_putchar( uint8_t tx_byte) //========================================
{    while ( !(UCSRA & (_BV(UDRE))) );       // Wait for empty transmit buffer
     UDR =  tx_byte ;                        // Start transmittion
}


uint8_t uartTXempty() //======================================================
{ if ( UCSRA & (_BV(UDRE)) )
                return( HIGH) ;    // bit high, TX empty.
  else return( LOW) ;       // bit low, TX buffer busy.
}


uint8_t uartRXempty() //======================================================
{ if ( UCSRA & (_BV(RXC)))
       return( LOW) ;   // character waiting, not empty
  else return( HIGH) ;  // no character waiting,  is empty.
}


uint8_t uart_kbhit() //======================================================
{ if ( UCSRA & (_BV(RXC)))
       return( HIGH) ;  // character ready
  else return( LOW) ;   // no character waiting,  is empty.
}


uint8_t uart_getchar() //=====================================================
{ while ( !(UCSRA &  (_BV(RXC))) ); //  Wait for incoming data
  return UDR; 
}

void uartStringSend( char *str) //==========================================
{ while ( *str != (char) 0 )
   { uart_putchar( *str) ;
     ++str ;
   }  
}




