Arduino Compatible Libraries
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This folder contains Arduino compatible libraries.  There is good information on how to use these library functions at-

http://arduino.cc/en/Reference/HomePage#.UyFS2Zhih58

VERSION- 20140315A




CHANGES MADE
~~~~~~~~~~~~~
This section lists changes that had to be made to the Makefile or Arduino code.

 * The coff converter could not cope with integer constants.
   As the coff file is not needed coff conversion has been disabled.
   The program avr-nm is now used to make a sym file.
   
 * The pin to value arrays in pins_arduino.c are 0 numbered but the pins 
   start at 1 and pick up the next bit.  Had to add --pin; to the start of
   each function in wiring_digital.c that used pin as a parameter.

 * pins_arduiono.c had to add (uint16_t) in front of items such as &DDRB  

 * In Arduiono.h commented out the #define for PE as it conflicted with 
   something else.  Also commented out PF to PL.
   
 * Added many routines to wiring_digital.c, function prototypes added to
   Arduino.h
   
