	// definitions for global use //

	#ifndef __adctodaclib_h
	#define __adctodaclib_h

	// everything needed for the M7 STDLIB
	#include <stm32f7xx.h> // allow processor specific nonstandard commands
	#include <stdlib.h>// calloc()/ malloc
	#include <stdbool.h> // booleans

	// MATH Libraries for DSP
	#include "sin.h"// sin 

	// GLOBAL DEFINES //

	#define ADCB 0x40012200// base address for ADC3, the main ADC we will be using
	#define ADC1B 0x40012000
	#define ADCDataOffset 0x13
	#define GPIOBa 0x40020000

	#define PWRB 0x40007000
	#define FLASHB 0x40023c00
	#define RCCB 0x40023800
	#define DACB 0x40007400
	#define DMA2B 0x40026400
	#define TIM6B 0x40001000

	// CUSTOM TYPES FOR CLARITY //
	typedef unsigned char byte;
	typedef unsigned char bitnum;
	typedef unsigned volatile int* const address;
	
	
	// PROTOTYPES //
	// BIT OPERATIONS //
	static inline void set(address adr,bitnum bit); // sets a bit at the address with bitnumber bit
	static inline void reset(address adr, bitnum bit); // resets a bit at theaddress adr with bitnumber bit

	// READING FUNCTIONS //
	static inline volatile bool readbit(address adr, bitnum bit); // reads a bit and gives a logical value at the address and bitnumber

	// WRITING FUNCTIONS //
	static inline void write(address adr,bitnum start, byte length, unsigned int value);// writes a chunk of length length into address adr starting at bitnumber start
	static inline void zero(address adr, bitnum start, bitnum length);// zeros a chunk of bits at adr starting at start and length long
	static inline unsigned int mask(bitnum start, bitnum length);


	// APPLIED FUNCTIONS //

	// DAC Functionality //
	static void DACconfig(void);// init
	static inline void writeDAC(unsigned short value);// writes to the dac data reg and sets a software trigger
	static inline void startDAC(void);// enable DAC channel 1
	static short discretesin( unsigned short amplitude, float frequency, float n); // generates an output value

	// DMA Functionality //
	static void DMAconfig(void);
	static inline byte DMADataLeft(void);// number of items left to transfer
	static inline void startDMA(void); // enable the start bit of the DMA config registers
	static inline void stopDMA(void); // disable start bit
	static inline bool DMADone(void);// check if DAC has finished transfer
  static inline bool buffer(void);

	// GPIO Functionality //
	static void GPIOconfig(void);
	static inline void setLED(byte LED);
	static inline void resetLED(byte LED);
	static inline bool button(void);

	//Timer Functionality//
	static void TIMERconfig(void); // init
	static inline void startTIMER(unsigned short scalar);   // start the timer with a prescaler
	static inline void stopTIMER(void); // turn off clock enable
	static void syncTIMER(void); // sets up the timer so that it syncs with the pulses

	// ADC functionality //
	void ADCconfig(void);// config for disc triggered setup
	void fastADCConfig(byte adc);// fast configuration setup
	static inline void startADC(byte adc);// starts one of the ADCs
	static inline void stopADC(byte adc); // stops one of the ADCs
	static inline unsigned short readADC(byte adc);// reads the corresponding data register 
	static inline volatile bool EOC(byte adc); // checks EOC flag of ADC
	static inline volatile bool AWD(byte adc); // checks the watchdog flag
	
	//Called on Startup//
	static void startup(void);// start everything
	static void init(void);// init clocks

	// MAIN INTERNAL FUNCTIONS //
	static void setup(void);// runs startup functions
	__attribute__((noreturn)) static void loop(void);// writes to DAC continuously, does not return
	
	// interrupt routines //
	void ADC_INT(void);



	#endif
	
	
	