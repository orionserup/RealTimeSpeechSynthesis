// DESIGNED AND WRITTEN BY ORION SERUP
// ADVANCED BIONICS PROPRIETARY SOFTWARE

#include "adctodaclib.h"   // library for this file

// LOCAL USEFUL PERIPHERAL ADDRESSES //
static address ADC3Base = (unsigned int*)ADCB;
static address ADC1Base = (unsigned int*)ADC1B;
static address PWRBase = (unsigned int*)PWRB;
static address DACBase = (unsigned int*)DACB;
static address DMA2Base = (unsigned int*)DMA2B;
static address TIM6Base = (unsigned int*)TIM6B;
static address RCCBase = (unsigned int*)RCCB;
static address GPIOBase = (unsigned int*)GPIOBa;
static address FLASHBase = (unsigned int*)FLASHB;

static unsigned short* DATABUFF = NULL;   // default the value so that it can be written to whatever
static unsigned short* DATABUFF1 = NULL;

// FITTING PARAMETERS //

#define T 28
#define M 282
#define R 10000
#define PW 10.8f

static const float VREF = 3.3f;                // the external reference voltage for the ADC/DAC
static unsigned short MINVOLTAGE = 0;       // minimum voltage for ADC threshold, related to T and R used
static unsigned short MAXVOLTAGE = 0xfff;       // same story except max

static unsigned short MINV6B  = 0xf;     // noise threshold used for timing 

// global variable for the order of channel reading
static const byte channelorder[16] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};

// What channels to output // 
static const byte output[] = {3, 4};

// the corresponding frequncies for each bin in kHz
static float frequencies[16] = {.382f, .453f, .538f, .640f, .760f, .902f, 1.1f, 1.3f, 1.5f, 1.8f, 2.1f, 2.5f, 3.0f, 3.6f, 4.2f, 6.8f}; 

// clock speeds for sampling and outputting in KHz
static float ADCClock;


//      MAIN      //
int main(){
    
	setup();   // executes once
  loop();    // continuously runs

}
//       END MAIN     //

//     MAIN FUNCTIONS    //

void setup(){       // runs once
	
    init();        // enables everything and sets up clock domains and timers
    syncTIMER();  // sync the ADCs to the pulse width
	
	// configure peripherals 
    DMAconfig();
    ADCconfig();
    DACconfig();
		
		// start peripherals
		startDAC();
		startADC(3);  // put ADC3 on standby
		startDMA();
	
		unsigned int counter;
		unsigned short prescaler;
	
		#ifdef PW
	
		counter = (unsigned int)PW*1.5f;
		prescaler = (unsigned short)27.0f*PW;
		
		#else
		
		counter = (unsigned int) 750.f/ADCClock;
		prescaler = (unsigned int)216.0f/ADCClock;
		prescaler *= 1000;
		
		#endif
		
		bool hit = false;  // flag
		bool ADCHIGH = true;
	
		startADC(2);
		
		while(counter > 0){  // down count into the center of the waveform
			
			if(EOC(2)){
				
				ADCHIGH = readADC(2) >= MINV6B;
				
				if(!ADCHIGH) hit = true;   // if its low mark the flag
				else if(hit) counter--;  // if there was a low and its high downcount
				
			}
		}
		stopADC(2);  // stop the fast ADC
		startTIMER(prescaler);  // get the timer going with the prescaler
		
		setLED(1);
		
		return;
}

void loop(){ 
		
		unsigned short* data;  // buffer select variable
    unsigned short val = 0x800;  // center value output value to dac
		unsigned long int ticks = 0;  // discrete time counter
		bool whichbuffer = buffer();
		
		for(int i = 0; i < 16; i++) frequencies[i] /= ADCClock;  // divide the frequencies so that its faster with sin
	
		while(true) if(buffer() != whichbuffer) break; // if the dma is done with a buffer break
			
		while(true){
			
			if(DMADone()){  // if ready to be read

				data = buffer()? DATABUFF: DATABUFF1; // pick the the non used buffer to print
				
				for(byte i = 0; i < sizeof(output) ; i++){
					val += discretesin(0xfff, frequencies[output[i]], ticks); // continuously generates all of the sine curves as fast as possible
				}
		
        writeDAC(val);  // insert the calculated value with a base at halfway
				
				val = 0x800;
				ticks++;
		}
	}
}

//   END MAIN FUNCTIONS   //

//   SETUP FUNCTIONS   //
void init(){
	
    while(1) if(RCCBase[0] & 0x2) break; // wait for HSI to be ready
		
		set(&PWRBase[0], 4); // turn on PVD
		write(&PWRBase[0], 14, 2, 0x01); // set power divider for 1: HIGH POWER
		write(&PWRBase[0], 5, 3, 0x7); // high voltage threshold
		zero(&PWRBase[0], 18, 2); 
		
		write(&FLASHBase[0], 0, 4, 0x8); // set the FLASH latency to 8 wait states due to fast clock
		set(&FLASHBase[0], 8); // enable prefetch stage
		
		reset(&RCCBase[1], 22); // set HSI as the PLL input
    write(&RCCBase[1], 6, 9, 216); // set the multiplier for 432
    zero(&RCCBase[1], 16, 2); // set the main divider for four
    write(&RCCBase[1], 0, 6, 0x8); // set the reference f to be 2 MHZ
		
		set(&RCCBase[16], 28); // turn on PWR CLOCK
		
		set(&PWRBase[0], 16); // turn on the overdrive mode option
		while(1) if(readbit(&PWRBase[1], 16) == 1) break;  // check if overdrive is ready
		
		set(&PWRBase[0], 17); // switch to overdrive
		while(1) if(readbit(&PWRBase[1], 17) == 1) break; // if switching is done move on
		
		set(&RCCBase[0], 24); // turn on the PLL 
		
    zero(&RCCBase[2], 4, 4); // set AHB speed for 216MHz
    write(&RCCBase[2], 10, 3, 0x4); // set APB2 speed for 108Mhz
    write(&RCCBase[2], 13, 3, 0x5); // set APB1 speed for 54MHz
		
		write(&FLASHBase[0], 0, 4, 0x7); // set the FLASH latency to 7 wait states due to fast clock
		set(&FLASHBase[0], 8); // enable prefetch stage
		
		while(1) if (readbit(&RCCBase[0], 25) == 1) break; // if PLL is ready move on

		write(&RCCBase[2], 0, 2, 0x2); // set system clock for the PLL
		while(1) if( readbit(&RCCBase[2], 1) == 1) break;  // if switched move on

    write(&RCCBase[12], 0, 6, 0x3f); // enable gpio clock for a-f
    set(&RCCBase[12], 22); // enable DMA2
    set(&RCCBase[14], 0); //enable FMC
    set(&RCCBase[16], 29 ); // set DAC clock to on
    set(&RCCBase[17], 14); // turn on syscfg clock
    write(&RCCBase[17], 8, 3, 0x7); // enable ADC Clock
    set(&RCCBase[16], 4); // turn on the timer6 clock
    set(&RCCBase[29], 24); //clear flags

		write(&RCCBase[4], 0, 6, 0x3f); // resets all GPIOs
    set(&RCCBase[4], 22);  // reset DMA
    set(&RCCBase[8], 29); // resets the DAC
    set(&RCCBase[9], 8); // reset ADC
		set(&RCCBase[8], 4);  // reset timer 6
	
	  write(&RCCBase[4], 0, 7, 0x00); // resets all GPIOs
    reset(&RCCBase[4], 22);  // reset DMA
    reset(&RCCBase[8], 29); // resets the DAC
    reset(&RCCBase[9], 8); // reset ADC
    reset(&RCCBase[8], 4); // reset timer6
		
		set(&RCCBase[35], 24); // set the timer clock to AHB Clock (216 MHz)
		write(&ADC3Base[0x26], 16, 2, 0x1); // sets the adc clock speed to 27Mhz

    TIMERconfig();   // set up the timer for timing the ADC
    GPIOconfig();   // set up the GPIOs
		
		return;
}

void syncTIMER(){
	
		fastADCConfig(2);
	
#ifdef PW  // if we have a pulsewidth already dont find it
	
		ADCClock = 500.0f/PW;  // ADCCLK in KHz
  
#else
    
    byte hit = 0;  // flag
    unsigned short ADCVAL = (unsigned short)0x03f;  // set val to max
	
    // start ADCs to scan as fast as possible in triple mode
		write(&ADC3Base[0x26], 0, 5, 0x17); // turn on interleaved mode for triple ADC
    write(&ADC3Base[0x26], 8, 4, 1); // set the time between sampling ot be 6ADCCLK
    
		fastADCConfig(1);
		fastADCConfig(3);
	
		startADC(1);
		startADC(2);
		startADC(3);
    
    // when capture the length of a waveform
    while(1){
			
					if(EOC(1)) ADCVAL = readADC(1);
					else if(EOC(2)) ADCVAL = readADC(2);
					else if(EOC(3)) ADCVAL = readADC(3);
            
            if((ADCVAL < MINV6B)){
                hit = 1;
                if(( counter > 0)){
                    stopADC(1);            // turn off readings, its work has been done
                    stopADC(3);
										stopADC(2);
                    break;
                }
            }
            else if(ADCVAL > MINV6B) {
                if(hit == 1) counter++;
            }
        }
		
    ADCClock = 4500.0f/counter;   // change the global frequency variables so they line up
    
	#endif
	
	return ;
}

//   END SETUP FUNCTIONS   //

//   LOOP FUNCTIONS   //

// outputs the value of the discrete sine
short discretesin(unsigned short amplitude, float frequency, float n){        
		unsigned short amp = amplitude >> 5;       // divide amplitude by 32 so that 16 channels at max fill up the DAC
    float phase = frequency*n;  // discrete phase representation of (wt)
		return (short)((float)amp*sin_f32(phase));  // combine and output
}

//   END LOOP FUNCTION   //

//   HELPER FUNCTIONS   //
// Configurations //
void DMAconfig(){ // for DMA2 (ADC to RAM
	
		stopDMA();
	
    write(&DMA2Base[4], 25,3,0x2);  // channel two selection   (ADC3)
    zero(&DMA2Base[4], 23, 2);   // single transfer operations
    zero(&DMA2Base[4], 21, 2);  // single transaction operation for periphs
    write(&DMA2Base[4], 16, 2, 0x3); // set the priority level high
		set(&DMA2Base[4], 18); // turn on double buffer mode
	
		set(&DMA2Base[4], 19); // turn on DATABUFF1 as first target
	
		set(&DMA2Base[4], 8); // turn on circular mode	
	
    reset(&DMA2Base[4], 15); // set variable periph mem size
	
    write(&DMA2Base[4], 13, 2, 0x1); // set memory data to 16 bits
    write(&DMA2Base[4], 11, 2, 0x1); // set periph mem size to 16
	
    set(&DMA2Base[4], 10);  // have the DMA do an auto array
	
    reset(&DMA2Base[4], 9); // set the peripheral address as fixed
    zero(&DMA2Base[4], 6, 2); // set mode for peripheral to memory 
    reset(&DMA2Base[4], 5); // set DMA to control the flow
		
    DMA2Base[5] = 0x10 ; // set 16 events in chain
		
		reset(&DMA2Base[9], 2); // turn on direct mode
		
		if(DATABUFF == NULL) DATABUFF = (unsigned short*)calloc(16, sizeof(short)); // dynamically allocate 32 bits of space for the data buffer
		if(DATABUFF1 == NULL) DATABUFF1 = (unsigned short*)calloc(16, sizeof(short));
			
    DMA2Base[6] = (unsigned int)(ADC3Base + ADCDataOffset); // pass ADC3 Data Register as the source address
    DMA2Base[7] = (unsigned int)DATABUFF; // set DATABUFF as the memory location 0
		DMA2Base[8] = (unsigned int)DATABUFF1; // set DATABUFF1 as memory location 1	
		
		return;
}

void GPIOconfig(){
    
    // ANALOG  PA0-3, PC0-3, PF3-10  //
    write(&GPIOBase[0x0], 0, 10, 0x3ff); // set 0,1,2,3 on port a,c to analog and A4 analog for DAC Channel 1
    zero(&GPIOBase[0x3], 0, 8); // no pullup/down 
		
    write(&GPIOBase[0x200], 0, 8, 0xff);
    zero(&GPIOBase[0x203], 0, 8); // no pullup/down
	
    write(&GPIOBase[0x300], 6, 16, 0xffff); // set port f to analog mode
    zero(&GPIOBase[0x303], 6, 16); // no pull up/down
    
    
    // LEDs //
		write(&GPIOBase[0x100], 0, 2, 0x1); // output
		reset(&GPIOBase[0x101], 0); // pushpull mode
		write(&GPIOBase[0x102], 0, 2, 0x3); // fast as possible output
		write(&GPIOBase[0x103], 0, 2, 0x0); // no pull
		
		write(&GPIOBase[0x100], 14, 2, 0x1); // output
		reset(&GPIOBase[0x101], 7); // pushpull mode
		write(&GPIOBase[0x102], 14, 2, 0x3); // fast as possible output
		write(&GPIOBase[0x103], 14, 2, 0x0); // no pull
		
		write(&GPIOBase[0x100], 28, 2, 0x1); // output
		reset(&GPIOBase[0x101], 14); // pushpull mode
		write(&GPIOBase[0x102], 28, 2, 0x3); // fast as possible output
		write(&GPIOBase[0x103], 28, 2, 0x0); // no pull
		
    // BUTTON PC13
    write(&GPIOBase[0x200], 26, 2, 0x0); // input mode
    write(&GPIOBase[0x202], 26, 2, 0x3); // very fast speed
    write(&GPIOBase[0x203], 26, 2, 0); // no pull up/down
		return ;
}

void fastADCConfig(byte adc){
	
    write(&ADC1Base[1 + 0x40*(adc-1)], 24, 2, 0x3); // set it to be 6 bit accuracy for fast timing
		set(&ADC1Base[1 + 0x40*(adc-1)], 23); // enable watchdog

    reset(&ADC1Base[2 + 0x40*(adc-1)], 11); // align data to the right
    set(&ADC1Base[2 + 0x40*(adc-1)], 1);  // turn on continuous converison
		write(&ADC1Base[9 + 0x40*(adc-1)], 0, 6, MINV6B); // set the watchdog to watch for values above MINV6B	
	
		return;
}

void TIMERconfig(){           // Timer two runs at same clk as ADC, so we need to set the clk to count to the number of
    
    reset(&TIM6Base[0], 7);    	// auto reloads timer
    reset(&TIM6Base[0], 1); 	// enable update event
		reset(&TIM6Base[0], 3); 	// dont stop counting on an update event
		reset(&TIM6Base[0], 2);   // UG and counter events trigger ADC
    
    write(&TIM6Base[1], 4, 3, 0b010); // set update as the trigger for ADC
	
		return;
}

void DACconfig(){ // configure channel one of DAC
	
    zero(DACBase, 6, 2); // turn off noise and triangle wave mode
    write(DACBase, 3, 3 ,0x7); // set trigger to software trigger
    set(DACBase, 2); //enable trigger
    set(DACBase, 1); // enable an output buffer
	
		return ;
}

void ADCconfig(){  // set up ADC3 for the timer triggered disc mode with DMA requests and IRQ on Watchdog event
    
    const float voltageconstant = (155.7f*ADCClock*4096*R)/(1000000*VREF); // relationship between cu and V in 12-bit analog form 
    
    MINVOLTAGE = (unsigned short)(T*voltageconstant);
    MAXVOLTAGE = (unsigned short)(M*voltageconstant);
   
    zero(&ADC3Base[0x26], 0, 5); // turn off interleaved mode for triple ADC
    
		set(&ADC3Base[1], 23); // turn on analog watchdog
    set(&ADC3Base[1], 6); // enable watchdog interrupt
    
    NVIC_EnableIRQ(ADC_IRQn);                  // enable ADC Irq
    NVIC_SetPriority(ADC_IRQn, 7);             // set the priority to the highest nonhardware level
    NVIC_SetVector(ADC_IRQn, (unsigned int)&ADC_INT); // turn on the timing protector interrupt

    reset(&ADC3Base[1],26 ); // disables overrun interrupt
    zero(&ADC3Base[1], 24, 2); // set resolution to 12 bits
    set(&ADC3Base[1], 8); // enable scan mode
		set(&ADC3Base[1], 11); // turn on discontinuous mode
		zero(&ADC3Base[1], 13, 3); // one channel per trigger
		
    set(&ADC3Base[2], 9); // keeps DMA on
    set(&ADC3Base[2], 8); // turns DMA requests on 
    
    write(&ADC3Base[2], 28, 2, 0x2); // trigger on rising edge
    write(&ADC3Base[2], 24, 4, 0b1101);  // TIMER6 TRGO set as the trigger
    set(&ADC3Base[2], 10);  // EOC set after conversion and overrun disabled
    reset(&ADC3Base[2], 1);  // turn off continuous converison
    
    write(&ADC3Base[10], 0, 12, 0x000); // set the lowerbound to the minvoltage
    write(&ADC3Base[9], 0, 12, 0xfff); // set the upperbound to the max voltage
		
		// set the scan mode read sequence
    byte i = 0;
    address curraddr = ADC3Base;  // sets a constant address to the ADC we selected

    for( ; i<6; i++) write(&curraddr[13], 5*i, 5, channelorder[i]); // configures first selection register
    for( ;i<12; i++) write(&curraddr[12], 5*i, 5, channelorder[i]);  // configures second selction register
    for( ;i<16; i++) write(&curraddr[11], 5*i, 5, channelorder[i]);   // configures last selection register
		
    write(&curraddr[11], 20, 4, 0xf);  // sets the number of channels
		
		return;
}


// Status Functions //
volatile bool EOC(byte adc){
	return ADC1Base[0x40*(adc-1)] & 0x2;
}

volatile bool AWD(byte adc){
	bool data = ADC1Base[0x40*(adc-1)] & 0x1;
	if(data) ADC1Base[0x40*(adc-1)] &= ~0x1;
	return data;
}

bool buffer() {
		bool read = DMA2Base[4] & (1<<19);
		return read;
}

byte DMADataLeft(){
		 return (byte)DMA2Base[5];
}

bool DMADone(){
		byte read = DMA2Base[0] & (1<<5);

	if(read) DMA2Base[2] |= (1<<5); 
		return read;
}

// Writers and Readers //
unsigned short readADC(byte adc){ 
    return (unsigned short)ADC1Base[0x40*(adc-1) + 0x13];
}

void writeDAC(unsigned short value){ // send data to DAC and put it out
    DACBase[8] = value; // write the data to the data register
    DACBase[1] |= 0x1;          // set a software trigger
		return;
}


// Starters and Stoppers //
void startTIMER(unsigned short scalar){           
	
		TIM6Base[11] = scalar;   // auto reload to one
    TIM6Base[0] |= 0x1; // start the counter
		TIM6Base[5] = 0x1;  // trigger ADC and reset the counter
		return;
}
void stopTIMER(){
		TIM6Base[0] &= ~0x1; // stop the timer
		return;
}

void startADC(byte adc){  // starts the adc
		ADC1Base[0x40*(adc-1) + 2] |= 0x1; // turn on the ADC
		ADC1Base[0x40*(adc-1) + 2]  |= (1<<30); // start regular channel conversion
		return;
}

void stopADC(byte adc){
    ADC1Base[0x40*(adc-1) + 2] &= ~(1<<30); // stop channel conversion
		ADC1Base[0x40*(adc-1) + 2] &= ~0x1; // turn off the ADC
		return;
}

void startDAC(){
		DACBase[0] |= 0x1; // set en bit
		return;
}

void startDMA(){
		DMA2Base[4] |= 0x1;
		return;
}

void stopDMA(){
		DMA2Base[4] &= ~0x1;
		return;
}

// END HELPER FUNCTIONS //

//  GPIO FUNCTIONS  //

void setLED(byte LED){
		set(&GPIOBase[0x105], 7*LED);  // writes the pin to what its not
		return ;
}

void resetLED(byte LED){
		reset(&GPIOBase[0x105], 7*LED);
		return ;
}

bool button(){
		return readbit(&GPIOBase[0x204], 13); // checks the button status
}

// END GPIO FUNCTIONS //

void ADC_INT(){
	
	unsigned int counter = (unsigned int)(750.0f/ADCClock); // half of the count
	unsigned int prescaler = (unsigned int)(216000.0f/ADCClock); // the division factor for the clock
	
	bool hit = false;  // flag
	bool ADCHIGH = true;
	
	stopTIMER();  // stop the things we are trying to configure
	stopDMA();
	
	startADC(2);
	
	while(counter > 0){  // down count into the center of the waveform
		
		if(EOC(2)){
			
			ADCHIGH = readADC(2) >= MINV6B;
			
			if(!ADCHIGH) hit = true;   // if its low mark the flag
			else if(hit) counter--;  // if there was a low and its high downcount
			
		}
	}
	
	startADC(3);         // start the ADC
	DMA2Base[5] = 16;  // 16 items left for the DMA to transfer in this chain
	stopADC(2);  // stop the fast ADC
	startTIMER((unsigned int)(prescaler));  // get the timer going with the prescaler
	startDMA();  // get the dma transfers going
	
	return;
}

/// BASIC FUNCTIONS ///

void reset(address adr, bitnum bit){
		*adr &= ~(1<<bit);  // reset the value at bit
		return;
}

void set(address adr, bitnum bit){
		*adr |= (1<<bit);  // set the value at bit
		return;
}

volatile bool readbit(address adr, bitnum bit){  // returns one or zero
		return (*adr & (1<<bit));
}

void zero(address adr, bitnum start, bitnum length){
		unsigned int val = *adr & mask(start, length);
		*adr = val;
		return;
}
void write(address adr, bitnum start, byte length, unsigned int value){
		unsigned int val = *adr & mask(start, length);  // zero the values
		val |= (value<<start);   // write in the ones
		*adr = val;
		return;
}

unsigned int mask(bitnum start, bitnum length){
		unsigned MASK = (1<<(length + 1))-1;
		return ~(MASK<<start);
}

/// END BASIC FUNCTIONS ///

