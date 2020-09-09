/*
Ports used:

PORTG = ADC control
PORTL = ADC input
PORTA = DAC data bus
PORTK = DAC control
pin A6 = potentiometer input
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdlib.h>

#define setbit(port, bit) (port) |= (1 << (bit))
#define clearbit(port, bit) (port) &= ~(1 << (bit))

#define portPin A6

volatile byte input;              //variable to store ADC reading
volatile byte output;             //variable to store DAC output
volatile int pot;                 //variable to store potentiometer reading
volatile int td;                  //time delay between consecutive speakers (interupt cycles)

byte buffer[2048];
int new_data = 0;                 //Pointer to the current spot in the buffer to put the newly measured audio data
int out = 1024;                   //Pointer to the least-delayed output value - ensures that the currently used datapoint is far away from the updated datapoint

void setup() {
  //setup ADC connection
  DDRL = 0x00;                    //set pins C0-C7 as input from ADC
  input = 0;                      //set input to 0
  DDRG = B00000111;               //set pins G0-G3 to output for ADC control
  clearbit(PORTG,0);              //Set initial values of PORTG, CONVST LOW, CS and RD HIGH
  setbit(PORTG,1);
  setbit(PORTG,2);
  
  //setup DAC connections
  DDRA = 0xFF;                    //DAC data bus
  DDRK = B00111111;               //DAC control register (*,*,BUSY,DAC1,DAC2,DAC3,A1,A0)
  output = 0;
  PORTA = output;
  PORTK = B00011100;              //set initial register to OUTA, all DAC WR pins HIGH
  
  
  cli();                          //stop interrupts
  //set timer1 interrupt at ~44.1kHz
  TCCR1A = 0;                     // set entire TCCR1A register to 0
  TCCR1B = 0;                     // same for TCCR1B
  TCNT1  = 0;                     //initialize counter value to 0
  // set compare match register for 44.1kHz increments
  OCR1A = 361;                    // = (16*10^6) / (44100*1) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);         // turn on CTC mode
  TCCR1B |= (1 << CS10);          // Set CS10 bit for 1 prescaler
  TIMSK1 |= (1 << OCIE1A);        // enable timer compare interrupt
  sei();			    //enable interrupts
}

ISR(TIMER1_COMPA_vect){           //timer1 interrupt ~44.1kHz  (it is really 44.199kHz)
  
  setbit(PORTK,5);                //Set BUSY pin to HIGH to indicate beginning of interrupt
  
  /* ADC Control sequence:
  -CONVST goes HIGH then LOW, initiate a conversion
  -Wait 5 microseconds for confersion to finish
  -Bring CS LOW
  -Bring RD LOW, loading the data on the bus
  -Read the Parallel ports C
  -Bring CS and RD back HIGH
  
  PORTG0 = CONVST
  PORTG1 = CS
  PORTG2 = RD
  */
  setbit(PORTG,0);
  clearbit(PORTG,0);
  delayMicroseconds(5);
  clearbit(PORTG,1);
  clearbit(PORTG,2);
  input = PINL;
  setbit(PORTG,2);
  setbit(PORTG,1);
  buffer[new_data&0x07FF] = input;      // store ADC input in the next place in the buffer
  new_data++;                           // increment pointer new_data, so the next interrupt will write to the next entry in the buffer
  
/*
  Speaker connections
  DAC1 OUTA = 1       -PORTK = (0,0,1,WR,1,1,0,0)
  DAC1 OUTB = 2       -PORTK = (0,0,1,WR,1,1,0,1)
  DAC1 OUTC = 3       -PORTK = (0,0,1,WR,1,1,1,0)
  DAC1 OUTD = 4       -PORTK = (0,0,1,WR,1,1,1,1)
  DAC2 OUTA = 5       -PORTK = (0,0,1,1,WR,1,0,0)
  DAC2 OUTB = 6       -PORTK = (0,0,1,1,WR,1,0,1)
  DAC2 OUTC = 7       -PORTK = (0,0,1,1,WR,1,1,0)
  DAC2 OUTD = 8       -PORTK = (0,0,1,1,WR,1,1,1)
  DAC3 OUTA = 9       -PORTK = (0,0,1,1,1,WR,0,0)
  DAC3 OUTB = 10      -PORTK = (0,0,1,1,1,WR,0,1)
  DAC3 OUTC = 11      -PORTK = (0,0,1,1,1,WR,1,0)
  DAC3 OUTD = 12      -PORTK = (0,0,1,1,1,WR,1,1)

  DAC output sequence
  Bitwise &0x07FF makes pointers loop around the buffer of length 2048.
  
  If (td == 0)  ==> all outputs are buffer[out&0x07FF];
  
  If (td != 0)  ==> speaker 1 buffer[out]
                ==> speaker 2 buffer[(out+td)&0x07FF]
                ==> speaker 3 buffer[(out+2*td)&0x07FF]

*/

  //DAC1
  clearbit(PORTK,0);            // set values of A0 and A1
  clearbit(PORTK,1);
  PORTA = buffer[out&0x07FF];   // set correct output for speaker 1 to the data bus
  clearbit(PORTK,4);            // pull WR of DAC1 LOW to make OUTA transparent to the data bus
  clearbit(PORTK,4);            // duplicate to take enough time for WR to be low, but quicker than delayMicroseconds(1)
  setbit(PORTK,4);              // set WR of DAC1 back to logical HIGH to latch OUTA
  
  setbit(PORTK,0);
  clearbit(PORTK,1);
  PORTA = buffer[(out+td)&0x07FF];
  clearbit(PORTK,4);
  clearbit(PORTK,4);
  setbit(PORTK,4);
  
  clearbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+2*td)&0x07FF];
  clearbit(PORTK,4);
  clearbit(PORTK,4);
  setbit(PORTK,4);
  
  setbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+3*td)&0x07FF];
  clearbit(PORTK,4);
  clearbit(PORTK,4);
  setbit(PORTK,4);
  
  //DAC2
  clearbit(PORTK,0);
  clearbit(PORTK,1);
  PORTA = buffer[(out+4*td)&0x07FF];
  clearbit(PORTK,3);
  clearbit(PORTK,3);
  setbit(PORTK,3);
  
  setbit(PORTK,0);
  clearbit(PORTK,1);
  PORTA = buffer[(out+5*td)&0x07FF];
  clearbit(PORTK,3);
  clearbit(PORTK,3);
  setbit(PORTK,3);
  
  clearbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+6*td)&0x07FF];
  clearbit(PORTK,3);
  clearbit(PORTK,3);
  setbit(PORTK,3);
  
  setbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+7*td)&0x07FF];
  clearbit(PORTK,3);
  clearbit(PORTK,3);
  setbit(PORTK,3);
  
  //DAC3
  clearbit(PORTK,0);
  clearbit(PORTK,1);
  PORTA = buffer[(out+8*td)&0x07FF];
  clearbit(PORTK,2);
  clearbit(PORTK,2);
  setbit(PORTK,2);
  
  setbit(PORTK,0);
  clearbit(PORTK,1);
  PORTA = buffer[(out+9*td)&0x07FF];
  clearbit(PORTK,2);
  clearbit(PORTK,2);
  setbit(PORTK,2);
  
  clearbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+10*td)&0x07FF];
  clearbit(PORTK,2);
  clearbit(PORTK,2);
  setbit(PORTK,2);
  
  setbit(PORTK,0);
  setbit(PORTK,1);
  PORTA = buffer[(out+11*td)&0x07FF];
  clearbit(PORTK,2);
  clearbit(PORTK,2);
  setbit(PORTK,2);
  
  out++;                        // Increment out so the next interrupt will read the next datapoint
  clearbit(PORTK,5);            // Clear BUSY pin to indicate end of interrupt
}


//Loop only reads input from the potentiometer to set the time delay
void loop() {
  pot = analogRead(portPin);            //Set value of time delay to one of 9 predefined values based on potentiometer reading.
  if(pot<100) td = -50;
		else if (pot < 200) td = -37;
		else if (pot < 300) td = -25;
		else if (pot < 400) td = -12;
		else if (pot < 600) td = 0;
		else if (pot < 700) td = 12;
		else if (pot < 800) td = 25;
		else if (pot < 900) td = 37;
		else td = 50;
}
