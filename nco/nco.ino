#define FS 20 // should be 44100 <- using 20 Hz to test if NCO working properly (digitally)
uint32_t  DacSample;

int LUT[2048];
bool interruptFlag = false;
unsigned int cycle = 0; // this counter determines the frequency of the waveform, user inputs desired frequency value and it is multiplexed into a positive integer

void setup() {
  
  Serial.begin(9600);
  startTimer(TC1, 0, TC3_IRQn, FS); // TC1 channel 0, the IRQ for that channel and the desired frequency
  DacSetup();
  for(int i=0; i < 2048; i++)
  {
    LUT[i] = (2047*sin(2*PI*i/2048) + 2048); // build lookup table for our digitally created sine wave
  }

}


void loop() {

  while(!interruptFlag) //do nothing until interrupt
  {
    digitalWrite(13, 1); //best practice to do some NOOP task inside empty loop to avoid errors
  }
  
  while(cycle > 2047) //to prevent overflow, reset the cycle counter <- previously if(cycle > 2047){ reset cycle }. Uusing empty while loop to cut off waveform to test results
  {
    digitalWrite(13, 0); //best practice to do some NOOP task inside empty loop to avoid errors
  }
  
/* UNCOMMENT THIS CODE TO TEST NCO DIGITALLY
 
  Serial.println(LUT[cycle]);

*/

  DacSample = LUT[cycle]; // push into DAC
  DACC->DACC_CDR = DacSample;  // Start the next DAC conversion
  cycle++; // increment cycle counter to fetch next value in LUT, the higher the increment value is, the higher the sine wave frequency, counter is incremented by its initial value, for example, iteration1: 2, iteration2: 2+2, iteration3: 4+2, etc.

  interruptFlag = false; // wait for next interrupt
}

void TC3_Handler()
{
  TC_GetStatus(TC1, 0); // accept interrupt
  interruptFlag = true; // interrupt over - send sample to DAC
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t)irq);
  TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);
  uint32_t rc = VARIANT_MCK/2/frequency; //2 because we selected TIMER_CLOCK1 above
  TC_SetRA(tc, channel, rc/2); //50% high, 50% low
  TC_SetRC(tc, channel, rc);
  TC_Start(tc, channel);
  tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
  tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(irq);
}

void DacSetup () {
  PIOB->PIO_PDR |= PIO_PDR_P15 | PIO_PDR_P16;  // Disable GPIO on corresponding pins DAC0 and DAC1
  PMC->PMC_PCER1 |= PMC_PCER1_PID38 ;     // DACC power ON
  DACC->DACC_CR = DACC_CR_SWRST ;         // reset DACC

  DACC->DACC_MR = DACC_MR_REFRESH (1)
                  | DACC_MR_STARTUP_0
                  | DACC_MR_MAXS
                  | DACC_MR_USER_SEL_CHANNEL1;

  DACC->DACC_CHER =  DACC_CHER_CH1;      // enable DAC  channel 1
}
