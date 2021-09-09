uint32_t  AdcResult;
uint32_t  DacSample;

#define FS 44000
static int digital_sig = 4095;
float arr1[4400];
float arr2[4400];
int h;
int x;
int j = 0;;
volatile unsigned long elapsedTime = 0;
void setup() {
  Serial.begin(9600);
  startTimer(TC1, 0, TC3_IRQn, FS); //TC1 channel 0, the IRQ for that channel and the desired frequency
  DacSetup();
  AdcSetup();
//  for(int i = 0; i < 4400; i++)
//  {
//    arr1[i] = random(663,3401);
//  }
//
//  for(int i = 0; i < 4400; i++)
//  {
//    arr1[i] = arr1[i]*3.3/4096;
  }

//    x = micros();
//    h = micros();

void loop() {
  //calculate voltages adcresult*3.3/2^12
//  unsigned long interval = elapsedTime;
//  Serial.println(interval);
Serial.print("ADC Result in Digital: ");
Serial.println(AdcResult);
Serial.print("ADC Result in Voltage (DigitalValue * 3.3 / 4096): ");
  Serial.print(AdcResult*3.3/4096); //(DAC output range is actually from 0.55 V to 2.75 V only i.e. 663-3400 in digital)
  Serial.println("V");
  Serial.println("");

//  x = micros();
//  int sum = 0;
//  int square = 0;
//  for(int i = 0; i < 4400; i++)
//  {
//    arr1[i] = pow(arr1[i],2);
//    sum = sum + arr1[i];
//  }
//  square = sqrt(sum);
//  int y = micros();
//  Serial.print("RMS Time: ");
//  Serial.println(y-x);

}
void TC3_Handler()
{
//  static unsigned long previousTime = 0;
//  unsigned long time = micros();
//  elapsedTime = time - previousTime;
//  previousTime = time;
//  if(j == 0)
//  {
//    h = micros();
//  }
  TC_GetStatus(TC1, 0);
  AdcResult = ADC->ADC_CDR[7];            // Read the previous result
  ADC->ADC_CR |= ADC_CR_START;            // Begin the next ADC conversion. 
  DACC->DACC_CDR = DacSample;  // Start the next DAC conversion
//  arr2[j] = AdcResult;
//  j++;
//  if(j>4399)
//  {
//    j = 0;
//    int f = micros();
//    Serial.print("Buffer buildup time: ");
//    Serial.println(f-h);
//  }
  DacSample = digital_sig;
  
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

void AdcSetup(){ 
  ADC->ADC_WPMR &= ~(ADC_WPMR_WPEN); //Disable the Write Protect Mode 
  ADC->ADC_CHER |= ADC_CHER_CH7; //Enable A0 pin 
  ADC->ADC_MR = 0; 
  ADC->ADC_MR = ADC_MR_PRESCAL(4);    //ADC Clock set to 8MHz 
  ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
  ADC->ADC_EMR = 0;
}
