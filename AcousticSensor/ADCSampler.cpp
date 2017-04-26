#include "ADCSampler.h"

#include <Arduino.h>

#define INSTRUMENT_CLOCK_RATE 84000000
#define DEFAULT_CHANNEL A0
#define DEFAULT_SAMPLING_FREQ 44100

ADCSamplerClass::ADCSamplerClass(void){
  _channel = DEFAULT_CHANNEL;  // by default
  _samplingFrequency = DEFAULT_SAMPLING_FREQ;
  _callback = NULL;
  _currentBufferIndex = 0;
}

void ADCSamplerClass::init(){
  
  // enable the ADC
  pmc_enable_periph_clk(ID_ADC);
  
  //Initialize the given ADC with the SystemCoreClock and startup time
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  
  // trigger on TC0
  ADC->ADC_MR |= 0x3;
  
  // enable the adc interrupt
  NVIC_EnableIRQ(ADC_IRQn);
  
  // set it as the highest priority
  NVIC_SetPriority(ADC_IRQn, 0);
  
  //disable all interrupts except RXEND
  ADC->ADC_IDR=~(1<<27);
  
  // only use ADC RXEND interrupt
  ADC->ADC_IER=1<<27;
}

void ADCSamplerClass::start(){
  TC_Start(TC0,0);
}

void ADCSamplerClass::stop(){
  TC_Stop(TC0, 0);
}

void ADCSamplerClass::reset(){
  // force stop! 
  this->stop();
  
  // stop the ADC
  
  // reconfigure sampling frequency
  {
    // PB25 = 2 Arduino Due
    pinMode(2,OUTPUT);

    // Start the pin HIGH
    analogWrite(2,255);

    // Disables the PIO from controlling the pin 25 (enables peripheral control of pin 25).
    REG_PIOB_PDR = 1<<25;

    // Assigns the Pin 25 I/O line to the Peripheral B function.
    REG_PIOB_ABSR= 1<<25;
    
    // Timer Control Channel Mode register
    // ABETRG = 1 -- TIOA is used as an external trigger
    // WAVE = 1 -- Capture mode is disabled (Waveform mode is enabled): PWM with independent duty cycles
    // CPCTRG = 1 --  RC Compare resets the counter and restarts the counter (Reset on Comparison with TC0_RC0)
    // LDRA = 01 -- Selected Edge for PIOA Rising
    // LDRB = 10 -- Selected Edge for PIOB Falling
    REG_TC0_CMR0=0b00000000000010011100010000000000; //CLock selection

    // Compare register C for the TC
    REG_TC0_RC0=INSTRUMENT_CLOCK_RATE/(_samplingFrequency*2);
    // Compare register A for the TC
    REG_TC0_RA0=1;
  }
  
  // reconfigure ADC channel and DMA
  {
    // disable all channels except target channel
    ADC->ADC_CHDR = (0xFF ^ (0x01 << _channel));
    
    // enable the specific channel
    ADC->ADC_CHER = (0x01 << _channel);
    
    // Do not TAG measurements with channel number
    // no comparions done
    ADC->ADC_EMR = 0x00000000;
    
    // ADC writes to the DMA buffer (address of buffer to write to)
    // start at buffer 0
    _currentBufferIndex = 0;
    ADC->ADC_RPR=(uint32_t)(&_sampleBuffers[_currentBufferIndex][0]);

    // number of samples to take
    ADC->ADC_RCR=(BUFFER_SIZE);
    
    // prepare next buffer
    this->prepareNextBuffer();
    
    //enable dma pdc receiver channel
    ADC->ADC_PTCR=1;
  }
  
  // start adc
  ADC->ADC_CR=2;
}

void ADCSamplerClass::prepareNextBuffer(){
  // next DMA buffer
  ADC->ADC_RNPR=(uint32_t)(&_sampleBuffers[(_currentBufferIndex+1)%NUM_BUFFERS][0]);
  //next dma receive counter
  ADC->ADC_RNCR=(BUFFER_SIZE);
}

void ADCSamplerClass::setChannel(uint8_t channel){
  _channel = channel;
}

void ADCSamplerClass::setSamplingFrequency(uint32_t freq){
  _samplingFrequency = freq;
}

void ADCSamplerClass::setInterruptCallback(adc_callback callback){
  _callback = callback;
}

adc_callback ADCSamplerClass::getInterruptCallback(){
  return _callback;
}

uint16_t *ADCSamplerClass::getBufferAtIndex(int8_t index){
  if(index < 0){  // wrap around incase of negative index
    index += NUM_BUFFERS;
  }
  return _sampleBuffers[index%NUM_BUFFERS];
}

void ADCSamplerClass::bufferFullInterrupt(){
  if(this->_callback != NULL){
    this->_callback(
      _currentBufferIndex,
      &_sampleBuffers[_currentBufferIndex][0], 
      BUFFER_SIZE
    );
  }
  // increment _currentBufferIndex
  _currentBufferIndex = (_currentBufferIndex+1)%NUM_BUFFERS;
  this->prepareNextBuffer();
}

ADCSamplerClass ADCSampler;


/**
 *  ADC_Handler
 */

void ADC_Handler(){
  //Read reason for interrupt
  int f=ADC->ADC_ISR;

  // The Receive Counter Register has reached 0 since the last write in ADC_RCR or ADC_RNCR.
  // Check if the Buffer is full
  if (f&(1<<27)){
    ADCSampler.bufferFullInterrupt();
    return;
  }
}
