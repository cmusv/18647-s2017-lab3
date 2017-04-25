#ifndef _SAMPLER_H
#define _SAMPLER_H

#include "ADCConfig.h"

/**
 * WARNING: This callback runs in an ISR context.
 * Make sure to not block and run as fast as possible.
 * (i.e. delegate the action to a task via posting to a queue)
 */

typedef void (*adc_callback)(uint8_t index, uint16_t *readyBuffer, uint16_t size);

class ADCSamplerClass
{
  public:
    void init();
    void start();
    void stop();
    void reset();
    
    // these settings do not take effect until a reset occurs
    /* use channel numbers instead of pin numbers (e.g. 0-15) */
    void setChannel(uint8_t channel);         // which ADC channel to sample
    void setSamplingFrequency(uint32_t freq); // this probably won't be exact
    
    void setInterruptCallback(adc_callback callback);
    adc_callback getInterruptCallback();
    
    uint16_t *getBufferAtIndex(int8_t index);
  
  public: // protected methods - do not call these directly
    ADCSamplerClass();
    void bufferFullInterrupt();
  
  protected:
    void prepareNextBuffer();
    
  private:
    uint8_t _channel;
    uint32_t _samplingFrequency;
    adc_callback _callback;
    
    uint8_t _currentBufferIndex;
    uint16_t _sampleBuffers[NUM_BUFFERS][BUFFER_SIZE];
};

extern ADCSamplerClass ADCSampler;

#endif  //_SAMPLER_H
