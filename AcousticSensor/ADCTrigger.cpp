#include "ADCTrigger.h"

#include <math.h>

ADCTrigger::ADCTrigger(){
  _isTriggered = false;
  _windowCount = 0;
  _currentIndex = 0;
  _triggerCount = 0;
  _stdDistance = trigger_default_std_distance;
}

void ADCTrigger::feed(uint16_t *buffer, uint16_t len){
  // TODO: How long does this actually run?
  // Does it run faster than the buffer filling up?

  // get these before updating them
  float meanEnergy = getEnergyMean();
  float stdEnergy = getEnergyStd();
  
  float winEnergy = updateMeanAndEnergy(buffer, len);
  
  if(winEnergy > (meanEnergy + _stdDistance*stdEnergy)){
    if(_triggerCount >= 1){
      _isTriggered = true;
    } else {
      // ignore the first trigger
      // likely due to startup
      _triggerCount++; 
    }
  }
}

void ADCTrigger::setStdDistance(float stdDistance){
  _stdDistance = stdDistance;
}

bool ADCTrigger::isTriggered(){
  return (_windowCount>=MIN_WINDOW_COUNT) && _isTriggered;
}

void ADCTrigger::reset(){
  _isTriggered = false;
}

float ADCTrigger::updateMeanAndEnergy(uint16_t *buffer, uint16_t len){
  float ampTotal=0;
  float enTotal=0;
  
  // unroll loop by 4
  uint8_t unrollFactor = 2u;
  float windows = len >> unrollFactor;
  uint32_t index=0;
  float sigMean = getSignalMean();
  for(int i=0; i < windows; i++){
    index = i*unrollFactor;
    ampTotal += buffer[index];
    enTotal += pow((buffer[index]-sigMean),2);
    ampTotal += buffer[index+1];
    enTotal += pow((buffer[index+1]-sigMean),2);
    ampTotal += buffer[index+2];
    enTotal += pow((buffer[index+2]-sigMean),2);
    ampTotal += buffer[index+3];
    enTotal += pow((buffer[index+3]-sigMean),2);
  }
  
  // get mean
  ampTotal = ampTotal/len;
  
  // insert into arrays
  _mean[_currentIndex] = ampTotal;
  _energy[_currentIndex] = enTotal;
  
  // wait for 2*MIN_WINDOW_COUNT 
  // to say we have enough data
  if(_windowCount < MIN_WINDOW_COUNT){
    _windowCount++;
  }
  _currentIndex = (_currentIndex+1)%WINDOW_COUNT;
  
  return enTotal;
}

float ADCTrigger::getSignalMean(){
  float tot=0;
  for(int i=0; i < WINDOW_COUNT; i++){
    tot += _mean[i];
  }
  return tot/WINDOW_COUNT;
}

float ADCTrigger::getEnergyMean(){
  float tot=0;
  for(int i=0; i < WINDOW_COUNT; i++){
    tot += _energy[i];
  }
  return tot/WINDOW_COUNT;
}

float ADCTrigger::getEnergyStd(){
  float mean = getEnergyMean();
  float tot=0;
  for(int i=0; i < WINDOW_COUNT; i++){
    tot += pow((_energy[i] - mean),2);
  }
  tot = tot/WINDOW_COUNT;
  return sqrt(tot);
}
