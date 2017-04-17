#include <Arduino.h>

#include <assert.h>
#include <FreeRTOS_ARM.h>
#include <IPAddress.h>
#include <PowerDueWiFi.h>

#include "ADCConfig.h"
#include "ADCSampler.h"
#include "ADCTrigger.h"
#include "ADCClock.h"
#include "PowerDue.h"

#include "debug.h"
#include "packet.h"
#include "synchronization.h"

QueueHandle_t xBufferQueue;
QueueHandle_t xTcpQueue;
ADCTrigger trigger;

// store the timestamps for each buffer to report later
tstamp_t tstamps[NUM_BUFFERS];

#define PACKET_BUFFER_COUNT 2   // packet buffers

AcousticEvent_t capturedEvent[PACKET_BUFFER_COUNT];

/*
 *  Initialization Routines
 */

inline void initQueues(){
  xBufferQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));
  xTcpQueue = xQueueCreate(BUFFER_QUEUE_LENGTH, sizeof(uint8_t));
}

inline void initPacket(){
  for(int p=0; p < PACKET_BUFFER_COUNT; p++){
    capturedEvent[p].pktHeader = EVENT_PACKET_HEADER;
    for(int i=0; i < DEVICE_ID_LENGTH; i++){
      capturedEvent[p].deviceID[i] = DEVICE_ID[i];
    }
    capturedEvent[p].clockOffset = 0;
    capturedEvent[p].networkDelay = 0;
    capturedEvent[p].samplingFrequency = ADC_SAMPLE_RATE;  
    capturedEvent[p].pktFooter = EVENT_PACKET_FOOTER;
  }  
}

inline void initPins(){
  pinMode(CLOCK_GPIO_OUT, OUTPUT);
}

volatile int pinState = HIGH;
inline void togglePin(int state){
  digitalWrite(CLOCK_GPIO_OUT, state);
  pinState = state;
}

inline void initADC(){
  ADCClock.init(BUFFER_SIZE);
  ADCClock.reset();
  
  ADCSampler.init();
  ADCSampler.setChannel(ADC_CHANNEL_MIC);
  ADCSampler.setSamplingFrequency(ADC_SAMPLE_RATE);
  ADCSampler.setInterruptCallback(adc_full);
  ADCSampler.reset();
  ADCSampler.start();
}



void adc_full(uint8_t index, uint16_t *readyBuffer, uint16_t size){
  // clock goes tick!
  ADCClock.tick();
  
  // save current time!
  tstamps[index] = ADCClock.getTime();
  
  // if even, toggle low... if odd, toggle high
  togglePin(ADCClock.getMajorTicks()%2);
  
  // send buffer index for processing
  xQueueSendToBackFromISR(xBufferQueue, &index, NULL);
}

/**
 *  Handles each buffer received
 */
void BufferHandlerTask(void *arg){
  initADC();
  uint8_t bufIndex;
  uint8_t bufCount = 0;
  uint8_t currPacket = 0;
  while(1){
    bufIndex = 0;
    if(xQueueReceive(xBufferQueue, &bufIndex, portMAX_DELAY)){
      
      // process the buffer to check for an event
      trigger.feed(ADCSampler.getBufferAtIndex(bufIndex), BUFFER_SIZE);
      
      // did we detect an event? Should we queue up buffers?
      if(bufCount > 0 || trigger.isTriggered()){
        trigger.reset();
        // we want to send BUFFERS_TO_SEND buffers
        // if this is the first time to trigger, set how many buffers we should count
        if(bufCount == 0){
          bufCount = BUFFERS_TO_SEND; // start sending out buffers
          
          // start by sending the previous buffer
          uint8_t ind = 0;
          if(bufIndex == 0){
            ind = NUM_BUFFERS-1;
          } else {
            ind = bufIndex-1;
          }

          capturedEvent[currPacket].timestamp = tstamps[ind];
          // add NTP information to allow estimation of errors offline
          capturedEvent[currPacket].clockOffset = ADCClock.getOffset();
          capturedEvent[currPacket].networkDelay = ADCClock.getNetworkDelay();
          
          // capture the previous buffer into the packet
          memcpy( 
                  &(capturedEvent[currPacket].samples[BUFFER_SIZE * (BUFFERS_TO_SEND-bufCount)]), 
                  (uint8_t *)ADCSampler.getBufferAtIndex(ind),
                  BUFFER_SIZE * BYTES_PER_SAMPLE
                );
          bufCount--;
        }
        
        // copy the current buffer into the packet
        memcpy( 
                &(capturedEvent[currPacket].samples[BUFFER_SIZE * (BUFFERS_TO_SEND-bufCount)]), 
                (uint8_t *)ADCSampler.getBufferAtIndex(bufIndex),
                BUFFER_SIZE * BYTES_PER_SAMPLE
              );
        bufCount--;
        
        // when we've collected BUFFERS_TO_SEND buffers, queue the packet for transmission
        if(bufCount == 0){
          // send out the buffer
          sendPacket(currPacket);
          currPacket = (currPacket+1)%PACKET_BUFFER_COUNT;  
        }
      }
    }
  }
  ADCSampler.stop();
}

void sendPacket(uint8_t packetIndex){
  // don't block when sending
  xQueueSendToBack(xTcpQueue, &packetIndex, 0);
}

void TcpStreamOut(void *arg){
  struct sockaddr_in serverAddr;  
  socklen_t socklen;
  // outer loop to establish connection
  while(1){
    memset(&serverAddr, 0, sizeof(serverAddr));
    
    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    inet_pton(AF_INET, SERVER_IP, &(serverAddr.sin_addr));
    
    int s = lwip_socket(AF_INET, SOCK_STREAM, 0);
    int r = 0;
    while(r = lwip_connect(s, (struct sockaddr *)&serverAddr, sizeof(serverAddr))){
      PRINT_DEBUGLN("Failed to connect to server!!!");
      
      // retry
      lwip_close(s);
      lwip_shutdown(s, SHUT_RDWR);
      // flush the receive queue so we don't block the rest of the system
      xQueueReset(xTcpQueue);
      
      vTaskDelay(pdMS_TO_TICKS(2000)); // wait a bit and try again
    }
    
    // add led notice here when ready
    pd_rgb_led(PD_RED);
    
    uint8_t packetIndex;
    
    // inner loop to handle sending data
    while(1){
      if(xQueueReceive(xTcpQueue, &packetIndex, portMAX_DELAY)){
        pd_rgb_led(PD_GREEN);
        PRINT_DEBUG("Packet Length:");
        PRINT_DEBUGLN(PACKET_LENGTH);

        if(lwip_write(s, (uint8_t *)&capturedEvent[packetIndex], PACKET_LENGTH) < 0){
          // an error occurred during write.
          // did we disconnect? break out and try to reconnect.
          break;
        }
        pd_rgb_led(PD_RED);
      }
    }
    
    // close socket after everything is done
    lwip_close(s);
    lwip_shutdown(s, SHUT_RDWR);
    pd_rgb_led(PD_OFF);
  }
  
}

void onError(int errorCode){
  // An error occurred when trying to connect to the network
  PRINT_DEBUG("Failed to initialize device: ");
  PRINT_DEBUGLN(errorCode);
  
  assert(false);
}

void onReady(){
  // WiFi connection ready!
  // initialize tasks
  PRINT_DEBUGLN("Device Ready!");
  PRINT_DEBUG("Device IP: ");
  PRINT_DEBUGLN(IPAddress(PowerDueWiFi.getDeviceIP()));
  
  startTasks();
}

void startTasks(void){
  xTaskCreate(BufferHandlerTask, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  xTaskCreate(TcpStreamOut, NULL, configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  startSyncTasks(NTP_TASK_PRIORITY);
}

void setup(){
  
#if DEBUG
  // only needed when debug is turned on
  // most likely not needed when already deployed
  SerialUSB.begin(0);
  while(!SerialUSB);
#endif
  
  initPins();
  initPacket();
  initQueues();
  pd_rgb_led_init();
  
  // initialize WiFi connection and lwIP stack
  PowerDueWiFi.init(WIFI_SSID, WIFI_PASS);
  PowerDueWiFi.setCallbacks(onReady, onError);

  vTaskStartScheduler();

  while(1);
}

void loop(){
  // sleep...
}
