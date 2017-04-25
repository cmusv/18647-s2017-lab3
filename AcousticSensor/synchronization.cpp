#include "synchronization.h"

#include "debug.h"
#include "ADCClock.h"

#include <assert.h>
#include <FreeRTOS_ARM.h>
#include <PowerDueWiFi.h>

void initNtpServerAddr();
int getNtpSocket();

#if MASTER_CLOCK
  void vNtpMasterReceiveTask(void *arg);
#else
  void vNtpRequestCallback( TimerHandle_t pxTimer );
  void vNtpSlaveReceiveTask(void *arg);
  TimerHandle_t xNtpTimer;
#endif

#define NTP_SERVER_TASK_NAME "NTPServer"
#define NTP_SYNC_TIMER_NAME  "NTPSync"
#define NTP_RECV_TASK_NAME   "NTPRecv" 

// define freertos tasks and timers
void startSyncTasks(int priority){
  initNtpServerAddr();
#if MASTER_CLOCK
  xTaskCreate(vNtpMasterReceiveTask, NTP_SERVER_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, priority, NULL);
#else
  xNtpTimer = xTimerCreate(NTP_SYNC_TIMER_NAME, pdMS_TO_TICKS(sync_frequency), pdTRUE, 0, vNtpRequestCallback);
  xTimerStart(xNtpTimer, portMAX_DELAY);
  xTaskCreate(vNtpSlaveReceiveTask, NTP_RECV_TASK_NAME, configMINIMAL_STACK_SIZE, NULL, priority, NULL);
#endif
}

/**
 * MTP Tasks
 */

static struct sockaddr_in _ntpServer;  
void initNtpServerAddr(){
  memset(&_ntpServer, 0, sizeof(_ntpServer));
  _ntpServer.sin_len = sizeof(_ntpServer);
  _ntpServer.sin_family = AF_INET;
  _ntpServer.sin_port = htons(SYNC_PORT);
#if MASTER_CLOCK
  _ntpServer.sin_addr.s_addr = htonl(INADDR_ANY); // listens on any IP
#else
  // communicate to MASTER_CLOCK_IP
  if(strlen(MASTER_CLOCK_IP) == 0){
    PRINT_DEBUGLN("ERROR: MUST DEFINE MASTER_CLOCK_IP!!!");
    assert(false);
  }
  inet_pton(AF_INET, MASTER_CLOCK_IP, &(_ntpServer.sin_addr));  
#endif
}
 
static int _ntpSocket = -1;
int getNtpSocket(){
  // lazy load of mtp socket
  if(_ntpSocket < 0){
    struct sockaddr_in serverAddr;  
    socklen_t socklen;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_len = sizeof(serverAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SYNC_PORT);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    int sock = lwip_socket(AF_INET, SOCK_DGRAM, 0);
    if (bind(sock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0) {
      PRINT_DEBUGLN("ERROR: NTP Socket bind failed!!!");
      return -1;
    }
    _ntpSocket = sock;
  }
  return _ntpSocket;
}

#if !MASTER_CLOCK

void changeSyncPeriod(uint32_t period){
  PRINT_DEBUG("Change NTP sync period to ");
  PRINT_DEBUGLN(period);
  xTimerChangePeriod( xNtpTimer, pdMS_TO_TICKS(sync_frequency), 5 );
}

void vNtpRequestCallback( TimerHandle_t pxTimer ){
  // send mtp request
  tstamp_t t0 = ADCClock.getTime();
  
  // send sync packet
  lwip_sendto(getNtpSocket(), 
              (uint8_t *)&t0, 
              SYNC_PACKET_SIZE, 
              0, 
              (struct sockaddr*)&_ntpServer, 
              sizeof(_ntpServer));  
}

void vNtpSlaveReceiveTask(void *arg){
  int socket = getNtpSocket();
  uint8_t recvBuffer[RESP_PACKET_SIZE];
  tstamp_t t0, t1, t2, t3;
  struct sockaddr clientAddr;
  socklen_t sockLen;
  while(1){
    // wait for ntp responses
    // this would block until something is received
    lwip_recvfrom(socket, recvBuffer, RESP_PACKET_SIZE, 0, &clientAddr, &sockLen);
    
    // get t3
    t3 = ADCClock.getTime();
    
    // read t0, t1 and t2 from packet
    t0 = *((tstamp_t*)&recvBuffer[0]);
    t1 = *((tstamp_t*)&recvBuffer[TIMESTAMP_SIZE]);;
    t2 = *((tstamp_t*)&recvBuffer[TIMESTAMP_SIZE*2]);;
    
    int offset = ((t1-t0) + (t2-t3))/2;
    int delay = ((t3-t0) - (t2-t1))/2;
    
    PRINT_DEBUG("Offset: ");
    PRINT_DEBUG(offset);
    PRINT_DEBUG(" Delay: ");
    PRINT_DEBUGLN(delay);
    
    // adjust the clock based on computed offset an delay
    // TODO: how do we choose when to adjust our clock?
    ADCClock.addOffset(offset, delay);
  }
}

#endif

#if MASTER_CLOCK

void vNtpMasterReceiveTask(void *arg){
  int socket = getNtpSocket();
  uint8_t recvBuffer[SYNC_PACKET_SIZE];
  uint8_t respBuffer[RESP_PACKET_SIZE];
  tstamp_t t0, t1, t2;
  struct sockaddr clientAddr;
  socklen_t sockLen;
  while(1){
    // wait for ntp requests
    // this would block until something is received
    lwip_recvfrom(socket, recvBuffer, SYNC_PACKET_SIZE, 0, &clientAddr, &sockLen);
    // get t1
    t1 = ADCClock.getTime();
    
    // move t0 and t1 into the response buffer 
    memcpy(&respBuffer[0], recvBuffer, TIMESTAMP_SIZE);
    memcpy((&respBuffer[TIMESTAMP_SIZE]), (uint8_t*)(&t1), TIMESTAMP_SIZE);
    
    t0 = *((tstamp_t*)&recvBuffer[0]);
    
    PRINT_DEBUG("t0: ");
    PRINT_DEBUG((uint32_t)t0);
    PRINT_DEBUG(" t1: ");
    PRINT_DEBUGLN((uint32_t)t1);
    
    // get t2
    t2 = ADCClock.getTime();
    
    // place it in the packet
    memcpy((&respBuffer[TIMESTAMP_SIZE*2]), (uint8_t*)(&t2), TIMESTAMP_SIZE);
    
    // send response packet
    lwip_sendto(getNtpSocket(), 
                respBuffer, 
                RESP_PACKET_SIZE, 
                0, 
                (struct sockaddr*)&clientAddr, 
                sockLen); 
  }
}

#endif
