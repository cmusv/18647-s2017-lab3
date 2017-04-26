#ifndef _PACKET_H
#define _PACKET_H

#include "ADCConfig.h"

// Do not modify these values, these are expected by the server
#define EVENT_PACKET_HEADER   0xF0F0F0F0
#define EVENT_PACKET_FOOTER   0xF7F7F7F7

#define DEVICE_ID_LENGTH      8

/* this is the packet format to send each time we detect an event */
struct AcousticEvent_t {
  uint32_t pktHeader;                 // do not modify
  uint16_t pktLength;                 // set before transmitting, removed before consuming by scope
  char deviceID[DEVICE_ID_LENGTH];    // device unique ID
  uint32_t samplingFrequency;         // rate this device is sampling at
  uint64_t timestamp;                 // sample number of first in buffer
  int32_t clockOffset;                // the measured clock offset between this device and the master
  int32_t networkDelay;               // the measured network delay from NTP
  uint16_t samples[BUFFERS_TO_SEND * BUFFER_SIZE]; // buffer of samples to send
  uint32_t pktFooter;                 // do not modify
} __attribute__ ((packed));

#define PACKET_LENGTH sizeof(AcousticEvent_t)

#endif  //_PACKET_H
