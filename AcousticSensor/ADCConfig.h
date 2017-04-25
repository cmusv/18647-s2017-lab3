#ifndef ADCCONFIG_H
#define ADCCONFIG_H

#include <inttypes.h>

/*
 * Device Parameters
 */

#define DEVICE_ID "PDue0000"    // use this to uniquely identify your device (max of 8 characters)

/*
 * Application Parameters
 */

// TODO: Update these values to the proper network / server IP
#define WIFI_SSID "PowerDue"
#define WIFI_PASS "powerdue"
#define SERVER_IP "10.230.12.1"  // typically the IP of your Laptop (e.g. "10.230.12.1")
#define SERVER_PORT 1991          // Port of TCP input node of Node-Red flow TCP-MQTT adapter

/*
 * Clock parameters
 */

// This defines whether this device is the master clock or not
// 0 - this is a slave clock (see below for extra parameters)
// 1 - this is the master clock
#define MASTER_CLOCK 1

// The udp port to use for synchronization
#define SYNC_PORT 12345         // no need to modify. must be consistent across all devices

#if MASTER_CLOCK == 0
  // TODO: If this is a slave clock, this must be defined properly
  // Set this to the IP Address of your Master Clock
  #define MASTER_CLOCK_IP ""    // e.g. "10.230.12.10"
#endif

#define SYNC_FREQUENCY  10000   // Time period between NTP synchronization trials (in milliseconds)

#define NTP_TASK_PRIORITY   1   // priority of NTP synchronization tasks

extern uint32_t sync_frequency;

/*
 *  ADCSampler parameters
 */
 
// TODO: Modify the parameters below to suit your application needs
#define ADC_SAMPLE_RATE 1000    // Sampling Rate in Hz of the ADC Sampler

#define NUM_BUFFERS   8         // How many buffers should the ADC Sampler keep track of?

#define BUFFER_SIZE   256       // How many samples per buffer should the DMA fill up

#define BUFFERS_TO_SEND 4       // How many buffers to send to the visualizer

extern uint32_t adc_sample_rate;

/*
 * ADCTrigger parameters
 */
 
#define WINDOW_COUNT 8          // How many signal windows to keep track of
#define MIN_WINDOW_COUNT 24     // How many signal windows to ignore on start up before enabling triggering
#define TRIGGER_DEFAULT_STD_DISTANCE   20.0f  // default std distance from mean to consider an event

extern float trigger_default_std_distance;

/*
 * Do not modify the following parameters
 */ 
#define ADC_CHANNEL_MIC 2       // mic is connected to A2

#define BYTES_PER_SAMPLE 2      // the ADC generates 12-bit samples (i.e. 2 bytes of data per sample)
 
typedef uint64_t tstamp_t;      /* defines the size of a timestamp that's sent over the network */

#define CLOCK_GPIO_OUT 51       // GPIO pin to toggle on each clock tick

#define BUFFER_QUEUE_LENGTH 8   // FreeRTOS Queue length 
 
#endif  //ADCCONFIG_H
