#ifndef LORA_H
#define LORA_H

#include <Arduino.h>
#include <ttgov21new.h>
#define PAYLOAD_BUFFER_SIZE 50
#define SEND_QUEUE_SIZE 250
#define LORA_ADR true
#define LORA_DEFAULT_SF 7
#define RCMDPORT 4
#define LORA_CONFIRMED 0
#define CLOCK_ERROR_PROCENTAGE 5
#define CFG_eu868 1
#define CFG_sx1276_radio 1

//   buchutest Config
#define APPEUI_DEF{ 0x64, 0x47, 0x1d, 0xa3, 0xe1, 0x71, 0x99, 0x13}
#define DEVEUI_DEF{  0x64, 0x47, 0x1d, 0xa3, 0xe1, 0x71, 0x99, 0x13}
 //LSB
//#define APPKEY_DEF {0x53, 0x6c, 0xaa, 0x19, 0x27, 0x53, 0xdc, 0x21, 0x25, 0xc2, 0xa7, 0xc3, 0x50, 0xf7, 0x37, 0xd6} //LSB
#define APPKEY_DEF { 0xd6, 0x37, 0xf7, 0x50, 0xc3, 0xa7, 0xc2, 0x25, 0x21, 0xdc, 0x53, 0x27, 0x19, 0xaa, 0x6c, 0x53}  //MSB

/*
#define DEVEUI_DEF {0x00, 0x70, 0xF0, 0xBF, 0x71, 0x3C, 0x1F, 0x18}
#define APPEUI_DEF {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define APPKEY_DEF {0x06, 0x26, 0x35, 0x77, 0xC3, 0xBB, 0xC9, 0xEB, 0x2F, 0xEF, 0x99, 0x13, 0x5E, 0xF0, 0xF6, 0xA5}
*/
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

enum sendprio_t { prio_low, prio_normal, prio_high };

typedef struct {
  uint8_t MessageSize;
  uint8_t MessagePort;
  uint8_t Message[PAYLOAD_BUFFER_SIZE];
} MessageBuffer_t;

extern QueueHandle_t LoraSendQueue;

void onEvent(ev_t ev);
void os_getDevKey(u1_t *buf);
void os_getArtEui(u1_t *buf);
void os_getDevEui(u1_t *buf);
void switch_lora(uint8_t sf, uint8_t tx);
void lora_send(osjob_t *job);
void lora_enqueuedata(MessageBuffer_t *message, sendprio_t prio);
void lora_queuereset(void);
extern void command_callback(uint8_t*, uint8_t);
void setCallback(void (*cb)(uint8_t*, uint8_t));
void loraLoop();
void initLora();


#endif