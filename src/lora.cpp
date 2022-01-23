#include <lora.h>

osjob_t sendjob;
QueueHandle_t LoraSendQueue;

void SendPayload(uint8_t port, sendprio_t prio, uint8_t *buff, uint8_t size) {

  MessageBuffer_t SendBuffer; // contains MessageSize, MessagePort, Message[]

  SendBuffer.MessageSize = size;
  SendBuffer.MessagePort = port;
  memcpy(SendBuffer.Message, buff, size);
  lora_enqueuedata(&SendBuffer, prio);

} // SendPayload

class MyHalConfig_t : public Arduino_LMIC::HalConfiguration_t {

public:
  MyHalConfig_t(){};
  virtual void begin(void) override {
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  }
};

MyHalConfig_t myHalConfig{};

// LMIC pin mapping

const lmic_pinmap lmic_pins = {
    .nss = LORA_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LORA_RST == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_RST,
    .dio = {LORA_IRQ, LORA_IO1,
            LORA_IO2 == NOT_A_PIN ? LMIC_UNUSED_PIN : LORA_IO2},
    // optional: set polarity of rxtx pin.
    .rxtx_rx_active = 0,
    // optional: set RSSI cal for listen-before-talk
    // this value is in dB, and is added to RSSI
    // measured prior to decision.
    // Must include noise guardband! Ignored in US,
    // EU, IN, other markets where LBT is not required.
    .rssi_cal = 0,
    // optional: override LMIC_SPI_FREQ if non-zero
    .spi_freq = 0,
    .pConfig = &myHalConfig};

// LMIC callback functions
static const u1_t PROGMEM DEVEUI[8] = DEVEUI_DEF;
static const u1_t PROGMEM APPEUI[8] = APPEUI_DEF;
static const u1_t PROGMEM APPKEY[16] = APPKEY_DEF;

void os_getDevKey(u1_t *buf) { memcpy(buf, APPKEY, 16); }

void os_getArtEui(u1_t *buf) { memcpy(buf, APPEUI, 8); }

void os_getDevEui(u1_t *buf) {
  memcpy(buf, DEVEUI, 8); // get fixed DEVEUI from loraconf.h
}

void onEvent(ev_t ev) {
  char buff[24] = "";
  uint32_t now_micros = 0;

  switch (ev) {

  case EV_SCAN_TIMEOUT:
    strcpy_P(buff, PSTR("SCAN TIMEOUT"));
    break;

  case EV_BEACON_FOUND:
    strcpy_P(buff, PSTR("BEACON FOUND"));
    break;

  case EV_BEACON_MISSED:
    strcpy_P(buff, PSTR("BEACON MISSED"));
    break;

  case EV_BEACON_TRACKED:
    strcpy_P(buff, PSTR("BEACON TRACKED"));
    break;

  case EV_JOINING:
    strcpy_P(buff, PSTR("JOINING"));
    break;

  case EV_JOINED:
    strcpy_P(buff, PSTR("JOINED"));
    // set data rate adaptation according to saved setting
    LMIC_setAdrMode(LORA_ADR);
    // set cyclic lmic link check to off if no ADR because is not supported by
    // ttn (but enabled by lmic after join)
    LMIC_setLinkCheckMode(LORA_ADR);
    // Set data rate and transmit power (note: txpower seems to be ignored by
    // the library)
    //switch_lora(LORA_DEFAULT_SF, 14);
    // kickoff first send job
    os_setCallback(&sendjob, lora_send);
    // show effective LoRa parameters after join
    Serial.printf("DEVaddr=%08X\n", LMIC.devaddr);
    break;

  case EV_RFU1:
    strcpy_P(buff, PSTR("RFU1"));
    break;

  case EV_JOIN_FAILED:
    strcpy_P(buff, PSTR("JOIN FAILED"));
    break;

  case EV_REJOIN_FAILED:
    strcpy_P(buff, PSTR("REJOIN FAILED"));
    break;

  case EV_TXCOMPLETE:
    strcpy_P(buff, (LMIC.txrxFlags & TXRX_ACK) ? PSTR("RECEIVED ACK")
                                               : PSTR("TX COMPLETE"));

    if (LMIC.dataLen) { // did we receive payload data -> display info
      Serial.printf("Received %d bytes of payload, RSSI %d SNR %d\n",
               LMIC.dataLen, LMIC.rssi, LMIC.snr / 4);

      if (LMIC.txrxFlags & TXRX_PORT) { // FPort -> use to switch

        switch (LMIC.frame[LMIC.dataBeg - 1]) {

        case RCMDPORT: // opcode -> call rcommand interpreter
          command_callback(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
          break;

        default:
          // unknown port -> display info
          Serial.printf("Received data on unsupported port #%d\n",
                   LMIC.frame[LMIC.dataBeg - 1]);
          break;
        }
      }
    }
    break;

  case EV_LOST_TSYNC:
    strcpy_P(buff, PSTR("LOST TSYNC"));
    break;

  case EV_RESET:
    strcpy_P(buff, PSTR("RESET"));
    break;

  case EV_RXCOMPLETE:
    // data received in ping slot
    strcpy_P(buff, PSTR("RX COMPLETE"));
    break;

  case EV_LINK_DEAD:
    strcpy_P(buff, PSTR("LINK DEAD"));
    break;

  case EV_LINK_ALIVE:
    strcpy_P(buff, PSTR("LINK_ALIVE"));
    break;

  case EV_SCAN_FOUND:
    strcpy_P(buff, PSTR("SCAN FOUND"));
    break;

  case EV_TXSTART:
    if (!(LMIC.opmode & OP_JOINING))
        strcpy_P(buff, PSTR("TX START"));
    break;

  case EV_TXCANCELED:
    strcpy_P(buff, PSTR("TX CANCELLED"));
    break;

  case EV_RXSTART:
    strcpy_P(buff, PSTR("RX START"));
    break;

  case EV_JOIN_TXCOMPLETE:
    strcpy_P(buff, PSTR("JOIN WAIT"));
    break;

  default:
    sprintf_P(buff, PSTR("LMIC EV %d"), ev);
    break;
  }

  // Log & Display if asked
  if (*buff) {
     Serial.printf("%s\n", buff);
  }
}

void switch_lora(uint8_t sf, uint8_t tx) {
  if (tx > 20)
    return;
  uint8_t txpower = tx;
  switch (sf) {
  case 7:
    LMIC_setDrTxpow(DR_SF7, tx);
    break;
  case 8:
    LMIC_setDrTxpow(DR_SF8, tx);
    break;
  case 9:
    LMIC_setDrTxpow(DR_SF9, tx);
    break;
  case 10:
    LMIC_setDrTxpow(DR_SF10, tx);
    break;
  case 11:
    LMIC_setDrTxpow(DR_SF11, tx);
    break;
  case 12:
    LMIC_setDrTxpow(DR_SF12, tx);
    break;
  default:
    break;
  }
}

void lora_send(osjob_t *job) {
  MessageBuffer_t SendBuffer;
  // Check if there is a pending TX/RX job running, if yes don't eat data
  // since it cannot be sent right now
  if ((LMIC.opmode & (OP_JOINING | OP_REJOIN | OP_TXDATA | OP_POLL)) != 0) {
    // waiting for LoRa getting ready
  } else {
    if (xQueueReceive(LoraSendQueue, &SendBuffer, (TickType_t)0) == pdTRUE) {
      // SendBuffer now filled with next payload from queue
      char string_message[256];
      for (int i = 0; i < SendBuffer.MessageSize; i ++) {
        sprintf(string_message + 3*i, "%02x ", SendBuffer.Message[i]);
      }
     Serial.printf("sending lora message %s\n", string_message);
      if (!LMIC_setTxData2(SendBuffer.MessagePort, SendBuffer.Message,
                           SendBuffer.MessageSize, (LORA_CONFIRMED))) {
        Serial.printf("%d byte(s) sent to LoRa\n", SendBuffer.MessageSize);
      } else {
        Serial.printf("could not send %d byte(s) to LoRa\n",
                 SendBuffer.MessageSize);
      }
      // sprintf(display_line7, "PACKET QUEUED");
    }
  }
  // reschedule job every 0,5 - 1 sec. including a bit of random to prevent
  // systematic collisions
  os_setTimedCallback(job, os_getTime() + 500 + ms2osticks(random(500)),
                      lora_send);
}

esp_err_t lora_stack_init() {
  assert(SEND_QUEUE_SIZE);
  LoraSendQueue = xQueueCreate(SEND_QUEUE_SIZE, sizeof(MessageBuffer_t));
  if (LoraSendQueue == 0) {
    Serial.printf("Could not create LORA send queue. Aborting.\n");
    return ESP_FAIL;
  }
  Serial.printf("LORA send queue created, size %d Bytes\n",
           SEND_QUEUE_SIZE * PAYLOAD_BUFFER_SIZE);

  Serial.printf("Starting LMIC...\n");

  os_init();    // initialize lmic run-time environment on core 1
  LMIC_reset(); // initialize lmic MAC
  LMIC_setLinkCheckMode(0);
  // This tells LMIC to make the receive windows bigger, in case your clock is
  // faster or slower. This causes the transceiver to be earlier switched on,
  // so consuming more power. You may sharpen (reduce) CLOCK_ERROR_PERCENTAGE
  // in src/lmic_config.h if you are limited on battery.
  LMIC_setClockError(MAX_CLOCK_ERROR * CLOCK_ERROR_PROCENTAGE / 100);
  // Set the data rate to Spreading Factor 7.  This is the fastest supported
  // rate for 125 kHz channels, and it minimizes air time and battery power.
  // Set the transmission power to 14 dBi (25 mW).
  LMIC_setDrTxpow(LORA_DEFAULT_SF, 14);

  if (!LMIC_startJoining()) { // start joining
    Serial.printf("Already joined\n");
  }

  return ESP_OK; // continue main program
}

void lora_enqueuedata(MessageBuffer_t *message, sendprio_t prio) {
  // enqueue message in LORA send queue
  BaseType_t ret;
  MessageBuffer_t DummyBuffer;
  switch (prio) {
  case prio_high:
    // clear space in queue if full, then fallthrough to normal
    if (uxQueueSpacesAvailable == 0)
      xQueueReceive(LoraSendQueue, &DummyBuffer, (TickType_t)0);
  case prio_normal:
    ret = xQueueSendToFront(LoraSendQueue, (void *)message, (TickType_t)0);
    break;
  case prio_low:
  default:
    ret = xQueueSendToBack(LoraSendQueue, (void *)message, (TickType_t)0);
    break;
  }
  if (ret != pdTRUE)
    Serial.printf("LORA sendqueue is full\n");
}

void lora_queuereset(void) { xQueueReset(LoraSendQueue); }

void initLora() {
  assert(lora_stack_init() == ESP_OK);
}
void loraLoop() {
   os_runloop_once();
   delay(2);
}