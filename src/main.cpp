#include <Arduino.h>
#include <WiFi.h>
#include <ModbusRTU.h>
#include <Ticker.h>
#include <lora.h>
#include <EEPROM.h>
/**
 * @brief INPUT_1
 *      1Input pin 23 como entrada
 *      coment this line para configurar  pin 23 como salida
 */
#define INPUT_1

/**
 * @brief POSITIVE_LOGIC
 *    se activa con logica positiva de normal IDLE 0 cuando detecta  1 
 */
#define POSITIVE_LOGIC 

#define MODBUS_RX 25
#define MODBUS_TX 4
#define MODBUS_RXTX 12
#define MODBUS_SLAVE_ID 42
#define MODBUS_REGN_PLACES 8
#define MODBUS_REGN_BRIGHTNESS 10

#define INPUT_BAND_1 23
#define INPUT_BAND_2 34

#define SENDDATA_PORT 1
#define RESPONSE_PORT 2
#define INFO_PORT 3

#define EEPROM_SIZE 256

#define SET_CURRENT_VALUE 0xA0
#define SET_MAX_VALUE 0xA1
#define SET_BRIGHTNESS 0xA2
#define GET_CURRENT_VALUE 0xB0
#define GET_MAX_VALUE 0xB1
#define GET_BRIGHTNESS 0xB2

Ticker updateTicker;
Ticker resetTicker;
Ticker statusTicker;
ModbusRTU mb;

bool shouldSendStatus = true;

int counter = 0;
int maxParkingValue = 10;
int brightness = 16;
bool updateCounter = true;

uint8_t res1, res2;

void resetDevice()
{
    ESP.restart();
}

void setSendStatus()
{
    shouldSendStatus = true;
}

void increaseCounter()
{
    updateCounter = true;
}

void saveConfig()
{
    Serial.println("saving config");
    uint16_t addr = 0;
    EEPROM.put(addr, maxParkingValue);
    addr += sizeof(maxParkingValue);
    EEPROM.put(addr, counter);
    addr += sizeof(counter);
    EEPROM.put(addr, brightness);
    addr += sizeof(brightness);
    EEPROM.commit();
}

void sendData(uint8_t port, uint8_t *buff, uint8_t size)
{
    MessageBuffer_t message;
    message.MessagePort = port;
    message.MessageSize = size;
    for (int i = 0; i < size; i++)
    {
        message.Message[i] = buff[i];
    }
    lora_enqueuedata(&message, prio_normal);
}

void command_callback(uint8_t *buff, uint8_t size)
{
    if (size < 2)
        return;
    switch (buff[0])
    {
    case SET_CURRENT_VALUE:
    {
        if (size == 3)
        {
            counter = buff[1] * 256 + buff[2];
            Serial.printf("Setting current counter to %d\n", counter);
            sendData(RESPONSE_PORT, buff, size);
            saveConfig();
        }
        break;
    }
    case GET_CURRENT_VALUE:
    {
        byte buf[3];
        buf[0] = buff[0];
        buf[1] = highByte(counter);
        buf[2] = lowByte(counter);
        sendData(RESPONSE_PORT, buf, 3);
        break;
    }
    case SET_MAX_VALUE:
    {
        if (size == 3)
        {
            maxParkingValue = buff[1] * 256 + buff[2];
            Serial.printf("Setting current maxParkingValue to %d\n", maxParkingValue);
            sendData(RESPONSE_PORT, buff, size);
            saveConfig();
        }
        break;
    }
    case GET_MAX_VALUE:
    {
        byte buf[3];
        buf[0] = buff[0];
        buf[1] = highByte(maxParkingValue);
        buf[2] = lowByte(maxParkingValue);
        sendData(RESPONSE_PORT, buf, 3);
        break;
    }
    case SET_BRIGHTNESS:
    {
        if (size == 2)
        {
            brightness = buff[1];
            Serial.printf("Setting current brightness to %d\n", brightness);
            sendData(RESPONSE_PORT, buff, size);
            saveConfig();
        }
        break;
    }
    case GET_BRIGHTNESS:
    {
        byte buf[2];
        buf[0] = buff[0];
        buf[1] = brightness;
        sendData(RESPONSE_PORT, buf, 2);
        break;
    }
    }
}

void readSavedData()
{
    uint16_t addr = 0;
    EEPROM.get(addr, maxParkingValue);
    addr += sizeof(maxParkingValue);
    EEPROM.get(addr, counter);
    addr += sizeof(counter);
    EEPROM.get(addr, brightness);
    addr += sizeof(brightness);
}

void sendStatus()
{
    byte buf[3];
    buf[0] = 10;
    buf[1] = res1;
    buf[2] = res2;
    sendData(INFO_PORT, buf, sizeof(buf));
}

void setup()
{
    counter = 0;
    pinMode(INPUT_BAND_1, INPUT_PULLUP);
    pinMode(INPUT_BAND_2, INPUT_PULLUP);
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, MODBUS_TX, MODBUS_RX);
    mb.begin(&Serial2, MODBUS_RXTX, true);
    mb.master();

    EEPROM.begin(EEPROM_SIZE);
    readSavedData();
    initLora();

    updateTicker.attach(0.5, increaseCounter);
    resetTicker.once(86400, resetDevice);
    statusTicker.attach(300, setSendStatus);

    Serial.println("start");
}

#define DEBOUNCE_TH 100
unsigned long b1LastStateChange = 0;
unsigned long b2LastStateChange = 0;
int b1LastState = LOW;
int b2LastState = LOW;
bool firstTimeIn = false;
bool firstTimeOut = false;
bool keyIN = false;
bool keyOUT = false;
unsigned long time0Input, time1Input;
unsigned long time0Out, time1Out;

void checkInput()
{
    //Lectura de las bandas
    int b1Value = digitalRead(INPUT_BAND_1); //detecta con logica positiva
    if (b1Value == HIGH)
    {
        updateCounter = true;
        //El boton esta activado
        if (!firstTimeIn)
        {
            firstTimeIn = true;
            time0Input = millis();
            keyIN = true;
        } //Se detecta la primera activacion y se guarda ese tiempo
        else
        {
            time1Input = millis();
            if ((time1Input - time0Input) > 200 && keyIN)
            { //supera los 200ms  activamos
                keyIN = false;
#ifdef INPUT_1
                counter++;
#elif
                counter--;
#endif Serial.print("Boton activado: ");
                Serial.println(counter);
            }
        }
    }
    else
    {
        firstTimeIn = false;
        //desactivado
    }
}
void checkOutput()
{
    //Lectura de las bandas
    int b2Value = digitalRead(INPUT_BAND_2); //detecta con logica positiva
    if (b2Value == HIGH)
    {
        updateCounter = true;
        //El boton esta activado
        if (!firstTimeOut)
        {
            firstTimeOut = true;
            time0Out = millis();
            keyOUT = true;
        } //Se detecta la primera activacion y se guarda ese tiempo
        else
        {
            time1Out = millis();
            if ((time1Out - time0Out) > 200 && keyOUT)
            { //supera los 200ms  activamos
                keyOUT = false;
#ifdef INPUT_1
                counter--;
#elif
                counter++;
#endif
                Serial.print("Boton activado: ");
                Serial.println(counter);
            }
        }
    }
    else
    {
        firstTimeOut = false;
        //desactivado
    }
}
void loop()
{
    checkInput();
    checkOutput();
    if (updateCounter)
    {
        int valueToSend = maxParkingValue - counter;
        if (valueToSend < 0)
        {
            valueToSend = 0;
        }
        if (valueToSend > maxParkingValue)
        {
            valueToSend = maxParkingValue;
        }
        res1 = mb.writeHreg(MODBUS_SLAVE_ID, MODBUS_REGN_PLACES, valueToSend);
        res2 = mb.writeHreg(MODBUS_SLAVE_ID, MODBUS_REGN_BRIGHTNESS, brightness);
        updateCounter = false;
    }
    if (shouldSendStatus)
    {
        sendStatus();
        shouldSendStatus = false;
    }
    mb.task();
    loraLoop();
    yield();
}