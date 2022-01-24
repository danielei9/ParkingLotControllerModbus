#include <Arduino.h>
#include <WiFi.h>
#include <ModbusRTU.h>
#include <Ticker.h>
#include <lora.h>
#include <EEPROM.h>

#include <EEPROM.h>
/**
 * @brief INPUT_1
 *      1Input pin 23 como entrada
 *      coment this line para configurar  pin 23 como salida
 */

#define ONE_WAY
/**
 * @brief ONE_WAY
 *      solo 1 carril 1 y 2 en el mismo carril si esta varible esta comentada
 *      actua como 1 para entrar y 2 para salir 
 */
#define INPUT_1

/**
 * @brief POSITIVE_LOGIC
 *    se activa con logica positiva de normal IDLE 0 cuando detecta  1 
 */
#define POSITIVE_LOGIC

// MODBUS CONFIG **********************************************
//*************************************************************
#define MODBUS_RX 25
#define MODBUS_TX 4
#define MODBUS_RXTX 12
#define MODBUS_SLAVE_ID 42
#define MODBUS_REGN_PLACES 8
#define MODBUS_REGN_BRIGHTNESS 10

#define SET_CURRENT_VALUE 0xA0
#define SET_MAX_VALUE 0xA1
#define SET_BRIGHTNESS 0xA2
#define GET_CURRENT_VALUE 0xB0
#define GET_MAX_VALUE 0xB1
#define GET_BRIGHTNESS 0xB2
ModbusRTU mb;

// INPUTS CONFIG **********************************************
//*************************************************************
#define INPUT_BAND_1 23
#define INPUT_BAND_2 34

// LORA CONFIG **********************************************
//*************************************************************
#define SENDDATA_PORT 1
#define RESPONSE_PORT 2
#define INFO_PORT 3
Ticker updateTicker;
Ticker resetTicker;
Ticker statusTicker;

#define EEPROM_SIZE 256
bool shouldSendStatus = true;
int counter = 0;
int maxParkingValue = 10;
int brightness = 16;
bool updateCounter = true;
uint8_t res1, res2;
//****************************************************************
//****************************************************************
// one Way definitions
enum BandPosition
{
    IDLE,
    FIRST_STEP,
    SECOND_STEP,
    FIRST_RELEASE,
    SECOND_RELEASE
};
BandPosition firstBand = IDLE, secondBand = IDLE;


/**
 * declarations
 */


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
void checkInput();
void checkOutput();
void readSavedData();
void checkBands(int band1Value, int band2Value);
void checkInputs();
void increaseCounter();
void resetDevice();
void setSendStatus();
void sendStatus();
//**************** SETUP ********************
//*******************************************
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


/**
 * LOOP ***************************************************
 * ******************************************************** 
 */
void loop()
{
#ifdef ONE_WAY
    checkInputs();
    checkBands(b1LastState, b2LastState);
#endif
#ifndef ONE_WAY
    checkInput();
    checkOutput();
#endif

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
/**
 * one_way ***************************************************
 * ******************************************************** 
 */
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
/**
 * helpfull functions *************************************
 * ******************************************************** 
 */
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
/**
 * @brief sendData 
 *      Send data to Lora 
 * @param port port number to send
 * @param buff buffer data to send
 * @param size size of buff
 */
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
/**
 * @brief command_callback 
 *     callback when we recive some data 
 * 
 * @param buff recived buff
 * @param size size of recived buff
 */
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
/**
 * @brief readSavedData 
 *  read the data saved in eeprom
 */
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
/**
 * @brief sendStatus 
 *  send status to lora
 */
void sendStatus()
{
    byte buf[3];
    buf[0] = 10;
    buf[1] = res1;
    buf[2] = res2;
    sendData(INFO_PORT, buf, sizeof(buf));
}
// ONE WAY ******************************************
//***************************************************
BandPosition checkBand(BandPosition b, int firstValue, int secondValue) 
{
    if (b == IDLE && firstValue == LOW && secondValue == HIGH)
    {
        return FIRST_STEP;
    }
    else if (b == FIRST_STEP && firstValue == LOW && secondValue == LOW)
    {
        return SECOND_STEP;
    }
    else if (b == SECOND_STEP && firstValue == HIGH && secondValue == LOW)
    {
        return FIRST_RELEASE;
    }
    else if (b == FIRST_RELEASE && firstValue == HIGH && secondValue == HIGH)
    {
        return SECOND_RELEASE;
    }
    else if(firstValue == HIGH && secondValue == HIGH)
    {
        return IDLE;
    }    
    return b;
}

void checkInputs()
{
    int b1Value = digitalRead(INPUT_BAND_1);
    int b2Value = digitalRead(INPUT_BAND_2);

    if(b1Value == b1LastState)
    {
        b1LastStateChange = millis();
    }
    else if (millis() - b1LastStateChange > DEBOUNCE_TH)
    {
        b1LastState = b1Value;
        b1LastStateChange = millis();
    }

    if(b2Value == b2LastState)
    {
        b2LastStateChange = millis();
    }
    else if (millis() - b2LastStateChange > DEBOUNCE_TH)
    {
        b2LastState = b2Value;
        b2LastStateChange = millis();
    }
}

void checkBands(int band1Value, int band2Value)
{
    int previousCounter = counter;

    if (secondBand == IDLE)
    {
        BandPosition fbPrev = firstBand;
        firstBand = checkBand(firstBand, band1Value, band2Value);
        if(fbPrev != firstBand)
        {
            Serial.printf("change first band to %d\n", firstBand);
        }
        if(firstBand == SECOND_RELEASE)
        {
            counter +=1;            
        }
    }
    if (firstBand == IDLE)
    {
        BandPosition fbPrev = secondBand;
        secondBand = checkBand(secondBand, band2Value, band1Value);
        if(fbPrev != secondBand)
        {
            Serial.printf("change second band to %d\n", secondBand);
        }
        if(secondBand == SECOND_RELEASE)
        {
            counter -= 1;            
        }
    }

    if (previousCounter != counter)
    {
        firstBand = IDLE;
        secondBand = IDLE;
        byte buf[4];
        buf[0] = highByte(counter);
        buf[1] = lowByte(counter);
        buf[2] = highByte(maxParkingValue);
        buf[3] = lowByte(maxParkingValue);
        sendData(SENDDATA_PORT, buf, sizeof(buf));
        saveConfig();
        Serial.print("Setting counter to ");
        Serial.println(counter);
    }
}