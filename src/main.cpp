

#include <esp_task_wdt.h>

#include "BLEDevice.h"
#define DeviceName "Flux2"

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#include <SPI.h>
#include <Wire.h>


#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())

#define LED_ON LOW
#define LED_OFF HIGH

//See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN 22 // Pin D2 mapped to pin GPI22/ADC12 of ESP32, control on-board LED

#define PIN_D0 0 // Pin D0 mapped to pin GPIO0/BOOT/ADC11/TOUCH1 of ESP32
#define PIN_D1 1 // Pin D1 mapped to pin GPIO1/TX0 of ESP32
#define PIN_D2 2 // Pin D2 mapped to pin GPIO2/ADC12/TOUCH2 of ESP32
#define PIN_D3 3 // Pin D3 mapped to pin GPIO3/RX0 of ESP32
#define PIN_D4 4 // Pin D4 mapped to pin GPIO4/ADC10/TOUCH0 of ESP32
#define PIN_D5 5 // Pin D5 mapped to pin GPIO5/SPISS/VSPI_SS of ESP32
#define PIN_D6 6 // Pin D6 mapped to pin GPIO6/FLASH_SCK of ESP32
#define PIN_D7 7 // Pin D7 mapped to pin GPIO7/FLASH_D0 of ESP32
#define PIN_D8 8 // Pin D8 mapped to pin GPIO8/FLASH_D1 of ESP32
#define PIN_D9 9 // Pin D9 mapped to pin GPIO9/FLASH_D2 of ESP32

#define PIN_D10 10 // Pin D10 mapped to pin GPIO10/FLASH_D3 of ESP32
#define PIN_D11 11 // Pin D11 mapped to pin GPIO11/FLASH_CMD of ESP32
#define PIN_D12 12 // Pin D12 mapped to pin GPIO12/HSPI_MISO/ADC15/TOUCH5/TDI of ESP32
#define PIN_D13 13 // Pin D13 mapped to pin GPIO13/HSPI_MOSI/ADC14/TOUCH4/TCK of ESP32
#define PIN_D14 14 // Pin D14 mapped to pin GPIO14/HSPI_SCK/ADC16/TOUCH6/TMS of ESP32
#define PIN_D15 15 // Pin D15 mapped to pin GPIO15/HSPI_SS/ADC13/TOUCH3/TDO of ESP32
#define PIN_D16 16 // Pin D16 mapped to pin GPIO16/TX2 of ESP32
#define PIN_D17 17 // Pin D17 mapped to pin GPIO17/RX2 of ESP32
#define PIN_D18 18 // Pin D18 mapped to pin GPIO18/VSPI_SCK of ESP32
#define PIN_D19 19 // Pin D19 mapped to pin GPIO19/VSPI_MISO of ESP32

#define PIN_D21 21 // Pin D21 mapped to pin GPIO21/SDA of ESP32
#define PIN_D22 22 // Pin D22 mapped to pin GPIO22/SCL of ESP32
#define PIN_D23 23 // Pin D23 mapped to pin GPIO23/VSPI_MOSI of ESP32
#define PIN_D24 24 // Pin D24 mapped to pin GPIO24 of ESP32
#define PIN_D25 25 // Pin D25 mapped to pin GPIO25/ADC18/DAC1 of ESP32
#define PIN_D26 26 // Pin D26 mapped to pin GPIO26/ADC19/DAC2 of ESP32
#define PIN_D27 27 // Pin D27 mapped to pin GPIO27/ADC17/TOUCH7 of ESP32

#define PIN_D32 32 // Pin D32 mapped to pin GPIO32/ADC4/TOUCH9 of ESP32
#define PIN_D33 33 // Pin D33 mapped to pin GPIO33/ADC5/TOUCH8 of ESP32
#define PIN_D34 34 // Pin D34 mapped to pin GPIO34/ADC6 of ESP32

//Only GPIO pin < 34 can be used as output. Pins >= 34 can be only inputs
//See .../cores/esp32/esp32-hal-gpio.h/c
//#define digitalPinIsValid(pin)          ((pin) < 40 && esp32_gpioMux[(pin)].reg)
//#define digitalPinCanOutput(pin)        ((pin) < 34 && esp32_gpioMux[(pin)].reg)
//#define digitalPinToRtcPin(pin)         (((pin) < 40)?esp32_gpioMux[(pin)].rtc:-1)
//#define digitalPinToAnalogChannel(pin)  (((pin) < 40)?esp32_gpioMux[(pin)].adc:-1)
//#define digitalPinToTouchChannel(pin)   (((pin) < 40)?esp32_gpioMux[(pin)].touch:-1)
//#define digitalPinToDacChannel(pin)     (((pin) == 25)?0:((pin) == 26)?1:-1)

#define PIN_D35 35 // Pin D35 mapped to pin GPIO35/ADC7 of ESP32
#define PIN_D36 36 // Pin D36 mapped to pin GPIO36/ADC0/SVP of ESP32
#define PIN_D39 39 // Pin D39 mapped to pin GPIO39/ADC3/SVN of ESP32

#define PIN_RX0 3 // Pin RX0 mapped to pin GPIO3/RX0 of ESP32
#define PIN_TX0 1 // Pin TX0 mapped to pin GPIO1/TX0 of ESP32

#define PIN_SCL 22 // Pin SCL mapped to pin GPIO22/SCL of ESP32
#define PIN_SDA 21 // Pin SDA mapped to pin GPIO21/SDA of ESP32

#include <WiFiUdp.h>
/* Specify the Service UUID of Server */
static BLEUUID serviceUUID("00001826-0000-1000-8000-00805f9b34fb");
/* Specify the Characteristic UUID of Server */
static BLEUUID    charSpeedUUID("00002ad2-0000-1000-8000-00805f9b34fb");
static BLEUUID    charPowerUUID("00002ad9-0000-1000-8000-00805f9b34fb");
static BLEUUID    charStatusUUID("00002ada-0000-1000-8000-00805f9b34fb");

static BLERemoteCharacteristic* pRemoteCharacteristic=nullptr;
static BLERemoteCharacteristic* pResistanceCharacteristic=nullptr;
static BLERemoteCharacteristic* pStatusRemoteCharacteristic=nullptr;

static BLEAdvertisedDevice* myDevice;
bool connected=false;
bool doConnect=false;

static void notifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  Serial.print("Notify callback for characteristic ");
  Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  
    uint16_t value = 0;

  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  value = *((uint16_t *)pData+1);
  Serial.println(value);
}

static void notifyStatusCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  //Serial.print("Notify Status callback for characteristic ");
  //Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
  
    uint8_t value = 0;

  Serial.print("message reply data length ");
  Serial.println(length);
  Serial.print("data: ");
  value = *((uint8_t *)pData);
  Serial.println(value);
  value = *((uint8_t *)pData+1);
  Serial.println(value);
  value = *((uint8_t *)pData+2);
  Serial.println(value);
}

static void notifyResistanceCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                            uint8_t* pData, size_t length, bool isNotify)
{
  Serial.print("Resistance Notify callback for characteristic ");
  
    uint8_t value = 0;

  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  value = *((uint8_t *)pData+0);
  Serial.print(value);
  value = *((uint8_t *)pData+1);
  Serial.print(value);
  value = *((uint8_t *)pData+2);
  Serial.println(value);
}


class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient* pclient)
  {
    
    Serial.println("Connect");
  }

  void onDisconnect(BLEClient* pclient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};

/* Start connection to the BLE Server */
bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());
    
  BLEClient*  pClient  = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

    /* Connect to the remote BLE Server */
  pClient->connect(myDevice->getAddress(),BLE_ADDR_TYPE_RANDOM);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");

    /* Obtain a reference to the service we are after in the remote BLE server */
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println("00001826-0000-1000-8000-00805f9b34fb");
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

	std::map<std::string, BLERemoteCharacteristic*>* characteristics = pRemoteService->getCharacteristics();
  for(const auto & c: *characteristics)
  {
      Serial.println(c.first.c_str());
      Serial.println(c.second->toString().c_str());
  }

  /* Obtain a reference to the characteristic in the service of the remote BLE server */
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charSpeedUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charSpeedUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if(pRemoteCharacteristic->canNotify())
  {
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  }
  
  pStatusRemoteCharacteristic = pRemoteService->getCharacteristic(charStatusUUID);
  if (pStatusRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charStatusUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  if(pStatusRemoteCharacteristic->canNotify())
  {
    pStatusRemoteCharacteristic->registerForNotify(notifyStatusCallback);

  }
  pResistanceCharacteristic = pRemoteService->getCharacteristic(charPowerUUID);
  if (pResistanceCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charPowerUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

    pResistanceCharacteristic->registerForNotify(notifyResistanceCallback);
    connected = true;
    return true;
}

/* Scan for BLE servers and find the first one that advertises the service we are looking for. */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
 /* Called for each advertising BLE server. */
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    /* We have found a device, let us now see if it contains the service we are looking for. */
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID))
    {
      BLEDevice::getScan()->stop();
    Serial.print("Scanning stopped ");
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
    }
  }
};


void setup()
{
  //set led pin as output
  pinMode(LED_BUILTIN, OUTPUT);
  
  digitalWrite(LED_BUILTIN,LED_ON);
  Serial.begin(115200);
  while (!Serial);

  delay(200);
  digitalWrite(LED_BUILTIN,LED_ON);

  Serial.println(F("\nStarting " DeviceName)); 

  Serial.setDebugOutput(false);

  digitalWrite(LED_BUILTIN,LED_ON);
  //unsigned long startedAt = millis();


  digitalWrite(LED_BUILTIN, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.


BLEDevice::init("ESP32-BLE-Client");

  /* Retrieve a Scanner and set the callback we want to use to be informed when we
     have detected a new device.  Specify that we want active scanning and start the
     scan to run for 5 seconds. */
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
BLEDevice::getScan()->start(0);

    Serial.print("BLE Scan started: ");
  digitalWrite(LED_BUILTIN,LED_ON);
  

}
#pragma pack(push,1)
#pragma pack(1)
struct powerMessage {
  uint8_t opCode;
  uint16_t resistance; 
};
#pragma pack(pop)
enum opCodes
{
  ocRequesControl=0x00,
  ocReset=0x01,
  ocSetTargetSpeed=0x02,
  ocSetTargetInclination=0x03,
  ocSetTargetResistaneLevel=0x04,
  ocSetTargetPower=0x05,
  ocStart=0x07,
  ocStop =0x08,
  ocResponseCode = 0x80
} ;
bool firstTime = true;
void loop()
{
 if(doConnect)
 {
  
      connectToServer();
  doConnect = false;
 } 
 if(connected)
 {
   if(pResistanceCharacteristic)
   {
     uint8_t *result;
     if (firstTime)
     {
         Serial.println("do Init    dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
       powerMessage msg;
       msg.opCode = ocReset;
       msg.resistance = 100;
       pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1,true);
       sleep(1);
       msg.opCode = ocRequesControl;
       msg.resistance = 100;
       pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1,true);
       sleep(1);
       msg.opCode = ocStart;
       msg.resistance = 100;
       pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1,true);
       sleep(1);
       firstTime=false;
     }
     powerMessage msg;
     msg.opCode = ocSetTargetResistaneLevel;
     msg.resistance = 8;
     pResistanceCharacteristic->writeValue((uint8_t *)&msg, sizeof(msg),true);
Serial.println("update");
     result = pResistanceCharacteristic->readRawData();
     if(result)
       {
         Serial.print(result[0]);
         Serial.print(result[1]);
         Serial.println(result[2]);
       }
       else{
        Serial.println("noResponse");
       }
       sleep(1);
   }
 }
}

void doSleep()
{
   //Configure GPIO33 as ext0 wake up source for HIGH logic level
 // esp_sleep_enable_ext0_wakeup((gpio_num_t)ZeroPin,0);

  //Go to sleep now
  esp_deep_sleep_start();
}