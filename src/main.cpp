// usage:
// normaly it gets an IP through DHCP, if this does not work:
// power through USB and do not connect to an ethernet --> this will start the accesspoint
// doubleclick might also start the AP
// connect to FLUX2_ and visit: http//192.168.4.1
// configure an IP and SAVE, this should be it
// default now 192.168.0.45
/*
modify async wifi manager so that it does not try to connect after saving
        //notify that configuration has changed and any optional parameters should be saved
        if (_savecallback != NULL)
        {
          //todo: check if any custom parameters actually exist, and check if they really changed maybe
          _savecallback();
        }
        break;
*/
#include <ArduinoOTA.h>
#include <esp_task_wdt.h>
#include <ESP32Encoder.h>
#define DeviceName "Flux2"
#include "BLEDevice.h"
#undef WIFIManager
#ifdef WIFIManager
#include <BLEUtils.h>
#include <BLEClient.h>
#include <HeadwindController.h>

#define ESP_ASYNC_WIFIMANAGER_VERSION_MIN_TARGET "ESPAsync_WiFiManager v1.9.2"

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ESPASYNC_WIFIMGR_LOGLEVEL_ 3

#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>

#endif

#define ETH_CLK_MODE ETH_CLOCK_GPIO17_OUT
#define ETH_PHY_POWER 12

#include <ETH.h>

static bool eth_connected = false;
static bool eth_link = false;

#include <SPI.h>
#include <Wire.h>

// From v1.1.1
#ifdef WIFIManager
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#endif

// Use LittleFS
//#include "FS.h"

#include <LittleFS.h>

TaskHandle_t Task0;

void BluetoothLoop( void * parameter );

FS *filesystem = &LittleFS;
#define FileFS LittleFS
#define FS_Name "LittleFS"

BLEClient *pClient = nullptr;
#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())
BLERemoteService* pRemoteService;

// SSID and PW for Config Portal
String ssid = DeviceName "_" + String(ESP_getChipId(), HEX);
String password;

// SSID and PW for your Router
String Router_SSID;
String Router_Pass;

#define LED_ON HIGH
#define LED_OFF LOW

// See file .../hardware/espressif/esp32/variants/(esp32|doitESP32devkitV1)/pins_arduino.h
#define LED_BUILTIN 13 // Pin D2 mapped to pin GPI22/ADC12 of ESP32, control on-board LED

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
static BLEUUID charSpeedUUID("00002ad2-0000-1000-8000-00805f9b34fb");
static BLEUUID charPowerUUID("00002ad9-0000-1000-8000-00805f9b34fb");
static BLEUUID charStatusUUID("00002ada-0000-1000-8000-00805f9b34fb");
static BLEAddress headwind1Address("dd:d5:42:30:32:4f");
static BLEAddress headwind2Address("dc:f8:0d:3c:c7:b9");

unsigned long lastUpdateTimeheadwind = 0;

static BLERemoteCharacteristic *pRemoteCharacteristic = nullptr;
static BLERemoteCharacteristic *pResistanceCharacteristic = nullptr;
static BLERemoteCharacteristic *pStatusRemoteCharacteristic = nullptr;

static BLEAdvertisedDevice *myDevice=nullptr;
bool connected = false;
bool doConnect = false;


ulong lastWorkingTime = 0;
ulong lastUpdateTime = 0;
ulong updatePeriod = 100;
unsigned long connectionTimeout = 0;

#include "debounceButton.h"
#include "loadCell.h"
#include "HeadwindController.h"

// setting pins for the load cell for braking system
const int BRAKE_DOUT_PIN = 13;
const int BRAKE_SCK_PIN = 16;
loadCell *brakeSensor;
const int brakeScaleFactor = -5000; // modify this to change the scale factor to adjust the sensitivity of the sensor
const unsigned long period = 300;
const int ZeroPin = 34;
debounceButton zeroButton(ZeroPin);

float speed;

// Pins for encoder to get steering wheel angle
const int I_PIN = 2;
const int A_PIN = 14;
const int B_PIN = 15;
int64_t encoderCount;
int64_t oldEncoderCount;
const float angleFactor = 0.008727;
const int ENCODER_OFFSET = 0;
ESP32Encoder encoder;

#define MIN_AP_PASSWORD_SIZE 8

#define SSID_MAX_LEN 32
// From v1.0.10, WPA2 passwords can be up to 63 characters long.
#define PASS_MAX_LEN 64

typedef struct
{
    char wifi_ssid[SSID_MAX_LEN];
    char wifi_pw[PASS_MAX_LEN];
} WiFi_Credentials;

typedef struct
{
    String wifi_ssid;
    String wifi_pw;
} WiFi_Credentials_String;

#define NUM_WIFI_CREDENTIALS 2

// Assuming max 49 chars
#define TZNAME_MAX_LEN 50
#define TIMEZONE_MAX_LEN 50

typedef struct
{
    WiFi_Credentials WiFi_Creds[NUM_WIFI_CREDENTIALS];
    char TZ_Name[TZNAME_MAX_LEN]; // "America/Toronto"
    char TZ[TIMEZONE_MAX_LEN];    // "EST5EDT,M3.2.0,M11.1.0"
    uint16_t checksum;
} WM_Config;

WM_Config WM_config;

#define CONFIG_FILENAME F("/wifi_cred.dat")
//////

// Indicates whether ESP has WiFi credentials saved from previous session, or double reset detected
bool initialConfig = false;

// Use false if you don't like to display Available Pages in Information Page of Config Portal
// Comment out or use true to display Available Pages in Information Page of Config Portal
// Must be placed before #include <ESP_WiFiManager.h>
#define USE_AVAILABLE_PAGES true

// From v1.0.10 to permit disable/enable StaticIP configuration in Config Portal from sketch. Valid only if DHCP is used.
// You'll loose the feature of dynamically changing from DHCP to static IP, or vice versa
// You have to explicitly specify false to disable the feature.
//#define USE_STATIC_IP_CONFIG_IN_CP          false

// Use false to disable NTP config. Advisable when using Cellphone, Tablet to access Config Portal.
// See Issue 23: On Android phone ConfigPortal is unresponsive (https://github.com/khoih-prog/ESP_WiFiManager/issues/23)
#define USE_ESP_WIFIMANAGER_NTP true

// Just use enough to save memory. On ESP8266, can cause blank ConfigPortal screen
// if using too much memory
#define USING_AFRICA false
#define USING_AMERICA false
#define USING_ANTARCTICA false
#define USING_ASIA false
#define USING_ATLANTIC false
#define USING_AUSTRALIA false
#define USING_EUROPE true
#define USING_INDIAN false
#define USING_PACIFIC false
#define USING_ETC_GMT false

// Use true to enable CloudFlare NTP service. System can hang if you don't have Internet access while accessing CloudFlare
// See Issue #21: CloudFlare link in the default portal (https://github.com/khoih-prog/ESP_WiFiManager/issues/21)
#define USE_CLOUDFLARE_NTP false

// New in v1.0.11
#define USING_CORS_FEATURE true
//////

// Use USE_DHCP_IP == true for dynamic DHCP IP, false to use static IP which you have to change accordingly to your network
#if (defined(USE_STATIC_IP_CONFIG_IN_CP) && !USE_STATIC_IP_CONFIG_IN_CP)
// Force DHCP to be true
#if defined(USE_DHCP_IP)
#undef USE_DHCP_IP
#endif
#define USE_DHCP_IP true
#else
// You can select DHCP or Static IP here
#define USE_DHCP_IP true
//#define USE_DHCP_IP     false
#endif

#if (USE_DHCP_IP)
// Use DHCP
//#warning Using DHCP IP
IPAddress stationIP = IPAddress(0, 0, 0, 0);
IPAddress gatewayIP = IPAddress(192, 168, 2, 1);
IPAddress netMask = IPAddress(255, 255, 255, 0);
#else
// Use static IP
#warning Using static IP

#ifdef ESP32
IPAddress stationIP = IPAddress(192, 168, 0, 232);
#else
IPAddress stationIP = IPAddress(192, 168, 0, 186);
#endif

IPAddress gatewayIP = IPAddress(192, 168, 0, 1);
IPAddress netMask = IPAddress(255, 255, 255, 0);
#endif

#ifdef WIFIManager
#define USE_CONFIGURABLE_DNS true

IPAddress dns1IP = gatewayIP;
IPAddress dns2IP = IPAddress(8, 8, 8, 8);
#else
#define USE_CONFIGURABLE_DNS false
#endif

#define USE_CUSTOM_AP_IP false

IPAddress APStaticIP = IPAddress(192, 168, 100, 1);
IPAddress APStaticGW = IPAddress(192, 168, 100, 1);
IPAddress APStaticSN = IPAddress(255, 255, 255, 0);

#include <ESPAsync_WiFiManager.h> //https://github.com/khoih-prog/ESPAsync_WiFiManager

#define HTTP_PORT 80


WiFi_AP_IPConfig WM_AP_IPconfig;
WiFi_STA_IPConfig WM_STA_IPconfig;

HeadwindController *headwind1=nullptr;
HeadwindController *headwind2=nullptr;
void initAPIPConfigStruct(WiFi_AP_IPConfig &in_WM_AP_IPconfig)
{
    in_WM_AP_IPconfig._ap_static_ip = APStaticIP;
    in_WM_AP_IPconfig._ap_static_gw = APStaticGW;
    in_WM_AP_IPconfig._ap_static_sn = APStaticSN;
}

void initSTAIPConfigStruct(WiFi_STA_IPConfig &in_WM_STA_IPconfig)
{
    in_WM_STA_IPconfig._sta_static_ip = stationIP;
    in_WM_STA_IPconfig._sta_static_gw = gatewayIP;
    in_WM_STA_IPconfig._sta_static_sn = netMask;
#if USE_CONFIGURABLE_DNS
    in_WM_STA_IPconfig._sta_static_dns1 = dns1IP;
  in_WM_STA_IPconfig._sta_static_dns2 = dns2IP;
#endif
}

void displayIPConfigStruct(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
    LOGERROR3(F("stationIP ="), in_WM_STA_IPconfig._sta_static_ip, ", gatewayIP =", in_WM_STA_IPconfig._sta_static_gw);
    LOGERROR1(F("netMask ="), in_WM_STA_IPconfig._sta_static_sn);
#if USE_CONFIGURABLE_DNS
    LOGERROR3(F("dns1IP ="), in_WM_STA_IPconfig._sta_static_dns1, ", dns2IP =", in_WM_STA_IPconfig._sta_static_dns2);
#endif
}

void configWiFi(WiFi_STA_IPConfig in_WM_STA_IPconfig)
{
#if USE_CONFIGURABLE_DNS
    // Set static IP, Gateway, Subnetmask, DNS1 and DNS2. New in v1.0.5
  WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn, in_WM_STA_IPconfig._sta_static_dns1, in_WM_STA_IPconfig._sta_static_dns2);
#else
    // Set static IP, Gateway, Subnetmask, Use auto DNS1 and DNS2.
    WiFi.config(in_WM_STA_IPconfig._sta_static_ip, in_WM_STA_IPconfig._sta_static_gw, in_WM_STA_IPconfig._sta_static_sn);
#endif
}

///////////////////////////////////////////

uint8_t connectMultiWiFi()
{
#if ESP32
// For ESP32, this better be 0 to shorten the connect time.
// For ESP32-S2/C3, must be > 500
#if (USING_ESP32_S2 || USING_ESP32_C3)
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS 500L
#else
// For ESP32 core v1.0.6, must be >= 500
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS 800L
#endif
#else
    // For ESP8266, this better be 2200 to enable connect the 1st time
#define WIFI_MULTI_1ST_CONNECT_WAITING_MS 2200L
#endif

#define WIFI_MULTI_CONNECT_WAITING_MS 500L

    uint8_t status;

    // WiFi.mode(WIFI_STA);

    LOGERROR(F("ConnectMultiWiFi with :"));
#ifdef WIFIManager
    if ((Router_SSID != "") && (Router_Pass != ""))
  {
    LOGERROR3(F("* Flash-stored Router_SSID = "), Router_SSID, F(", Router_Pass[0] = "), Router_Pass[0]);
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW[0] = "), Router_Pass[0]);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());
  }

  for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
  {
    // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
    if ((String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE))
    {
      LOGERROR3(F("* Additional SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW[0] = "), WM_config.WiFi_Creds[i].wifi_pw[0]);
    }
  }

  LOGERROR(F("Connecting MultiWifi..."));

  // WiFi.mode(WIFI_STA);

#if !USE_DHCP_IP
  // New in v1.4.0
  configWiFi(WM_STA_IPconfig);
  //////
#endif

  int i = 0;
  status = wifiMulti.run();
  delay(WIFI_MULTI_1ST_CONNECT_WAITING_MS);

  while ((i++ < 20) && (status != WL_CONNECTED))
  {
    status = WiFi.status();

    if (status == WL_CONNECTED)
      break;
    else
      delay(WIFI_MULTI_CONNECT_WAITING_MS);
  }

  if (status == WL_CONNECTED)
  {
    LOGERROR1(F("WiFi connected after time: "), i);
    LOGERROR3(F("SSID:"), WiFi.SSID(), F(",RSSI="), WiFi.RSSI());
    LOGERROR3(F("Channel:"), WiFi.channel(), F(",IP address:"), WiFi.localIP());
  }
  else
  {
    LOGERROR(F("WiFi not connected"));
  }
#endif

    return status;
}

#if USE_ESP_WIFIMANAGER_NTP

int calcChecksum(uint8_t *address, uint16_t sizeToCalc)
{
    uint16_t checkSum = 0;

    for (uint16_t index = 0; index < sizeToCalc; index++)
    {
        checkSum += *(((byte *)address) + index);
    }

    return checkSum;
}

bool loadConfigData()
{
    File file = FileFS.open(CONFIG_FILENAME, "r");
    LOGERROR(F("LoadWiFiCfgFile "));

    memset((void *)&WM_config, 0, sizeof(WM_config));

    // New in v1.4.0
    memset((void *)&WM_STA_IPconfig, 0, sizeof(WM_STA_IPconfig));
    //////

    if (file)
    {
        file.readBytes((char *)&WM_config, sizeof(WM_config));

        // New in v1.4.0
        file.readBytes((char *)&WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
        //////

        file.close();
        LOGERROR(F("OK"));

        if (WM_config.checksum != calcChecksum((uint8_t *)&WM_config, sizeof(WM_config) - sizeof(WM_config.checksum)))
        {
            LOGERROR(F("WM_config checksum wrong"));

            return false;
        }

        // New in v1.4.0
        displayIPConfigStruct(WM_STA_IPconfig);
        //////

        return true;
    }
    else
    {
        LOGERROR(F("failed"));

        return false;
    }
}

void saveConfigData()
{
    File file = FileFS.open(CONFIG_FILENAME, "w",true);
    LOGERROR(F("SaveWiFiCfgFile "));

    if (file)
    {
        WM_config.checksum = calcChecksum((uint8_t *)&WM_config, sizeof(WM_config) - sizeof(WM_config.checksum));

        file.write((uint8_t *)&WM_config, sizeof(WM_config));

        displayIPConfigStruct(WM_STA_IPconfig);

        // New in v1.4.0
        file.write((uint8_t *)&WM_STA_IPconfig, sizeof(WM_STA_IPconfig));
        //////

        file.close();
        LOGERROR(F("OK"));
    }
    else
    {
        LOGERROR(F("failed"));
    }
}

void startConfigAP()
{

#ifdef WIFIManager
    digitalWrite(LED_BUILTIN, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

  AsyncWebServer webServer(HTTP_PORT);

#if (USING_ESP32_S2 || USING_ESP32_C3)
  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, NULL, "ConfigOnSwitch");
#else
  DNSServer dnsServer;

  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "ConfigOnSwitch");
#endif

  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////

  // set custom ip for portal
  // ESPAsync_wifiManager.setAPStaticIPConfig(IPAddress(192, 168, 100, 1), IPAddress(192, 168, 100, 1), IPAddress(255, 255, 255, 0));

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESPAsync_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif

  // Check if there is stored WiFi router/password credentials.
  // If not found, device will remain in configuration mode until switched off via webserver.
  Serial.println(F("Opening configuration portal. "));

  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

  // From v1.1.0, Don't permit NULL password
  if ((Router_SSID != "") && (Router_Pass != ""))
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    ESPAsync_wifiManager.setConfigPortalTimeout(120); // If no access point name has been previously entered disable timeout.
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }
  else if (loadConfigData())
  {
    ESPAsync_wifiManager.setConfigPortalTimeout(120); // If no access point name has been previously entered disable timeout.
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal"));
  }
  else
  {
    // Enter CP only if no stored SSID on flash and file
    Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
    initialConfig = true;
  }

  // Starts an access point
  // and goes into a blocking loop awaiting configuration
  if (!ESPAsync_wifiManager.startConfigPortal((const char *)ssid.c_str(), password.c_str()))
  {
    Serial.println(F("Not connected to WiFi but continuing anyway."));
  }
  else
  {
    // if you get here you have connected to the WiFi
    Serial.println(F("connected...yeey :)"));
    Serial.print(F("Local IP: "));
    Serial.println(WiFi.localIP());
  }

  // Only clear then save data if CP entered and with new valid Credentials
  // No CP => stored getSSID() = ""
  if (String(ESPAsync_wifiManager.getSSID(0)) != "" && String(ESPAsync_wifiManager.getSSID(1)) != "")
  {
    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESPAsync_wifiManager.getSSID(i);
      String tempPW = ESPAsync_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ((String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE))
      {
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw);
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
      }
    }

#if USE_ESP_WIFIMANAGER_NTP
    String tempTZ = ESPAsync_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char *TZ_Result = ESPAsync_wifiManager.getTZ(WM_config.TZ_Name);

    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);

    if (strlen(WM_config.TZ_Name) > 0)
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org");
#else
      // configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif


  }
  ESPAsync_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
  saveConfigData();
#endif

    digitalWrite(LED_BUILTIN, LED_OFF); // Turn led off as we are not in configuration mode.
}

void printLocalTime()
{
    struct tm timeinfo;

    getLocalTime(&timeinfo);

    // Valid only if year > 2000.
    // You can get from timeinfo : tm_year, tm_mon, tm_mday, tm_hour, tm_min, tm_sec
    if (timeinfo.tm_year > 100)
    {
        Serial.print("Local Date/Time: ");
        Serial.print(asctime(&timeinfo));
    }
}

#endif

void heartBeatPrint()
{
#if USE_ESP_WIFIMANAGER_NTP
    printLocalTime();
#else
    static int num = 1;

  if (WiFi.status() == WL_CONNECTED)
    Serial.print(F("H")); // H means connected to WiFi
  else
    Serial.print(F("F")); // F means not connected to WiFi

  if (num == 80)
  {
    Serial.println();
    num = 1;
  }
  else if (num++ % 10 == 0)
  {
    Serial.print(F(" "));
  }
#endif
}

void toggleLED()
{
    // toggle state
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                           uint8_t *pData, size_t length, bool isNotify)
{
    //Serial.print("Notify callback for characteristic ");
    //Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());

    uint16_t value = 0;

    // Serial.print(" of data length ");
    // Serial.println(length);
    // Serial.print("data: ");
    value = *((uint16_t *)pData + 1);
    // Serial.println(value);
    speed = value;
}

void encoderSetup()
{
    // Encoder setup
    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encoder.attachHalfQuad(A_PIN, B_PIN);

    digitalWrite(LED_BUILTIN, LED_ON);
}

static void notifyStatusCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                 uint8_t *pData, size_t length, bool isNotify)
{
}

static void notifyResistanceCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                     uint8_t *pData, size_t length, bool isNotify)
{
    Serial.print("Resistance Notify callback for characteristic ");
    uint8_t value = 0;
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    value = *((uint8_t *)pData + 0);
    Serial.print(value);
    value = *((uint8_t *)pData + 1);
    Serial.print(value);
    value = *((uint8_t *)pData + 2);
    Serial.println(value);
}

class MyClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient *pclient)
    {
        Serial.println("Connect");
    }

    void onDisconnect(BLEClient *pclient)
    {
        //connected = false;
        Serial.println("onDisconnect");
        /*   doConnect = true;
         delete pClient;
         pClient = nullptr;*/
    }
};




/* Start connection to the BLE Server */
bool connectToServer()
{
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());

    pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    /* Connect to the remote BLE Server */
    pClient->connect(myDevice->getAddress(), BLE_ADDR_TYPE_RANDOM); // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to flux");

    /* Obtain a reference to the service we are after in the remote BLE server */
    BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr)
    {
        Serial.print("Failed to find our service UUID: ");
        Serial.println("00001826-0000-1000-8000-00805f9b34fb");
        pClient->disconnect();
        return false;
    }
/*
    std::map<std::string, BLERemoteCharacteristic *> *characteristics = pRemoteService->getCharacteristics();
    for (const auto &c : *characteristics)
    {
        Serial.println(c.first.c_str());
        Serial.println(c.second->toString().c_str());
    }*/

    /* Obtain a reference to the characteristic in the service of the remote BLE server */
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charSpeedUUID);
    if (pRemoteCharacteristic == nullptr)
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charSpeedUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }

    if (pRemoteCharacteristic->canNotify())
    {
        pRemoteCharacteristic->registerForNotify(notifyCallback);
    }

    pResistanceCharacteristic = pRemoteService->getCharacteristic(charPowerUUID);
    if (pResistanceCharacteristic == nullptr)
    {
        Serial.print("Failed to find our characteristic UUID: ");
        Serial.println(charPowerUUID.toString().c_str());
        pClient->disconnect();
        return false;
    }

    // we don't nedd this pResistanceCharacteristic->registerForNotify(notifyResistanceCallback);
    connected = true;
    return true;
}

void startScan()
{
    BLEScan *pBLEScan = BLEDevice::getScan();

    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
    BLEDevice::getScan()->start(5);
}
/* Scan for BLE servers and find the first one that advertises the service we are looking for. */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    /* Called for each advertising BLE server. */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        //Serial.print("BLE Advertised Device found: ");
        //Serial.println(advertisedDevice.toString().c_str());
        //advertisedDevice.getServiceUUID();

        //Serial.println((long long)headwind1);
        //Serial.println(headwind1->address->toString().c_str());
        /* We have found a device, let us now see if it contains the service we are looking for. */
        if (myDevice == nullptr && advertisedDevice.haveServiceUUID() &&
            advertisedDevice.isAdvertisingService(serviceUUID)) {

            Serial.println("test1");
            myDevice = new BLEAdvertisedDevice(advertisedDevice);
            Serial.println("found ");
            Serial.println(myDevice->toString().c_str());
            /*
                BLEDevice::getScan()->stop();
                Serial.print("Scanning stopped ");
            */
            doConnect = true;
        }

        if (headwind1->init(advertisedDevice)) {
            Serial.println("Headwind1 gefunden");
        };
        if (headwind2->init(advertisedDevice)) {
            Serial.println("Headwind2 gefunden");
        };
        if (headwind1->isConnected() && headwind2->isConnected() && pClient!=nullptr &&pClient->isConnected())
        {
            BLEDevice::getScan()->stop();
        }

    }
};

// wifi and covise server
const int coverPort = 31319;
const int pluginPort = 31322;
WiFiUDP toCOVER;
IPAddress coverIP((uint32_t)0);

void sendDeviceInfo(IPAddress destinationAddress)
{
    char buffer[100];
    toCOVER.beginPacket(destinationAddress, coverPort);
    strcpy(buffer, "devInfo " DeviceName);
    Serial.println(buffer);
    toCOVER.write((const uint8_t *)&buffer, strlen(buffer) + 1);
    toCOVER.endPacket();
}

struct messageBuffer
{
    float brake;
    float steeringAngle;
    float speed;
    unsigned long state;
    // state = 0: initial state
    // state = 1: button was clicked --> calcZeroOffset --> send device info
    // state = 2: butteon isPressed --> blink & toggleLED --> esp_restart()
    // state = 4: button was double clicked --> Configuration portal requested --> startConfigAP
};

struct messageBuffer mb;

void check_WiFi()
{
    if ((WiFi.status() != WL_CONNECTED) && !eth_connected)
    {
        Serial.println(F("\nWiFi and wired connection lost."));
        // waiting for a new connection
    }
}

void check_status()
{
    static ulong checkstatus_timeout = 0;
    static ulong checkwifi_timeout = 0;

    static ulong current_millis;

#define WIFICHECK_INTERVAL 1000L
#define HEARTBEAT_INTERVAL 10000L
#define LED_INTERVAL 2000L

    current_millis = millis();

    // Check WiFi every WIFICHECK_INTERVAL (1) seconds.
    if ((current_millis > checkwifi_timeout) || (checkwifi_timeout == 0))
    {
        check_WiFi();
        checkwifi_timeout = current_millis + WIFICHECK_INTERVAL;
    }

    // Print hearbeat every HEARTBEAT_INTERVAL (10) seconds.
    if ((current_millis > checkstatus_timeout) || (checkstatus_timeout == 0))
    {
        heartBeatPrint();
        checkstatus_timeout = current_millis + HEARTBEAT_INTERVAL;
    }
}

void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
        case SYSTEM_EVENT_ETH_START:
            Serial.println("ETH Started");
            // set eth hostname here
            ETH.setHostname("esp32-ethernet");
            break;
        case SYSTEM_EVENT_ETH_CONNECTED:
        case 20:
            Serial.println("ETH Connected");
            eth_link = true;
            connectionTimeout = millis();
            break;
        case SYSTEM_EVENT_AP_STACONNECTED:
            Serial.println("AP_STA Connected");
            break;
        case SYSTEM_EVENT_ETH_GOT_IP:
        case 22:
            Serial.print("ETH MAC: ");
            Serial.print(ETH.macAddress());
            Serial.print(", IPv4: ");
            Serial.print(ETH.localIP());
            if (ETH.fullDuplex())
            {
                Serial.print(", FULL_DUPLEX");
            }
            Serial.print(", ");
            Serial.print(ETH.linkSpeed());
            Serial.println("Mbps");

            sendDeviceInfo(ETH.broadcastIP());
            eth_connected = true;
            break;
        case SYSTEM_EVENT_ETH_DISCONNECTED:
        case 21:
            Serial.println("ETH Disconnected");
            eth_connected = false;
            eth_link = false;
            break;
        case SYSTEM_EVENT_ETH_STOP:
            Serial.println("ETH Stopped");
            eth_connected = false;
            eth_link = false;
            break;
        default:
            Serial.printf("ETH event: %d\n", event);
            break;
    }
}




void setup()
{
    // set led pin as output
    pinMode(LED_BUILTIN, OUTPUT);

    unsigned long startedAt = millis();

    digitalWrite(LED_BUILTIN, LED_ON);
    Serial.begin(115200);
    while (!Serial)
        ;

    // Format FileFS if not yet
    if (!FileFS.begin(true))
    {
        FileFS.format();

        Serial.println(F("SPIFFS/LittleFS failed! Already tried formatting."));

        if (!FileFS.begin())
        {
            // prevents debug info from the library to hide err message.
            delay(100);

#if USE_LITTLEFS
            Serial.println(F("LittleFS failed!. Please use SPIFFS or EEPROM. Stay forever"));
#else
            Serial.println(F("SPIFFS failed!. Please use LittleFS or EEPROM. Stay forever"));
#endif

            while (true)
            {
                delay(1);
                toggleLED();
            }
        }
    }

    initAPIPConfigStruct(WM_AP_IPconfig);
    initSTAIPConfigStruct(WM_STA_IPconfig);

#ifdef WIFIManager
    AsyncWebServer webServer(HTTP_PORT);

  DNSServer dnsServer;

  ESPAsync_WiFiManager ESPAsync_wifiManager(&webServer, &dnsServer, "AsyncConfigOnSwitch");

  ESPAsync_wifiManager.setDebugOutput(true);

#if USE_CUSTOM_AP_IP
  // set custom ip for portal
  //  New in v1.4.0
  ESPAsync_wifiManager.setAPStaticIPConfig(WM_AP_IPconfig);
  //////
#endif

  ESPAsync_wifiManager.setMinimumSignalQuality(-1);

  // From v1.0.10 only
  // Set config portal channel, default = 1. Use 0 => random channel from 1-13
  ESPAsync_wifiManager.setConfigPortalChannel(0);
  //////

#if !USE_DHCP_IP
  // Set (static IP, Gateway, Subnetmask, DNS1 and DNS2) or (IP, Gateway, Subnetmask). New in v1.0.5
  // New in v1.4.0
  ESPAsync_wifiManager.setSTAStaticIPConfig(WM_STA_IPconfig);
  //////
#endif

  // New from v1.1.1
#if USING_CORS_FEATURE
  ESPAsync_wifiManager.setCORSHeader("Your Access-Control-Allow-Origin");
#endif
  Router_SSID = ESPAsync_wifiManager.WiFi_SSID();
  Router_Pass = ESPAsync_wifiManager.WiFi_Pass();

#endif
    // SSID to uppercase
    ssid.toUpperCase();
    password = DeviceName;

    bool configDataLoaded = false;

#ifdef WIFIManager
    // From v1.1.0, Don't permit NULL password
  if ((Router_SSID != "") && (Router_Pass != ""))
  {
    LOGERROR3(F("* Add SSID = "), Router_SSID, F(", PW = "), Router_Pass);
    wifiMulti.addAP(Router_SSID.c_str(), Router_Pass.c_str());

    ESPAsync_wifiManager.setConfigPortalTimeout(120); // If no access point name has been previously entered disable timeout.
    Serial.println(F("Got ESP Self-Stored Credentials. Timeout 120s for Config Portal"));
  }
#endif

    digitalWrite(LED_BUILTIN, LED_ON);
    if (loadConfigData())
    {
        configDataLoaded = true;

#ifdef WIFWManager
        ESPAsync_wifiManager.setConfigPortalTimeout(120); // If no access point name has been previously entered disable timeout.
    Serial.println(F("Got stored Credentials. Timeout 120s for Config Portal"));
#endif

#if USE_ESP_WIFIMANAGER_NTP
        if (strlen(WM_config.TZ_Name) > 0)
        {
            LOGERROR3(F("Current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
            configTime(WM_config.TZ, "pool.ntp.org");
#else
            // configTzTime(WM_config.TZ, "pool.ntp.org" );
            configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
        }
        else
        {
            Serial.println(F("Current Timezone is not set. Enter Config Portal to set."));
        }
#endif
    }
    else
    {
        // Enter CP only if no stored SSID on flash and file
        Serial.println(F("Open Config Portal without Timeout: No stored Credentials."));
        initialConfig = true;
    }

#ifdef WIFWManager
    digitalWrite(LED_BUILTIN, LED_ON);
  if (initialConfig)
  {
    Serial.print(F("Starting configuration portal @ "));

#if USE_CUSTOM_AP_IP
    Serial.print(APStaticIP);
#else
    Serial.print(F("192.168.4.1"));
#endif

    Serial.print(F(", SSID = "));
    Serial.print(ssid);
    Serial.print(F(", PWD = "));
    Serial.println(password);

    digitalWrite(LED_BUILTIN, LED_ON); // Turn led on as we are in configuration mode.

    // sets timeout in seconds until configuration portal gets turned off.
    // If not specified device will remain in configuration mode until
    // switched off via webserver or device is restarted.
    // ESPAsync_wifiManager.setConfigPortalTimeout(600);
    // Starts an access point
    if (!ESPAsync_wifiManager.startConfigPortal((const char *)ssid.c_str(), password.c_str()))
      Serial.println(F("Not connected to WiFi but continuing anyway."));
    else
    {
      Serial.println(F("WiFi connected...yeey :)"));
    }

    // Stored  for later usage, from v1.1.0, but clear first
    memset(&WM_config, 0, sizeof(WM_config));

    for (uint8_t i = 0; i < NUM_WIFI_CREDENTIALS; i++)
    {
      String tempSSID = ESPAsync_wifiManager.getSSID(i);
      String tempPW = ESPAsync_wifiManager.getPW(i);

      if (strlen(tempSSID.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_ssid, tempSSID.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_ssid) - 1);

      if (strlen(tempPW.c_str()) < sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1)
        strcpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str());
      else
        strncpy(WM_config.WiFi_Creds[i].wifi_pw, tempPW.c_str(), sizeof(WM_config.WiFi_Creds[i].wifi_pw) - 1);

      // Don't permit NULL SSID and password len < MIN_AP_PASSWORD_SIZE (8)
      if ((String(WM_config.WiFi_Creds[i].wifi_ssid) != "") && (strlen(WM_config.WiFi_Creds[i].wifi_pw) >= MIN_AP_PASSWORD_SIZE))
      {

#ifdef WIFIManager
        LOGERROR3(F("* Add SSID = "), WM_config.WiFi_Creds[i].wifi_ssid, F(", PW = "), WM_config.WiFi_Creds[i].wifi_pw);
        wifiMulti.addAP(WM_config.WiFi_Creds[i].wifi_ssid, WM_config.WiFi_Creds[i].wifi_pw);
        #endif
      }
    }

#if USE_ESP_WIFIMANAGER_NTP
    String tempTZ = ESPAsync_wifiManager.getTimezoneName();

    if (strlen(tempTZ.c_str()) < sizeof(WM_config.TZ_Name) - 1)
      strcpy(WM_config.TZ_Name, tempTZ.c_str());
    else
      strncpy(WM_config.TZ_Name, tempTZ.c_str(), sizeof(WM_config.TZ_Name) - 1);

    const char *TZ_Result = ESPAsync_wifiManager.getTZ(WM_config.TZ_Name);

    if (strlen(TZ_Result) < sizeof(WM_config.TZ) - 1)
      strcpy(WM_config.TZ, TZ_Result);
    else
      strncpy(WM_config.TZ, TZ_Result, sizeof(WM_config.TZ_Name) - 1);

    if (strlen(WM_config.TZ_Name) > 0)
    {
      LOGERROR3(F("Saving current TZ_Name ="), WM_config.TZ_Name, F(", TZ = "), WM_config.TZ);

#if ESP8266
      configTime(WM_config.TZ, "pool.ntp.org");
#else
      // configTzTime(WM_config.TZ, "pool.ntp.org" );
      configTzTime(WM_config.TZ, "time.nist.gov", "0.pool.ntp.org", "1.pool.ntp.org");
#endif
    }
    else
    {
      LOGERROR(F("Current Timezone Name is not set. Enter Config Portal to set."));
    }
#endif

    // New in v1.4.0
    ESPAsync_wifiManager.getSTAStaticIPConfig(WM_STA_IPconfig);
    //////

    saveConfigData();
  }
#endif
    Serial.println(F("\nStarting " DeviceName));

    Serial.setDebugOutput(false);


    digitalWrite(LED_BUILTIN, LED_ON);
    digitalWrite(LED_BUILTIN, LED_ON); // turn the LED on by making the voltage LOW to tell us we are in configuration mode.

    WiFi.onEvent(WiFiEvent);
    ETH.begin();

    startedAt = millis();

    while (WiFi.status() != WL_CONNECTED && eth_connected == false)
    {
        if(millis()%1000 == 0)
        {
            Serial.println(F("not connected, waiting..."));
            Serial.printf("wifi Status %d\n", WiFi.status());
        }

        if (!eth_connected && eth_link)
        {
            if ((millis() - connectionTimeout) > 5000)
            {
                // have link but no DHCP IP, thus set to default IP
                if (configDataLoaded)
                {

                    ETH.config(WM_STA_IPconfig._sta_static_ip, WM_STA_IPconfig._sta_static_gw, WM_STA_IPconfig._sta_static_sn, WM_STA_IPconfig._sta_static_dns1);

                    Serial.print(F("ETH using IP"));

                    displayIPConfigStruct(WM_STA_IPconfig);
                    eth_connected=true;
                }
                else
                {

                    // ETH.config(IPAddress(192, 168, 0, 45), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0), IPAddress(8, 8, 8, 8));

#ifdef WIFWManager
                    startConfigAP();
          Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
#endif
                    connectionTimeout = millis();
                }
            }
        }
        else if (!eth_link && ((millis() - connectionTimeout) > 5000))
        {
#ifdef WIFWManager
            startConfigAP();
      Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));
#endif
        }
        else
        {
            delay(1000);
        }
    }
    Serial.print(F("After waiting "));
    Serial.print((float)(millis() - startedAt) / 1000);
    Serial.print(F("connected. Local IP: "));
    if (eth_connected)
    {
        Serial.print(ETH.localIP());
    }
    else
    {
        Serial.println(WiFi.localIP());
    }
    ArduinoOTA.setHostname(DeviceName);

    // No authentication by default
    // ArduinoOTA.setPassword((const char *)"123");

    ArduinoOTA.onStart([]()
                       { Serial.println("Start"); });
    ArduinoOTA.onEnd([]()
                     {
                         Serial.println("\nEnd");
                         digitalWrite(LED_BUILTIN,LED_OFF); });
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          {
                              static int oldp = 0;
                              float per = progress / (total / 100.0);
                              if(oldp!=(int)per)
                              {
                                  oldp = (int)per;
                                  toggleLED();
                              }
                              esp_task_wdt_reset(); // reset the watchdog
                              Serial.printf("Progress: %u%%\r", (unsigned int) per); });
    ArduinoOTA.onError([](ota_error_t error)
                       {
                           Serial.printf("Error[%u]: ", error);
                           if (error == OTA_AUTH_ERROR)
                               Serial.println("Auth Failed");
                           else if (error == OTA_BEGIN_ERROR)
                               Serial.println("Begin Failed");
                           else if (error == OTA_CONNECT_ERROR)
                               Serial.println("Connect Failed");
                           else if (error == OTA_RECEIVE_ERROR)
                               Serial.println("Receive Failed");
                           else if (error == OTA_END_ERROR)
                               Serial.println("End Failed"); });
    ArduinoOTA.begin();
    Serial.println("OTAServer verbunden");
    toCOVER.begin(coverPort);
    zeroButton.init(true);
    Serial.println("First Send DevInfo");

    headwind1 = new HeadwindController(&headwind1Address);
    headwind2 = new HeadwindController(&headwind2Address);

    Serial.println((long long)headwind1);
    Serial.println((long long)headwind2);
    unsigned long frameTime = 0;
    unsigned long pressedTime = 0;
    // Bluetooth to connect with Tacx2
    BLEDevice::init("ESP32-BLE-Client");

    /* Retrieve a Scanner and set the callback we want to use to be informed when we
       have detected a new device.  Specify that we want active scanning and start the
       scan to run for 5 seconds. */
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    //Serial.println("Test irgendwo");
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    startScan();

    pinMode(A_PIN, INPUT_PULLUP);
    pinMode(B_PIN, INPUT_PULLUP);
    pinMode(I_PIN, INPUT_PULLUP);
    encoderSetup();
    digitalWrite(LED_BUILTIN, LED_ON);

    brakeSensor = new loadCell(BRAKE_DOUT_PIN, BRAKE_SCK_PIN);
    xTaskCreatePinnedToCore(    BluetoothLoop,    "Bluetooth_send_forces",    5000,      NULL,    2,    &Task0,    0);

    Serial.println("Finished Setup");
}

#pragma pack(push, 1)
#pragma pack(1)
// powerMessage has operational code and resistance that needs to be passed to Flux2
struct powerMessage
{
    uint8_t opCode;
    uint16_t resistance;
};
#pragma pack(pop)
enum opCodes
{
    ocRequesControl = 0x00,
    ocReset = 0x01,
    ocSetTargetSpeed = 0x02,
    ocSetTargetInclination = 0x03,
    ocSetTargetResistaneLevel = 0x04,
    ocSetTargetPower = 0x05,
    ocStart = 0x07,
    ocStop = 0x08,
    ocResponseCode = 0x80
};

bool firstTime = true;
unsigned long pressedTime = 0;
unsigned long printTime = 0;
float resistance;
void loop()
{

    unsigned long frameTime = millis();
    /*static int p=80;
    static unsigned long lastTime = 0;
    if(frameTime - lastTime > 200) {
        headwind1->setPower(abs(p));
        headwind2->setPower(abs(p));
        p+=4;
        lastTime = frameTime;
    }
    if(p>100)
        p=-100; */
    if (!eth_connected && eth_link)
    {
        if ((frameTime - connectionTimeout) > 5000)
        {
            // have link but no DHCP IP, thus set to default IP
            //ETH.config(IPAddress(192, 168, 0, 45), IPAddress(192, 168, 0, 1), IPAddress(255, 255, 255, 0), IPAddress(8, 8, 8, 8));


            ETH.config(WM_STA_IPconfig._sta_static_ip, WM_STA_IPconfig._sta_static_gw, WM_STA_IPconfig._sta_static_sn, WM_STA_IPconfig._sta_static_dns1);

            Serial.print(F("ETH using IP"));

            displayIPConfigStruct(WM_STA_IPconfig);
        }
    }
    // reset angle to 0 when index pin is high
    if (digitalRead(I_PIN))
    {
        encoder.setCount(ENCODER_OFFSET);
        if (frameTime - printTime > 1000)
        {
            printTime = frameTime;
            Serial.println("Re-calibrate steering angle");
        }
    }
    encoderCount = encoder.getCount();
    if (encoderCount != oldEncoderCount)
    {
        Serial.print("Encoder count: ");
        Serial.println(encoderCount);
        oldEncoderCount = encoderCount;
    }
    mb.steeringAngle = encoderCount * angleFactor;
    mb.speed = speed;

    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTimeheadwind >= 500) {
        speed = constrain(speed, 0, 3000);

        Serial.print("Constrained speed: ");
        Serial.println(speed);

        int headwind = map(speed, 0, 3000, 0, 100);

        Serial.print("Mapped headwind value: ");
        Serial.println(headwind);

        bool result1 = headwind1->setPower(headwind);
        bool result2 = headwind2->setPower(headwind);

        Serial.print("Set power result for headwind1: ");
        Serial.println(result1 ? "Success" : "Failure");
        Serial.print("Set power result for headwind2: ");
        Serial.println(result2 ? "Success" : "Failure");

        lastUpdateTimeheadwind = currentTime;
    }
    //Serial.printf("test1 %d\n",(millis() - frameTime));
    frameTime = millis();
    if (doConnect)
    {
        connectToServer();
        doConnect = false;
    }

    headwind1->update();
    headwind2->update();



    //Serial.printf("test11 %d\n",(millis() - frameTime));
    frameTime = millis();
    mb.state = 0;
    if (zeroButton.wasKlicked())
    {
        Serial.printf("button Klicked\n");
        brakeSensor->calcZeroOffset();
        sendDeviceInfo(ETH.broadcastIP()); // re announce ourselves
        lastWorkingTime = frameTime;
        mb.state = 1;
    }

    static int nblink = 0;
    if (zeroButton.isPressed())
    {
        lastWorkingTime = frameTime;
        Serial.printf("button isPressed\n");
        mb.state = 2;
        if ((frameTime - (pressedTime + (nblink * 200))) > 200)
        {
            nblink++;
            toggleLED();
        }
        if (frameTime - pressedTime > 5000)
        {
            esp_restart();
        }
    }
    else
    {
        pressedTime = frameTime;
        nblink = 0;
    }
    if (zeroButton.wasDoubleKlicked())
    {
        lastWorkingTime = frameTime;

        Serial.printf("button Double Klicked\n");

        startConfigAP();
        //Serial.println(ESPAsync_wifiManager.getStatus(WiFi.status()));

        Serial.print(F("ETH using IP"));

        displayIPConfigStruct(WM_STA_IPconfig);
        mb.state = 4;
    }
    ArduinoOTA.handle();
    debounceButton::update();
    int received = toCOVER.parsePacket();
    if (received > 0)
    {
        char buffer[500];
        int numRead = toCOVER.read(buffer, 500);
        if (numRead > 0)
        {
            //Serial.print(buffer);
            if (numRead >= 5)
            {
                if (strcmp(buffer, "enum") == 0)
                {
                    Serial.printf("enum\n");
                    sendDeviceInfo(toCOVER.remoteIP());
                }
                else if (strcmp(buffer, "start") == 0)
                {
                    Serial.printf("remoteStart\n");
                    coverIP = toCOVER.remoteIP();
                }
                else if (strcmp(buffer, "stop") == 0)
                {
                    Serial.printf("stop\n");
                    coverIP = (uint32_t)0;
                }
                else
                {
                    Serial.printf("Read %d bytes\n", numRead);
                }
            }
            else if (numRead == 4)
            {
                resistance = *((float *)buffer);
                // we received
                if (frameTime - lastUpdateTime >= 32)
                {
                    // if our last message was sent more than 1/30th of a second ago we reply to this message from OpenCOVER
                    lastUpdateTime = 0;

                    //Serial.printf("resistance: %f\n",resistance);
                }
                else if ((frameTime - lastUpdateTime) >= updatePeriod - 32)// make sure we send a message back not later than 32ms
                {
                    lastUpdateTime = (frameTime - updatePeriod)+32;
                }
                //Serial.print("Resistance: ");
                //Serial.println(resistance);
            }
            else
            {
                Serial.printf("Read %d bytes\n", numRead);
            }
        }
    }
    if (coverIP != 0)
    {
        if (frameTime - lastUpdateTime >= updatePeriod)
        {
            //Serial.printf("mb1:\n");
            lastUpdateTime = frameTime;
            mb.brake = brakeSensor->getForce();
            mb.steeringAngle = encoderCount * angleFactor;
            toCOVER.beginPacket(coverIP, pluginPort);
            toCOVER.write((const uint8_t *)&mb, sizeof(mb));
            toCOVER.endPacket();

            //Serial.printf("mb: %f, %f, %d\n",mb.speed,mb.steeringAngle,(millis() - lastUpdateTime));
            lastWorkingTime = frameTime;
        }
    }
    else // we are not doing anything
    {
        digitalWrite(LED_BUILTIN, LED_OFF);
    }

    //Serial.printf("test2 %d\n",(millis() - frameTime));
    frameTime = millis();
    //check_status();

    //Serial.printf("test3 %d\n",(millis() - frameTime));
}


void sendForce()
{
    if (connected)
    {
        if (pResistanceCharacteristic)
        {
            uint8_t *result;
            if (firstTime)
            {
                Serial.println("do Init    dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
                powerMessage msg;
                msg.opCode = ocReset;
                msg.resistance = 100;
                pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1, true);
                sleep(1);
                msg.opCode = ocRequesControl;
                msg.resistance = 100;
                pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1, true);
                sleep(1);
                msg.opCode = ocStart;
                msg.resistance = 100;
                pResistanceCharacteristic->writeValue((uint8_t *)&msg, 1, true);
                sleep(1);
                firstTime = false;
            }
            // setting target resistance level, this should depend on the brake (load cell)
            powerMessage msg;
            msg.opCode = ocSetTargetResistaneLevel;
            // resistance depends on the value sent from COVISE through UDP
            msg.resistance = resistance;
            pResistanceCharacteristic->writeValue((uint8_t *)&msg, sizeof(msg), true);
        }
        else{ sleep(1);}
    }
    else{ sleep(1);}
}

void BluetoothLoop( void * parameter )
{
    for (;;) {
        sendForce();
    }
}