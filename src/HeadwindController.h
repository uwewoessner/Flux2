#include <arduino.h>
#include <BLEAddress.h>
#include <BLEAdvertisedDevice.h>
#include <BLEClient.h>
#include <BLERemoteCharacteristic.h>
#include <BLEDevice.h>

class HeadwindController: public BLEClientCallbacks
{
private: 
    BLEAddress *address=nullptr;
    BLEAdvertisedDevice *device=nullptr; 
    BLEClient *pClientHeadwind=nullptr;
    BLERemoteCharacteristic *powerCharacteristics=nullptr;
    bool doConnect=false;
    bool connectedToHeadwind=false;
    
    static void notifyPowerCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                             uint8_t *pData, size_t length, bool isNotify);

public:
    HeadwindController(BLEAddress *macAddress);
    ~HeadwindController();
    
   bool connectToHeadwind();
   bool setPower(int power);
   bool isConnected();
   bool update();
   bool init(BLEAdvertisedDevice &advertisedDevice);
   
   virtual void onDisconnect(BLEClient *pclient);
   virtual void onConnect(BLEClient *pclient);
};

