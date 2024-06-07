//
// Created by mathias on 5/24/24.
//

#include "HeadwindController.h"



bool HeadwindController::setPower(int power /* power is between 0 and 100 */)
{
    if(isConnected()) {
        uint8_t buffer[2];
        buffer[0] = 2;
        buffer[1] = power;
        Serial.print("settingPower ");
        powerCharacteristics->writeValue(buffer, 2);
        //powerCharacteristics->readRawData();
        Serial.println(power);
        return true;
    }

    return false;
}

bool HeadwindController::connectToHeadwind() {

    BLEUUID powerServiceUUID("a026ee0c-0a7d-4ab3-97fa-f1500f9feb8b");
    BLEUUID powerCharacteristicUUID("a026e038-0a7d-4ab3-97fa-f1500f9feb8b");
    BLEUUID initCharacteristicUUID("a026e038-0a7d-4ab3-97fa-f1500f9feb8b");
    Serial.print("Connecting to: ");
    Serial.println(device->toString().c_str());

    pClientHeadwind = BLEDevice::createClient();
    pClientHeadwind->setClientCallbacks(this);

    if (pClientHeadwind->connect(device)) {
        Serial.println("Connected to headwind!");

        // Services durchsuchen und ausgeben
        /*std::map<std::string, BLERemoteService*>* services = pClientHeadwind->getServices();
        for (auto const& pair : *services) {
            Serial.printf("Service UUID ist folgende: %s\n", pair.second->getUUID().toString().c_str());

            // Characteristics durchsuchen und ausgeben
            std::map<std::string, BLERemoteCharacteristic*>* characteristics = pair.second->getCharacteristics();
            for (auto const& charPair : *characteristics) {
                Serial.printf("  Characteristic UUID ist folgende: %s\n", charPair.second->getUUID().toString().c_str());
            }
        }*/
        BLERemoteService* pRemoteService = pClientHeadwind->getService(powerServiceUUID);

        if (pRemoteService != nullptr) {

            powerCharacteristics = pRemoteService->getCharacteristic(powerCharacteristicUUID);
            if (powerCharacteristics)
            {
                if (powerCharacteristics->canNotify())
                {
                    powerCharacteristics->registerForNotify(notifyPowerCallback);
                }
                connectedToHeadwind = true;
            }

        } else {
            Serial.println("Service not found.");
            return false;
        }
        return true;
    } else {
        Serial.println("Failed to connect to headwind.");
        return false;
    }
};


bool HeadwindController::update()
{
    if(!isConnected())
    {
        doConnect = true;
        delete pClientHeadwind;
        pClientHeadwind = nullptr;
        delete powerCharacteristics;
        powerCharacteristics = nullptr;
    }
    if (doConnect) {
        if (connectToHeadwind()) {
            Serial.println("Successfully connected to Headwind.");
        } else {
            Serial.println("Failed to connect to Headwind.");
            return false;
        }
        doConnect= false;
        return true;
    }
    return false;
}

bool HeadwindController::init(BLEAdvertisedDevice &advertisedDevice) {
    //Serial.println("init");
    //Serial.println(advertisedDevice.getAddress().toString().c_str());
    //Serial.println(advertisedDevice.toString().c_str());
    if (device == nullptr && advertisedDevice.getAddress()==*address) {

        Serial.println("found ");
        device = new BLEAdvertisedDevice(advertisedDevice);
        Serial.println(device->toString().c_str());
        doConnect = true;
        pClientHeadwind = nullptr;
        powerCharacteristics = nullptr;
        return true;
    }
    return false;
}

HeadwindController::HeadwindController(BLEAddress *macAddress) {
    address = macAddress;
};

HeadwindController::~HeadwindController()
{
}

void HeadwindController::onDisconnect(BLEClient *pclient)
{
    Serial.println("onDisconnect");
    /**/
}

void HeadwindController::onConnect(BLEClient *pclient)
{
    Serial.println("Headwind Client 1 and 2 Connected");
}

bool HeadwindController::isConnected()
{
    if(pClientHeadwind==nullptr)
        return false;
    return pClientHeadwind->isConnected();
}

void HeadwindController::notifyPowerCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                             uint8_t *pData, size_t length, bool isNotify)
{
    //Serial.print("Notify callback for power characteristic ");
}