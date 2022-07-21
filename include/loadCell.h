#ifndef LOAD_CELL_H
#define LOAD_CELL_H
#pragma once
#include <HX711.h>

class loadCell
{
    public:
        loadCell(int dout_pin, int sck_pin);
        void update(long* brakeReading); // return the float of the magnitude of force applied on the load cell
        void calcZeroOffset();
        int doutPin;

    private:
        HX711 brakeSensor;
        static unsigned long previousTime;
        static unsigned long currentTime;
        
        float brakeZeroOffset;
        float brakeReading;
        static const int brakeScaleFactor = -5000; // modify this to change the scale factor to adjust the sensitivity of the sensor
        static const int period = 300; // modify this to change how often it checks the reading
};

#endif