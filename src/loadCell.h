#ifndef LOAD_CELL_H
#define LOAD_CELL_H
#pragma once
#include <HX711.h>

class loadCell
{
    public:
        loadCell(int dout_pin, int sck_pin);
        float update(); // return the float of the magnitude of force applied on the load cell
        void calcZeroOffset();

    private:
        HX711 brakeSensor;
        static unsigned long previousTime;
        static unsigned long currentTime;
        
        float brakeZeroOffset;
        float brakeReading;
        static const int brakeScaleFactor = -5000;
        static const int period = 300;
};

#endif