#include "loadCell.h"

unsigned long loadCell::currentTime = 0;
unsigned long loadCell::previousTime = 0;

loadCell::loadCell(int doutPin, int sckPin)
{
    brakeSensor.begin(doutPin, sckPin);
    doutPin = doutPin;
    calcZeroOffset();
}

/**
 * Calculate and update the zero offset of the load cell when there is no force applied to it
 * 
 */
void loadCell::calcZeroOffset()
{
    //Calculate zero offset for the brake to ensure that when no force is applied the reading is at 0
    Serial.println("Calibrating Zero");
    for (int i = 0; i < 10; i++)
    {
        if (brakeSensor.wait_ready_timeout(1000))
        {
            long breakReading = brakeSensor.read();
            if (i == 0)
            {
                brakeZeroOffset = breakReading;
            }
            else
            {
                // brakeZeroOffset is the average of the other trials
                brakeZeroOffset = ((float)(i-1)/float(i))*brakeZeroOffset + (1.0/float(i)) * breakReading;
            }
        }
    }
}

/**
 * Update the value of the load cell
 * @return float of the reading of the load cell after the most recent update
 */
void loadCell::update(long* brakeReading)
{
    currentTime = millis();
    // Update and return the reading for the force value
    if (brakeSensor.is_ready())
    {
        brakeSensor.set_scale();
        *brakeReading = (brakeSensor.read() - brakeZeroOffset)/brakeScaleFactor;
        previousTime = currentTime;
    }
    else
    {
        Serial.println("HX711 not found.");
    }
}