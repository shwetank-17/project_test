#include <stdio.h> 
#include <stdlib.h> 
#include <time.h>

#include "random_data.h"
#include "data_structures.h"
#include "VESC_interface.h"


int randInt(int lowLim, int upLim, int value)
{
    int i;
    for (i = 0; i < value; i++) 
    {
        int randNum = (rand()%(upLim - lowLim + 1)) + lowLim;
        return randNum;
    }
}

float randfloat(float lowLim, float upLim)
{
    float range = (float)rand() / (float)RAND_MAX;
    return (lowLim + range * (lowLim - upLim));
}

void dataRandomizer(void)
{
    srand(time(0));
    VESC.inputVoltage = randfloat(12.0, 12.5);
    VESC.inputCurrent = randfloat(4.0, 4.5) * 1000;
    VESC.motorCurrent = randfloat(3.8, 4.0) * 1000;
    VESC.brakeCurrent = randfloat(1.8, 2.0) * 1000;
    VESC.rpm = randfloat(6.00, 6.01) * 1000;
    VESC.duty = randfloat(0.98, 0.995);
    VESC.amp_hours = 0.0;
    VESC.watt_hours = 0.0;
    VESC.tachometer_abs = 0;
}