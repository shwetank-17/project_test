#include "VESC_interface.h"
#include "data_structures.h"
#include <stdbool.h>

struct mc_values VESC;

void init_VESC(void)
{
    //printf("init_VESC - DEBUG check!\n");
    VESC.inputVoltage = 0.0;
    VESC.inputCurrent = 0.0;
    VESC.motorCurrent = 0.0;
    VESC.brakeCurrent = 0.0;
    VESC.rpm = 0.0;
    VESC.duty = 0.0;
    VESC.amp_hours = 0.0;
    VESC.watt_hours = 0.0;
    VESC.tachometer_abs = 0;
}


boolean VESC_readData(struct mc_values *val)
{
    return True;
}

/*
int send_payload(unsigned int* payload, unsigned int lenPay)
{
    return 0;
}
void VESC_setValue(struct settings setData, float *setValue)
{
    //do stuff
}
*/
