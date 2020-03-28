//#define randomData
#define virtualMotor

#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include "data_structures.h"
#include "VESC_interface.h"

#ifdef randomData
    #include "random_data.h"
#else
    #ifdef virtualMotor
        #include "sim_motor.h"
        #define VESC_Controller 0
        struct simParameters param;
    #endif

#endif

volatile char paramChoice = 'd';
volatile float alphaV = 0.0, betaV = 0.0, load = 0.0;

int returnstatus;
int pid;
volatile int count = 1, i = 0;

void childTask() 
{
    printf("Motor Simulator (Child process) started with)=> PPID: %d PID: %d\n", getppid(), getpid());
    
#ifdef randomData
    dataRandomizer();
#else
#ifdef virtualMotor
    printf("\nDefault electrical properties of the motor are as follows: \r\n");
    printf("Stator Resistance Phase to Neutral in Ohms: 0.020\n");
    printf("Inductance in d-Direction in H: 20*10^-6 \n");
    printf("Inductance in q-Direction in H: 20*10^-6 \n");
    printf("Flux linkage of the in mWb: 0.020\n");
    printf("Rotor+Load Inertia in Nm*s^2: 0.001\n");
    printf("Input voltage in V: 48\n");
    printf("Pole pairs: 8\n");
    printf("Control frequecy in kHz: 20.00\r\n");

    printf("\nEnter 'd' to use default electrical properties or 'u' to manually enter all electrical properties of the motor: ");
    scanf("%c", &paramChoice);
    initSim(paramChoice);
    /*
    if(paramChoice == 'd')
    {
        printf("Running the simulator with default values.\n");
    }
    else
    {
        printf("Running the simulator with user-defined motor properties.\n");
    }
    */
#ifdef DEBUG
    printf("Enter load torque in Nm [0.5 ~ 5]: ");
    scanf("%f", &load);
    alphaV = 24.01;
    betaV = 24.01;
#else
    printf("Enter voltage on alpha axis: ");
    scanf("%f", &alphaV);
    printf("Enter voltage on beta axis: ");
    scanf("%f", &betaV);
    printf("Enter load torque in Nm: ");
    scanf("%f", &load);
    // printf("Enter enter the number of times you want to run the simulation: ");
    // scanf("%d", &count);
#endif
    if (runSim(alphaV, betaV, load))
    {
        printf("\nSimulation finished. Shutting down simulator!\r\n");   
    }
    else
    {
        printf("Something went wrong. Simulation failed!\r\n");
    }
}

void parentTask() 
{
    float readData[20];
    printf("\nParent task with => PID: %d started!\n", getpid());
    printf("Reading from pipe...\n");
   //close(0);
   close(pipefds[1]);
   //dup(pipefds[0]);
   read(pipefds[0], readData, sizeof(readData));
   printf("\nInput Voltage is: %.2f V\r\n", readData);
#ifdef DEBUG
   if (readData != 48)
   {
       printf("\nPipe implementation seems to have failed!\n");
       printf("\nExpected value was: 48 but got %.2f\n", readData);
   }
   else
   {
       printf("\nPipe implementation passed!\n");
   }
#endif
}

int main()
{
    init_VESC();
    if (VESC_Controller)
    {
        //handle controller comms.... probably in Report 3 or 4
    }
    else
    {
        printf("VESC Controller not found... Running virtual motor model instead... \r\n");
    }
    returnstatus = pipe(pipefds);
    if (returnstatus == -1) {
        printf("Unable to create pipe\n");
        return 1;
    }
    pid_t pid = fork();

    if (pid == 0) 
    {
        childTask();
        exit(0);
    }
    else if (pid > 0) 
    {
        wait(0);
        parentTask();
    }
    else
    {
        printf("Unable to create child process.");
    }

        /*
        printf("Input Voltage is: %.2f V\r\n", VESC.inputVoltage);
        printf("Input current: %.2f mA\r\n", VESC.inputCurrent);
        printf("Motor current: %.2f mA\r\n", VESC.motorCurrent);
        printf("Brake current: %.2f mA\r\n", VESC.brakeCurrent);
        printf("Duty Cycle: %.2f %\r\n", (VESC.duty * 100));
        printf("Speed: %d RPM\r\n", VESC.rpm);
        */
        /*
        if (virtual_motor_is_connected())
        {
            printf("Input Voltage is: %.2f V\r\n", VESC.inputVoltage);
            printf("Input current: %.2f mA\r\n", VESC.inputCurrent);
            printf("Motor current: %.2f mA\r\n", VESC.motorCurrent);
            printf("Brake current: %.2f mA\r\n", VESC.brakeCurrent);
            printf("Duty Cycle: %.2f %\r\n", (VESC.duty * 100));
            printf("Speed: %d RPM\r\n", VESC.rpm);
            disconnect_virtual_motor();
        }
        else
        {
            printf("Virtual motor not found!\n");
        }
        */
    #endif
#endif
    return 0;
}
