#include <stdio.h>
#include <pthread.h>
#include <signal.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <syslog.h>

#include "util.h"
#include "car_lib.h"

typedef enum { left = -1, none = 0, right = 1}steerDirection;

// Global variables
pthread_t threads[3];

// Position Speed Control global variables
int speed, tol, posInit, posDes;
unsigned char gain;
bool enablePositionSpeed;

// PID Speed Control global variables
int pidSpeed, propGain, integralGain, differentialGain;

// Smooth Steering global variables
signed short endAngle, angleTick;
int usleepTick;


// Distance IR Sensor Control global variables
int channel[6] = { 1, 2, 3, 4, 5, 6 }, distance[6];

void* positionSpeedControl(void *arg)
{
    int position, posRead;

    position = 0;
    posRead = 0;

    while (1)
    {
        if (enablePositionSpeed)
        {
            //printf("Start run!\n");

            //jobs to be done beforehand;
               // speed controller must be also ON !!!
            SpeedControlOnOff_Write(CONTROL);
            
            DesireSpeed_Write(speed);

            //control on/off          
            PositionControlOnOff_Write(CONTROL);

            //position controller gain set
            //default value = 10, range : 1~50
            PositionProportionPoint_Write(gain);

            //position write
            EncoderCounter_Write(posInit);
            
            //position set
            position = posInit + posDes;
            DesireEncoderCount_Write(position);

            //position = DesireEncoderCount_Read();

            //printf("abs(posRead-position) = %d\n", abs(posRead-position));

            /* while(abs(posRead-position)>tol)
            {
                printf("check distance\n");
                usleep(100000);
                posRead = EncoderCounter_Read();
                if(posRead != CHECKSUMERROR)
                {
                    printf("EncoderCounter_Read() = %d %d\n", posRead, abs(posRead-position));
                }
                else
                {
                    printf("CHECKSUMERROR!, stop reading Encodercount! \n");
                    break;
                }
            }*/
        }
        
        usleep(100000);
    }
}

void speedPIDControl(int speed)
{
    //jobs to be done beforehand;
    PositionControlOnOff_Write(UNCONTROL); // position controller must be OFF !!!

    //control on/off
    SpeedControlOnOff_Write(CONTROL);

    //speed controller gain set
    //P-gain
    // default value = 10, range : 1~50
    SpeedPIDProportional_Write(propGain);

    //I-gain
    // default value = 10, range : 1~50
    SpeedPIDIntegral_Write(integralGain);
    
    //D-gain
    // default value = 10, range : 1~50
    SpeedPIDDifferential_Write(differentialGain);

    //speed set    
    DesireSpeed_Write(speed);
}

void* checkDistance(void *arg)
{
    int i;

    while (1)
    {
        for (i = 0; i < 6; ++i)
            distance[i] = DistanceSensor(channel[i]);

        printf("%d %d %d %d %d %d\n", distance[0], distance[1], distance[2], distance[3], distance[4], distance[5]);
        usleep(100000);
    }
}

void smoothSteeringControl(signed short endAngle, signed short tick, int usleepTick)
{
    signed short nowAngle;
    steerDirection direction;

    while (true)
    {
        nowAngle = SteeringServoControl_Read();

        if (nowAngle == endAngle)
            continue;

        direction = (nowAngle - endAngle) > 0 ? right : left;

        while (nowAngle != endAngle)
        {
            if (abs(nowAngle - endAngle) < tick)
            {
                SteeringServoControl_Write(endAngle);
                break;
            }
            else
            {
                if (direction == left) 
                    nowAngle -= tick;
                else if (direction == right)
                    nowAngle += tick;
                
                SteeringServoControl_Write(nowAngle);
            }

            printf("Steering : now = %d   end = %d\n", nowAngle, endAngle);
            
            usleep(usleepTick);
        }
    }
}

void loopCheckDistance(int sensorIndex, int wantDistance, bool isUp)
{
    if (isUp)
    {
        while (true)
        {       
            if (distance[sensorIndex] > wantDistance)
                break;

            usleep(10000);
        }
    }
    else
    {
        while (true)
        {
            if (distance[sensorIndex] < wantDistance)
                break;

            usleep(10000);
        }
    }
    
}

void main(void)
{
    int ret;

    CarControlInit();

    endAngle = 1550;
    SteeringServoControl_Write(1530);

    tol = 100;
    posInit = 0;
    posDes = 150;
    gain = 10;
    speed = 100;
    enablePositionSpeed = true;

    ret = pthread_create(&threads[0], NULL, positionSpeedControl, NULL);

    if (ret)
        MSG("Failed creating capture thread");
    
    pthread_detach(threads[0]);

    ret = pthread_create(&threads[1], NULL, checkDistance, NULL);

    if (ret)
        MSG("Failed creating capture thread");
    
    pthread_detach(threads[1]);

    //ret = pthread_create(&threads[2], NULL, smoothSteeringControl, NULL);

    if (ret)
        MSG("Failed creating capture thread");
    
    //pthread_detach(threads[2]);



    printf("Check 1st distance...\n");

    loopCheckDistance(1, 500, true);

    /* while (true)
    {       
        if (distance[1] > 500)
            break;

        usleep(10000);
    }*/

    printf("Check 2nd distance...\n");

    loopCheckDistance(2, 500, true);

    /*/while (true)
    {      
        if (distance[2] > 500)
            break;

        usleep(10000);
    }*/

    printf("Check 2nd distance...\n");

    loopCheckDistance(1, 500, false);

    /*while (true)
    {      
        if (distance[1] < 500)
            break;

        usleep(10000);
    }*/

    printf("Check 2nd distance...\n");

    loopCheckDistance(1, 500, true);

    /*/while (true)
    {      
        if (distance[1] > 500)
            break;

        usleep(10000);
    }*/

    printf("Check 3rd distance...\n");

    loopCheckDistance(2, 500, true);

    /*while (true)
    {
        if (distance[2] > 500)
            break;
        
        usleep(10000);
    }*/

    SteeringServoControl_Write(1900);

    printf("Check 4rd distance...\n");

    loopCheckDistance(1, 300, true);

    /*while (true)
    {
        if (distance[1] < 300)
            break;
        
        usleep(10000);
    }*/

    usleep(200000);
    enablePositionSpeed = false;
    printf("Stop Vehicle!\n");

    SteeringServoControl_Write(1050);

    propGain = 20;
    integralGain = 20;
    differentialGain = 20;

    speedPIDControl(-30);

    printf("Check 1st back distance...\n");

    loopCheckDistance(3, 750, true);

    /*while (true)
    {
        if (distance[3] > 750)
            break;
        
        usleep(50000);
    }*/

    
    SteeringServoControl_Write(1250);

    printf("Check 1st-2 back distance...\n");

    loopCheckDistance(3, 500, false);

    /*while (true)
    {
        if (distance[3] < 500)
            break;
        
        usleep(50000);
    }*/

    loopCheckDistance(3, 600, true);

    /*while (true)
    {
        if (distance[3] > 600)
            break;
        
        usleep(50000);
    }*/

    SteeringServoControl_Write(1950);

    printf("Check 2nd back distance...\n");

    loopCheckDistance(3, 1800, true);

    /*while (true)
    {
        if (distance[3] > 1800)
            break;
        
        usleep(50000);
    }*/

    printf("Check 3rd back distance...\n");

    SteeringServoControl_Write(1530);

    loopCheckDistance(3, 2000, true);

    /*while (true)
    {
        if (distance[3] > 2100)
            break;
        
        usleep(50000);
    }*/

    printf("Stop Vehicle!\n");

    speedPIDControl(0);

    SteeringServoControl_Write(1000);

    enablePositionSpeed = true;

    loopCheckDistance(0, 2000, true);

    enablePositionSpeed = false;

    SteeringServoControl_Write(1530);

    speedPIDControl(-10);

    loopCheckDistance(3, 2500, true);

    printf("Stop Vehicle!\n");

    speedPIDControl(0);

    sleep(2);

    SteeringServoControl_Write(2000);

    enablePositionSpeed = true;

    loopCheckDistance(3, 3500, true);
}



