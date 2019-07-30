#pragma once

#if 0
/*
*	Embeded Software Contest 2017
*	Davinci Code : Vehicle Control part
*	Copyright (c) 2017, Heremes Team. All rights reserved.
*/
#endif	/* Copyright */

#if 1
/* base library */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cv.h>
#endif	/* Library import */

#if 1

#define ACC_ENABLED 1

/* Image Feature Value */
#define MID_X	160
#define MID_Y	120
#define HALF_LENGTH 520//480	//160  //520 -> 18:21

/* Vehicle Desired Speed */
#define START_SPEED	250
#define NORMAL_SPEED	100
#define SAFE_SPEED	100
#define STOP_SPEED	0
#define SPEED_HILL_ASC	300
#define SPEED_HILL_DESC	100
#define SPEED_UP	120
#define SPEED_DOWN	50

/* Vehicle PID Gain */
#define	GAIN_POS_P 10
#define	GAIN_P 20
#define	GAIN_I 20
#define	GAIN_D 20

/* Vehicle DirServo Angle */
#define	ANGLE_MID 1500
#define R_CENTRAL_MAX 70
#define	ANGLE_STRAIGHT_MIN (ANGLE_MID - R_CENTRAL_MAX)
#define	ANGLE_STRAIGHT_MAX (ANGLE_MID + R_CENTRAL_MAX)

#define	ANGLE_LEFT_SOFT 1310
#define	ANGLE_RIGHT_SOFT 1710

#define	ANGLE_LEFT_SUBMAX 1310
#define	ANGLE_RIGHT_SUBMAX 1710

#define	ANGLE_LEFT_MAX 1000
#define	ANGLE_RIGHT_MAX 2000

/* Vehicle Camera Servo Angle */
#define	CAMERA_X_ANGLE_NORMAL 1500
#define	CAMERA_X_ANGLE_RIGHT 1200
#define	CAMERA_X_ANGLE_LEFT 1800

#define	CAMERA_Y_ANGLE_NORMAL 1700
#define	CAMERA_Y_ANGLE_CULVE 1700

#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240

/* For Vehicle Contol Condition */
#define PI_HALF 1.57
#define STOPLINE_THRESHOLD 3
#define PARKING_COUNT_THRESHOLD 2
#define PARKING_RING_DELAY 100
#define STRAIGHT_MOVFACTOR_THRESHOLD 80
#define OBS_DETECT_THRESHOLD 5

#define HILL_ACC_TIME_100MS 45
#define HILL_DSC_TIME_100MS 55
#define EXCAPE_TIME_100MS 30
#define ROTARY_EXCAPE_TIME_100MS 150

#define HILL_DETECT_DISTANCE_CM 20

#define VEHICLE_LANE_DISTANCE 15
#define DISTANCE_TOLERANCE 0
#define PARKPOST_DISTANCE (VEHICLE_LANE_DISTANCE + 13 + DISTANCE_TOLERANCE)
#define PARKPARALLEL_DISTANCE (VEHICLE_LANE_DISTANCE + 13 + 35 + DISTANCE_TOLERANCE)
#define PARKVERTICAL_DISTANCE (VEHICLE_LANE_DISTANCE + 13 + 45 + DISTANCE_TOLERANCE)
#define ROTARY_DISTANCE (10 + DISTANCE_TOLERANCE)
#define THREELANE_DISTANCE (25 + DISTANCE_TOLERANCE)

#define PARKING_SPEED 80


#define N_CENTRAL_MAX 90
#define N_CENTRAL_SUBMAX 60


#endif /* Macro definition */


/* Extern Variable form main */
extern volatile unsigned char lane_detect_state;

volatile int prior_CNT;
volatile char onthelane_state;	// state memory 1 : -> / 2 : <-

volatile float right_lane_theta;
volatile float left_lane_theta;

volatile unsigned long encoderCntL1, encoderCntL2;
volatile unsigned long encoderCntR1, encoderCntR2;

extern volatile _Bool start_state;	// starting condition
extern volatile _Bool vehicle_stop_state;

extern volatile _Bool StopLine_detect_flag;			// StopLine Detect State
extern volatile _Bool ReadyToParking_flag;
extern volatile char VerticalParking_detect_flag;	// Vertical Parking Lot  Detect State
extern volatile char ParallelParking_detect_flag;	// Parallel Parking Lot  Detect State

volatile int MeasuredDistance[6];			// updated distance value
volatile int ChDistanceParameter[6][3];		// pre-determinded distance parameter

volatile char leftParkingSequence[2];
volatile char rightParkingSequence[2];
volatile short leftSequenceCnt[2][2];
volatile short rightSequenceCnt[2][2];

int obs_cnt[6];
volatile _Bool obs_detect_state[6];

extern volatile _Bool Misson_HillDrive_Done;

extern volatile unsigned long long current_time;
extern volatile unsigned long long my_timer;
extern volatile _Bool reset_Mtimer;
extern volatile _Bool rotary_entered_state;
extern volatile _Bool Misson_RotaryDrive_Done;

extern volatile unsigned char sequence;		

volatile double m_central, m_curve, y_right, y_left;



#if 1

void my_sleep(int s);

/* Vehicle Initialization */
void vehicleInit(void);

/* Vehicle Emergency Stop */
void vehicleStop(void);

/* Make Buzzer Ring */
void BuzzerRing(int num, unsigned int delay_ms);

/* Vehicle Speed PID Setting */
void setVehicleSpeedPIDGain(int p_gain, int i_gain, int d_gain);

/* Vehicle EncoderCount PID Setting */
void setVehicleEncoderPIDGain(int p_pos_gain);

/* Vehicle Speed Setting */
void setVehicleSpeed(int speed); // backwaed -500 ~ 500 forward

/* Vehicle Direction Setting */
void setVehicleDirection(float movfactor); // left -100 ~ 100 right 

void setVehicleDirection_Value(unsigned int direction);

/* Vehicle Move Control with Time Setting */
void runtimeCotrol(int speed, int ms);

/* Vehicle Move Control with Distance Setting */
void positionControl(int speed, float cm);

/* Vehicle Direction Control using Vanishing Point */
void directionControl(int targetX);

/* Vehicle Speed Control by Situation */
void speedControl();

/* Covert Distance CM to Sensor Value */
int CvtcmToValue(int channel, int cm);

/* Comparate Distance with CM (Auto Converting to Sensor Value) */  
_Bool isCloserThanCM(int channel, int cm);

/* Comp Distance with Valuses */  
_Bool isInBoundryValue(int channel, int value_min, int value_max); 

/* Pre-Convertion of Distance Parameter for Multi-Threading */
void DistanceParameterInit();

/* Update the Distance State from Distance Sensor */
void updateDistanceState();

/* Update the Distance State from Line Sensor */
void update_LineState();

/* Update parking signal state to detect parking lot */
void update_ParkingSignalSequenceState();

/* Vertical Parking Sequence */
void parking_Vertical(int side);

/* Parallel Parking Sequence */
void parking_Parallel(int side);

/* Check the encoder count for parking */
void parking_Distance_Check();

/* Check the Obstacle */
void Obstacle_Checking();

/* Rotary Alghorithm */
int checkDiff(IplImage *frame);

/* parking sequence reset */
void reset_sequence();

#endif /* Fuction prototype */
