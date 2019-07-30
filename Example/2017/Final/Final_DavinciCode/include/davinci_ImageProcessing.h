#pragma once

#if 0
/*
*	Embeded Software Contest 2017
*	Davinci Code : ImageProcessing part
*	Copyright (c) 2017, Heremes Team. All rights reserved.
*/
#endif	/* copyright */

#if 1
/* base library */
#include <stdio.h>
#include <math.h>

/* OpenCV library */
#include <highgui.h>
#include <cv.h>

// For Visual Studio Test
//#include <opencv/cv.h>
//#include <opencv/highgui.h>  
#endif	/* Library import */


#if 1

#define EN_ROI_VARY 0

/* Skin Color Detection Threshold */
#define SC_lowH 165
#define SC_highH 20
#define SC_lowS 15
#define SC_highS 255
#define SC_lowV 0
#define SC_highV 255

/* Davinci Blue Color Detection Threshold */
#define BC_lowH 115
#define BC_highH 125
#define BC_lowS 115
#define BC_highS 200
#define BC_lowV 40 
#define BC_highV 140

/* Stop Sign RED Detection Threshold */
#define SSR_lowH 10
#define SSR_highH 170
#define SSR_lowS 120//150
#define SSR_highS 255
#define SSR_lowV  120
#define SSR_highV 180

/* Yellow Lane Detection Threshold */
#define LY_lowH 20
#define LY_highH 50	//40	
#define LY_lowS 100	//20
#define LY_highS 255
#define LY_lowV 80
#define LY_highV 255

/* White Lane Detection Threshold */
#define LW_lowH 70
#define LW_highH 100
#define LW_lowS 0
#define LW_highS 30
#define LW_lowV 170 //165
#define LW_highV 255

/* Vanishing Point ROI */
#define HALF_LENGTH 520//480	//160

#define VP_maxX (160+HALF_LENGTH) //320
#define VP_minX (160-HALF_LENGTH) //0
#define VP_maxY	240
#define VP_minY 0

/* Image Processing ROI ratio */
#define _ROI_Xmin 0.0f	
#define _ROI_Xmax 1.0f
#define _ROI_Ymin 0.5f
#define current_Y_min_RTY 0.6f
#define _ROI_Ymin_max 0.6f
#define _ROI_Ymax 1.0f	//0.9f

#define ONELANE_RATIO 0.0f

/* Lane DETERMINATION INFO */
#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240
#define MEMORY_CNT_MAX 2
#define CON_CURVE_SLOPE 0.577 // tan(30')
#define HORIZONTAL_SLOPE 1.41

/* Color Detection Codition */
#define SC_NUM_THRESHOLD 50000
#define SSR_NUM_THRESHOLD 700
#define STOPSIGN_DELAY_CNT 10

#endif /* Macro definition */

volatile unsigned char lane_detect_state;

extern volatile float right_lane_theta;
extern volatile float left_lane_theta;

extern volatile _Bool start_state;	// starting condition
extern volatile _Bool ready_state;   // Ready to start

extern volatile _Bool StopLine_detect_flag;			// StopLine Detect State
extern volatile _Bool StopSign_detect_flag;	// StopSign Detect State
extern volatile char stopSign_Cnt;

extern volatile _Bool reset_Mtimer;
extern volatile _Bool reset_Stimer;

extern volatile unsigned char sequence;

extern volatile unsigned long long my_timer;

#if 1

extern void my_sleep(int s);

/* Make Buzzer Ring */
extern void BuzzerRing(int num, unsigned int delay_ms);

/* Vehicle Speed Setting */
extern void setVehicleSpeed(int speed);

/* Lane Color Detection and Image Binarization */
void cvLaneBinariztionAndSignDetect(IplImage* src_hsv, IplImage *dst);

/* Lane Detection and Find Vanishing Point */
void cvDetectLaneVPoint(IplImage* org, IplImage* src, CvPoint* pre_targePoint);

/* Extract Vanishing Point of Two Lines  */
void extractVPoint(float* lineA, float* lineB, CvPoint* VPoint);

/* Display the Result */
void DisplayResult(IplImage *src, int situation);

/* Start Signal Detection */
void StartSignalDetect(IplImage *src_hsv);

/* 3-Lanes Alghorithm */
int countlW(IplImage *frame,int direc);

/* Traffic Light Discrimination */
int checkTL(IplImage *frame);

#endif /* Fuction prototype */
