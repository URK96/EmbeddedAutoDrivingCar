
#if 0
/*
 *	Embeded Software Contest 2017
 *	Davinci Code : ImageProcessing part
 *	Copyright (c) 2017, Heremes Team. All rights reserved.
 */	
#endif	/* Copyright */

#if 1
#include "davinci_ImageProcessing.h"
#endif	/* header import */

//#define DRAW_LANE_VP
#define DRAW_LANE_VP2
#define HOUGH_PROBABILISTIC
//#define CircleDetect

#if 1
 /* LANE INFOMATION - JWC */
typedef struct _laneInfo
{
	float rho;
	float theta;
	float slope;
	char memory_cnt;
	char uptodate;
}
laneInfo;
#endif /* Struct Definition */

laneInfo prior_leftLane = { RESIZE_WIDTH * 0.5, 0, 573, 0 }; // prior left lane information
laneInfo prior_rightLane = { RESIZE_WIDTH * 0.5, 0, -573, 0 }; // prior right lane information

volatile unsigned char prior_lane_detect_state = 0;

volatile double current_Y_min = _ROI_Ymin;
volatile int lane_Xmin, lane_Xmax,lane_Ymin, lane_Ymax;

volatile int first_move = 0;


 /* Lane Color Detection and Image Binarization */
void cvLaneBinariztionAndSignDetect(IplImage* src_hsv, IplImage *dst)
{
	int i, j;
	const unsigned char HUE = 0, SAT = 1, VAL = 2;
	unsigned int StopSignCnt = 0;
	lane_Xmin = src_hsv->width;
	lane_Xmax = 0;
	lane_Ymin = src_hsv->height;
	lane_Xmax = 0;

#ifdef CircleDetect
	IplImage *gray = cvCreateImage(cvGetSize(src_hsv), IPL_DEPTH_8U, 1);
#endif

	int ROI_Xs = (int)(src_hsv->width * _ROI_Xmin), ROI_Xd = (int)(src_hsv->width * _ROI_Xmax);
	int ROI_Yd = (int)(src_hsv->height * _ROI_Ymax);
	int ROI_Ys;
	if(my_timer < 20 && !first_move && start_state){
		 ROI_Ys = 0;
		// printf("%d \t %d \n",my_timer, first_move);
	}
	else if(sequence == 3 ||sequence == 2) ROI_Ys = (int)(src_hsv->height * current_Y_min_RTY);
	else{
		 ROI_Ys = (int)(src_hsv->height * current_Y_min);
		 first_move = 1;
		}
	
	
	for (i = 0; i < src_hsv->height; i++)
	{
		for (j = 0; j < src_hsv->width; j++) 
		{

			if(sequence == 8)
			{

				/* StopSign Red Count */
				if (
					((unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] <= SSR_lowH ||
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] >= SSR_highH) &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] >= SSR_lowS &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] <= SSR_highS &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] >= SSR_lowV &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] <= SSR_highV
					)
				{
					StopSignCnt++;
				}
			}



			if (/* Range to detect (Lane : Yellow or White) */
				// ROI range condition
				(i > MAX(0, ROI_Ys) && i < MIN(src_hsv->height, ROI_Yd) &&
				j > MAX(0, ROI_Xs) && j < MIN(src_hsv->width, ROI_Xd))
				&&
				// Color condition to detect
				(
					// Yellow Lane
					((unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] >= LY_lowH &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] <= LY_highH &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] >= LY_lowS &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] <= LY_highS &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] >= LY_lowV &&
					(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] <= LY_highV)
				//||
				//	
				//	// white Lane
				//	((unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] >= LW_lowH &&
				//	(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] <= LW_highH &&
				//	(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] >= LW_lowS &&
				//	(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] <= LW_highS &&
				//	(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] >= LW_lowV &&
				//	(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] <= LW_highV)
				)
			   )
			{
				dst->imageData[i*dst->widthStep + j] = (unsigned char)255;
				if (j < lane_Xmin) lane_Xmin = j;
				if (j > lane_Xmax) lane_Xmax = j;
				if (j < lane_Ymin) lane_Ymin = j;
				if (j > lane_Ymax) lane_Ymax = j;
			}
			else // elsewhere
			{
				dst->imageData[i*dst->widthStep + j] = 0;
			}

		}
	}
	if (StopSignCnt > SSR_NUM_THRESHOLD && !StopLine_detect_flag)
	{
			StopSign_detect_flag = 1;
			stopSign_Cnt = STOPSIGN_DELAY_CNT;
			printf("StopCNT : %d\t", StopSignCnt);
	}
	else StopSign_detect_flag = 0;
}

/* Lane Detection and Find Vanishing Point */
void cvDetectLaneVPoint(IplImage* org ,IplImage* src, CvPoint* pre_targePoint)
{
	int i;

#ifndef HOUGH_PROBABILISTIC 
	const int LANE_THRESHOLD = 200;
#endif

	int lp_num = 0, rp_num = 0, lane_num = 0;
	float rhol_sum = 0, rhor_sum = 0, thetal_sum = 0, thetar_sum = 0;
	float leftLane[2] = { 0 }, rightLane[2] = { 0 };

	IplImage *temp;
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvPoint targetPoint = *pre_targePoint;
	IplConvKernel *element;
	CvSeq* lines = 0;

	temp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 1);

	// Morphology (Reduce Noise)
	element = cvCreateStructuringElementEx(3, 3, 1, 1, CV_SHAPE_RECT, NULL); // Set filter size 3x3
	cvMorphologyEx(src, temp, NULL, element, CV_MOP_DILATE, 2);
	cvMorphologyEx(temp, temp, NULL, element, CV_MOP_ERODE, 3);
	
	// Canny Edge (Find Edge)
	cvCanny(temp, temp, 50, 150, 3);

	// Probabilistic Hough Transform (Find Line)
#ifdef HOUGH_PROBABILISTIC
	lines = cvHoughLines2(temp, storage, CV_HOUGH_PROBABILISTIC, 4, CV_PI / 180, 80, 30, 10);

	for (i = 0; i < MIN(lines->total, 20); i++)
	{
		int* line = (int*)cvGetSeqElem(lines, i);
		CvPoint g_pt1, g_pt2;
		g_pt1.x = line[0];
		g_pt1.y = line[1];
		g_pt2.x = line[2];
		g_pt2.y = line[3];

		if (g_pt1.x != g_pt2.x)
		{
			float slope = (float)(g_pt2.y - g_pt1.y) / (g_pt2.x - g_pt1.x);
			float y_intercept = g_pt1.y - g_pt1.x * slope;
			float rho = (float)(y_intercept >= 0 ? (fabs(y_intercept) / sqrt(1 + slope*slope)) : -(fabs(y_intercept) / sqrt(1 + slope*slope)));
			float theta = (float)(y_intercept != 0 ? CV_PI / 2 + atan(slope) : atan(slope));

			if (theta < HORIZONTAL_SLOPE || theta > -HORIZONTAL_SLOPE) // Remove Hrizontal Line (-10~10 degree)
			{
				if (slope < 0)
				{
					rhol_sum += rho;
					thetal_sum += theta;
					lp_num++;
#ifdef DRAW_LANE_VP
					cvLine(org, g_pt1, g_pt2, CV_RGB(255, 0, 255), 2, 4, 0);
#endif
				}
				else
				{
					rhor_sum += rho;
					thetar_sum += theta;
					rp_num++;
#ifdef DRAW_LANE_VP
					cvLine(org, g_pt1, g_pt2, CV_RGB(0, 255, 255), 2, 4, 0);
#endif
				}
			}
		}
	}
	// Standard Hough Transform (Find Line)
#else
	lines = cvHoughLines2(temp1, storage, CV_HOUGH_STANDARD, 4, CV_PI / 180, 200, 0, 10);	//230

	// Lane Determination
	for (i = 0; i < min(lines->total, 10); i++)
	{
		float* line = (float*)cvGetSeqElem(lines, i);
		float rho = line[0];
		float theta = line[1];
		
		// Lane classification by distance between detected line and origin (0, 0) & remove horizontal lines
		if (fabs(rho) > LANE_THRESHOLD && !(theta > 1.50  && theta < 1.64)) {
			rhol_sum += rho;
			thetal_sum += theta;
			lp_num++;
		}
		else if (fabs(rho) < LANE_THRESHOLD && !(theta > 1.50  && theta < 1.64)) {
			rhor_sum += rho;
			thetar_sum += theta;
			rp_num++;
		}
	}
#endif

	prior_leftLane.uptodate = 0;
	prior_rightLane.uptodate = 0;

	if (lp_num > 0) {	// Left Lane exist!

		leftLane[0] = rhol_sum / lp_num;
		leftLane[1] = thetal_sum / lp_num;

		prior_leftLane.rho = leftLane[0];
		prior_leftLane.theta = leftLane[1];
		prior_leftLane.slope = leftLane[0] != 0 ? (float)(tan(leftLane[1] - CV_PI / 2)) : (float)(tan(leftLane[1]));
		prior_leftLane.memory_cnt = 0;
		prior_leftLane.uptodate = 1;
		
		left_lane_theta = prior_leftLane.theta;		

		lane_num++;

#ifdef DRAW_LANE_VP2
		CvPoint l_pt1, l_pt2;
		float a = (float)cos(leftLane[1]), b = (float)sin(leftLane[1]);
		float x0 = a*leftLane[0], y0 = b*leftLane[0];
		l_pt1.x = cvRound(x0 + 1000 * (-b));
		l_pt1.y = cvRound(y0 + 1000 * (a));
		l_pt2.x = cvRound(x0 - 1000 * (-b));
		l_pt2.y = cvRound(y0 - 1000 * (a));
		cvLine(src, l_pt1, l_pt2, CV_RGB(0, 0, 255), 3, 4, 0);
#endif
	}
	if (rp_num > 0) {	// right Lane exist!

		rightLane[0] = rhor_sum / rp_num;
		rightLane[1] = thetar_sum / rp_num;

		prior_rightLane.rho = rightLane[0];
		prior_rightLane.theta = rightLane[1];
		prior_rightLane.slope = rightLane[0] != 0 ? (float)(tan(rightLane[1] - CV_PI / 2)) : (float)(tan(rightLane[1]));
		prior_rightLane.memory_cnt = 0;
		prior_rightLane.uptodate = 1;
		right_lane_theta = prior_rightLane.theta;		

		lane_num++;

#ifdef DRAW_LANE_VP2
		CvPoint r_pt1, r_pt2;
		float a = (float)cos(rightLane[1]), b = (float)sin(rightLane[1]);
		float x0 = a*rightLane[0], y0 = b*rightLane[0];
		r_pt1.x = cvRound(x0 + 1000 * (-b));
		r_pt1.y = cvRound(y0 + 1000 * (a));
		r_pt2.x = cvRound(x0 - 1000 * (-b));
		r_pt2.y = cvRound(y0 - 1000 * (a));
		cvLine(src, r_pt1, r_pt2, CV_RGB(0, 0, 255), 3, 4, 0);
#endif
	}


	if (lane_num == 2) // Two Lanes exist
	{
		if(EN_ROI_VARY) current_Y_min = current_Y_min < _ROI_Ymin ?  _ROI_Ymin : current_Y_min - 0.02;		

		extractVPoint(leftLane, rightLane, &targetPoint);
		
		if (targetPoint.x == 2017 && targetPoint.y == 2017) // Can't find Vanising Point
		{
			targetPoint = *pre_targePoint;
			lane_detect_state = 5;
		}
		else
		{
			targetPoint.x = targetPoint.x > VP_maxX ? VP_maxX :
				targetPoint.x < VP_minX ? VP_minX : targetPoint.x;
			targetPoint.y = targetPoint.y > VP_maxY ? VP_maxY :
				targetPoint.y < VP_minY ? VP_minY : targetPoint.y;

			lane_detect_state = 5;
		}
	}
	else if (lane_num == 1) // one Lane exist
	{
		if(EN_ROI_VARY) current_Y_min = current_Y_min > _ROI_Ymin_max ? _ROI_Ymin_max : current_Y_min + 0.02;

		if (lp_num)
		{
			if (prior_rightLane.memory_cnt < MEMORY_CNT_MAX
				&& fabs(prior_leftLane.slope) > CON_CURVE_SLOPE)
			{
				rightLane[0] = prior_rightLane.rho;
				rightLane[1] = prior_rightLane.theta;
				prior_rightLane.memory_cnt++;
				lane_detect_state = 5;
#ifdef DRAW_LANE_VP2
				CvPoint r_pt1, r_pt2;
				float a = (float)cos(rightLane[1]), b = (float)sin(rightLane[1]);
				float x0 = a*rightLane[0], y0 = b*rightLane[0];
				r_pt1.x = cvRound(x0 + 1000 * (-b));
				r_pt1.y = cvRound(y0 + 1000 * (a));
				r_pt2.x = cvRound(x0 - 1000 * (-b));
				r_pt2.y = cvRound(y0 - 1000 * (a));
				cvLine(src, r_pt1, r_pt2, CV_RGB(0, 0, 255), 3, 4, 0);
#endif
			}
			else
			{
				rightLane[0] = (float) (src->height * ONELANE_RATIO);
				rightLane[1] = (float)(0.5 * CV_PI);
				prior_rightLane.rho = rightLane[0];
				prior_rightLane.theta = rightLane[1];
				//--------------------------------------------------------------------------------------------------------------------------
				if (lane_Xmin > src->width * 0.3 && lane_Ymax < src->height * 0.5)
				{
					lane_detect_state = 0;
				}
				else
				{
					lane_detect_state = 1;
				}
			}
		}

		if (rp_num) 
		{
			if (prior_leftLane.memory_cnt < MEMORY_CNT_MAX
				&& prior_rightLane.slope > CON_CURVE_SLOPE)
			{
				leftLane[0] = prior_leftLane.rho;
				leftLane[1] = prior_leftLane.theta;
				prior_leftLane.memory_cnt++;
				lane_detect_state = 5;
#ifdef DRAW_LANE_VP2
				CvPoint l_pt1, l_pt2;
				float a = (float)cos(leftLane[1]), b = (float)sin(leftLane[1]);
				float x0 = a*leftLane[0], y0 = b*leftLane[0];
				l_pt1.x = cvRound(x0 + 1000 * (-b));
				l_pt1.y = cvRound(y0 + 1000 * (a));
				l_pt2.x = cvRound(x0 - 1000 * (-b));
				l_pt2.y = cvRound(y0 - 1000 * (a));
				cvLine(src, l_pt1, l_pt2, CV_RGB(0, 0, 255), 3, 4, 0);
#endif
			}
			else
			{
				leftLane[0] = (float)(src->height * ONELANE_RATIO);
				leftLane[1] = (float)(0.5 * CV_PI);
				prior_leftLane.rho = leftLane[0];
				prior_leftLane.theta = leftLane[1];
				//--------------------------------------------------------------------------------------------------------------------------
				if (lane_Xmax < src->width * 0.7 && lane_Ymax < src->height * 0.5)
				{
					lane_detect_state = 0;
				}
				else
				{
					lane_detect_state = 3;
				}
			}
		}
		extractVPoint(leftLane, rightLane, &targetPoint);
		
		if(targetPoint.x == 2017 && targetPoint.y == 2017)
		{
			targetPoint = *pre_targePoint;
		}
		else 
		{
			targetPoint.x = targetPoint.x > VP_maxX ? VP_maxX :
				targetPoint.x < VP_minX ? VP_minX : targetPoint.x;
			targetPoint.y = targetPoint.y > VP_maxY ? VP_maxY :
				targetPoint.y < VP_minY ? VP_minY : targetPoint.y;
		}

	}
	else // No Lane
	{
		if(EN_ROI_VARY) current_Y_min = current_Y_min < _ROI_Ymin ?  _ROI_Ymin : current_Y_min - 0.1;
		lane_detect_state = prior_lane_detect_state;
		targetPoint = *pre_targePoint;
	}

	// Update vanishing point
	*pre_targePoint = targetPoint;
	prior_lane_detect_state = lane_detect_state; 

	cvCircle(org, targetPoint, 3, CV_RGB(255, 255, 255), 3, 0, 0);
	switch (lane_detect_state)
	{
	case 0:
		DisplayResult(src, 7);
		break;
	case 1:
		DisplayResult(src, 1);
		break;
	case 2:
		DisplayResult(src, 2);
		break;
	case 3:
		DisplayResult(src, 3);
		break;
	case 4:
		DisplayResult(src, 4);
		break;
	case 5:
		DisplayResult(src, 5);
		break;
	default:
		break;
	}
#ifdef DRAW_LANE_VP2
	/* Draw ROI Range */
	int ROI_Xs = (int)(org->width * _ROI_Xmin), ROI_Xd = (int)(org->width * _ROI_Xmax);
	int ROI_Ys = (int)(org->height * current_Y_min), ROI_Yd = (int)(org->height * _ROI_Ymax);
	CvPoint p1 = { ROI_Xs, ROI_Ys };
	CvPoint p2 = { ROI_Xd, ROI_Ys };
	CvPoint p3 = { ROI_Xd, ROI_Yd };
	CvPoint p4 = { ROI_Xs, ROI_Yd };

	cvLine(src, p1, p2, CV_RGB(255, 255, 255), 2, 2, 0);
	//cvLine(src, p2, p3, CV_RGB(255, 255, 255), 1, 2, 0);
	cvLine(src, p3, p4, CV_RGB(255, 255, 255), 2, 2, 0);
	//cvLine(src, p4, p1, CV_RGB(255, 255, 255), 1, 2, 0);
#endif

	// Release memory
	cvReleaseImage(&temp);
	cvReleaseMemStorage(&storage);
}
#include <math.h>
/* Extract Vanishing Point from Two lines */
void extractVPoint(float* lineA, float* lineB, CvPoint* VPoint)
{

	double t, s;
	int alpha = 1000;
	float x0, y0;
	float cos_t, sin_t;

	cos_t = (float)cos(lineA[1]), sin_t = (float)sin(lineA[1]);
	x0 = cos_t*lineA[0], y0 = sin_t*lineA[0];
	CvPoint A_pt1 = { cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * (cos_t)) };
	CvPoint A_pt2 = { cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * (cos_t)) };

	cvRound((float)(x0 + alpha * (-sin_t)));
	cos_t = (float)cos(lineB[1]); sin_t = (float)sin(lineB[1]);
	x0 = cos_t*lineB[0]; y0 = sin_t*lineB[0];
	CvPoint B_pt1 = { cvRound(x0 + alpha * (-sin_t)), cvRound(y0 + alpha * (cos_t)) };
	CvPoint B_pt2 = { cvRound(x0 - alpha * (-sin_t)), cvRound(y0 - alpha * (cos_t)) };
	
	double denominator = (B_pt2.y - B_pt1.y)*(A_pt2.x - A_pt1.x) - (B_pt2.x - B_pt1.x)*(A_pt2.y - A_pt1.y);

	if (denominator)
	{
		double _t = (B_pt2.x - B_pt1.x)*(A_pt1.y - B_pt1.y) - (B_pt2.y - B_pt1.y)*(A_pt1.x - B_pt1.x);
		double _s = (A_pt2.x - A_pt1.x)*(A_pt1.y - B_pt1.y) - (A_pt2.y - A_pt1.y)*(A_pt1.x - B_pt1.x);

		t = _t / denominator;
		s = _s / denominator;

		if (!(t<0.0 || t>1.0 || s<0.0 || s>1.0)) 
		{
			if (!(_t == 0 && _s == 0)) {
				(*VPoint).x = (int)(A_pt1.x + t * (double)(A_pt2.x - A_pt1.x));
				(*VPoint).y = (int)(A_pt1.y + t * (double)(A_pt2.y - A_pt1.y));
			}
			else
			{
				(*VPoint).x = 0;
				(*VPoint).y = 0;
			}
		}
		else
		{
			(*VPoint).x = 2017;
			(*VPoint).y = 2017;
		}
	}
	else
	{
		(*VPoint).x = 2017;
		(*VPoint).y = 2017;
	}
}


/* Display the Result */ 
void DisplayResult(IplImage *src, int situation)
{

	CvFont font;
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.6, 0.6, 0, 1, CV_AA);

	switch (situation)
	{
	case 1:	// Detect Left Lane So Go Right
		cvPutText(src, "/ Go Right", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 2: // Although Detect Left Lane Go Left
		cvPutText(src, "Go Left /", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 3:	// Detect Right Lane So Go Left
		cvPutText(src, "Go Left \\", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 4: // Although Detect Right Lane Go Right
		cvPutText(src, "\\ Go Right", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 5: // Go Almost Straight
		cvPutText(src, "Go Straight", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 6:	// StopSign detected!
		cvPutText(src, "STOP", cvPoint(210, 40), &font, cvScalar(0, 0, 255, 0));
		break;
	case 7:	// No detected Keep Going!
		cvPutText(src, "Keep Going", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		break;
	case 101:	// No Vision!
		cvPutText(src, "No Vision!", cvPoint(10, 40), &font, cvScalar(255, 255, 255, 0));
		cvPutText(src, "Haven't started yet..", cvPoint(10, 90), &font, cvScalar(255, 255, 255, 0));
		break;
	default:
		cvPutText(src, "I don't know what to do!", cvPoint(10, 100), &font, cvScalar(255, 255, 255, 0));
		break;
	}
}

void StartSignalDetect(IplImage *src_hsv)
{
	int i, j;
	const unsigned char HUE = 0, SAT = 1, VAL = 2;

	unsigned int skin_color_num = 0;

	for (i = 0; i < src_hsv->height; i++)
	{
		for (j = 0; j <src_hsv->width; j++)
		{
			if (
				// Color condition to detect
				(
					// Skin Color
					(
						((unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] >= SC_lowH ||
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] <= SC_highH) &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] >= SC_lowS &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] <= SC_highS &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] >= SC_lowV &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] <= SC_highV)

					||

					// Davinci Blue Color
					(
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] >= BC_lowH &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + HUE] <= BC_highH &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] >= BC_lowS &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + SAT] <= BC_highS &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] >= BC_lowV &&
						(unsigned char)src_hsv->imageData[i*src_hsv->widthStep + 3 * j + VAL] <= BC_highV)
					)
				)
			{
				skin_color_num++;
			}
		}
	}

	if (ready_state)
	{
		if (src_hsv->height*src_hsv->width - skin_color_num > SC_NUM_THRESHOLD)
		{
			start_state = 1;
			ready_state = 0;
			BuzzerRing(2, 100);
			//reset_Stimer = 1;	// ON-Time
			reset_Mtimer = 1;	// My-Time
			//setVehicleSpeed(50);
		}
	}
	else 
	{
		if (skin_color_num > SC_NUM_THRESHOLD) ready_state = 1;
	}
}


int countlW(IplImage *frame,int direc) 
{

	int count = 0;
	int h, w;

	IplImage *gray = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	IplImage *canny = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);

	cvCvtColor(frame, gray, CV_RGB2GRAY);
	cvCanny(gray, canny, 50, 70, 3);
	if (direc == 1) {
		for (h = 90; h < canny->height - 70; h++) {
			for (w = 70; w < canny->width - 80; w++) {

				unsigned char p = canny->imageData[h*canny->width + w];

				int P = (int)p;

				if (P != 0) {
					count++;
				}
			}
		}
	}
	if (direc == 2) {
		for (h = 20; h < canny->height - 120; h++) {
			for (w = 90; w < canny->width - 90; w++) {

				unsigned char p = canny->imageData[h*canny->width + w];

				int P = (int)p;

				if (P != 0) {
					count++;
				}
			}
		}
	}
	if (direc == 3) {
		for (h = 90; h < canny->height - 70; h++) {
			for (w = 100; w < canny->width - 50; w++) {

				unsigned char p = canny->imageData[h*canny->width + w];

				int P = (int)p;

				if (P != 0) {
					count++;
				}
			}
		}
	}

	cvReleaseImage(&gray);
	cvReleaseImage(&canny);
	
	return count;
}



int checkTL(IplImage *frame)
{
	IplImage *conversion = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 1);
	CvMemStorage *stg = cvCreateMemStorage(0);
	int h, w;
	int rcount = 0;
	int ycount = 0;
	int gcount = 0;
	int widthIdx = 0;

	int idx;
	unsigned char cB, cG, cR;
	int R, G, B;

	for (h = 50; h < frame->height; h++)
	{
		for (w = 0; w < frame->widthStep; w = w + 3)
		{
			idx = h*frame->widthStep + w;

			cB = frame->imageData[idx + 0];
			cG = frame->imageData[idx + 1];
			cR = frame->imageData[idx + 2];

			R = (int)cR;
			G = (int)cG;
			B = (int)cB;
			//BGR = HSV

			if ((170 < B || B < 10) && (150 < G && G < 255) && (90 < R && R < 180))
			{
				rcount++;
			}
			else if ((20 < B && B < 30) && (200 < G) && (110 < R && R < 153))
			{
				ycount++;
			}
			else if (((B <= 100) && (B >= 70)) && ((G <= 255) && (G >= 150)) && ((R <= 80) && (R >= 50)))
			{
				conversion->imageData[widthIdx] = (unsigned char)255;
				gcount++;
			}
			else conversion->imageData[widthIdx] = 0;

			widthIdx++;
		}
	}
	CvSeq *circle = cvHoughCircles(conversion, stg, CV_HOUGH_GRADIENT, 1, 1, 100, 8, 5, 12);

	

	if (rcount > 250) 
	{
		cvReleaseImage(&conversion);
		return 1;
	}
	else if (ycount > 250) 
	{
		cvReleaseImage(&conversion);
		return 2;
	}
	else if (gcount > 100)
	{
		if (circle->total >= 1) 
		{
			cvReleaseImage(&conversion);
 			return 3;
		}
		else 
		{
			cvReleaseImage(&conversion);
 			return 4;
		}
	}
	else 
	{
		cvReleaseImage(&conversion);
		return 0;
	}	
}



