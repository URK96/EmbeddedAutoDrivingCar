#if 0
/*
 * Copyright (c) 2012-2013, NVIDIA CORPORATION. All rights reserved.
 * All information contained herein is proprietary and confidential to NVIDIA
 * Corporation.  Any use, reproduction, or disclosure without the written
 * permission of NVIDIA Corporation is prohibited.
 */

 /*
 *	Embeded Software Contest 2017
 *	Davinci Code : Main part
 *	Copyright (c) 2017, Heremes Team. All rights reserved. only for additional part.
 */

#endif	/* Copyright */

#if 1
/* base library */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/time.h>
#include <unistd.h>     // for sleep

/* NVEDIA library */
#include "nvthread.h"
#include <nvcommon.h>
#include <nvmedia.h>

/* board library */
#include <testutil_board.h>
#include <testutil_capture_input.h>

/* OpenCV library */
#include <highgui.h>
#include <cv.h>

//For Window Visual Studio Test
//#include <opencv\highgui.h>
//#include <opencv\cv.h>
//#include <time.h>
//#include <sys\utime.h> 

/* ETC library */
#include <ResTable_720To320.h>	// for Resizing Vedio Image
#include <pthread.h>	// for Multi-theading

/* Vehicle control library */
#include <davinci_ImageProcessing.h>

/* Image processing libarary */
#include <davinci_VehicleControl.h>

#endif	/* Library import */

extern void CameraXServoControl_Write(signed short angle);
extern void CameraYServoControl_Write(signed short angle);
extern void Alarm_Write(char status);

#if 1

/* Macro */ //---------------------------------------------------------------------------
#define IWANNATIMECHECK
//#define IWANNATIMECHECK2
//#define IWANNACAPTURE_ORG
//#define IWANNACAPTURE_BNY
//#define IWANNACAPTURE_PRC
#define JUST_TEST
#define DRIVING_TEST
//#define HILL_TEST 
//#define PARKING_TEST 	
//#define DISTANCE_TEST

#define VIP_BUFFER_SIZE 6
#define VIP_FRAME_TIMEOUT_MS 100
#define VIP_NAME "vip"
#define MESSAGE_PRINTF printf
#define RESIZE_WIDTH  320
#define RESIZE_HEIGHT 240
#define CRC32_POLYNOMIAL 0xEDB88320L

/* Structure Declaration */
typedef struct
{
    I2cId i2cDevice;
  
    CaptureInputDeviceId vipDeviceInUse;
    NvMediaVideoCaptureInterfaceFormat vipInputtVideoStd;
    unsigned int vipInputWidth;
    unsigned int vipInputHeight;
    float vipAspectRatio;

    unsigned int vipMixerWidth;
    unsigned int vipMixerHeight;

    NvBool vipDisplayEnabled;
    NvMediaVideoOutputType vipOutputType;
    NvMediaVideoOutputDevice vipOutputDevice[2];
    NvBool vipFileDumpEnabled;
    char * vipOutputFileName;

    unsigned int vipCaptureTime;
    unsigned int vipCaptureCount;
} TestArgs;

typedef struct
{
    NvMediaVideoSurface *surf;
    NvBool last;
} QueueElem;

typedef struct
{
    char *name;

    NvSemaphore *semStart, *semDone;

    NvMediaVideoCapture *capture;
    NvMediaVideoMixer *mixer;
    FILE *fout;

    unsigned int inputWidth;
    unsigned int inputHeight;

    unsigned int timeout;

    NvBool displayEnabled;
    NvBool fileDumpEnabled;

    NvBool timeNotCount;
    unsigned int last;
} CaptureContext;


/* Global Constant */
static NvMediaVideoSurface *capSurf = NULL;
static NvBool stop = NVMEDIA_FALSE;


/* My Global Variable */
CvPoint tPoint = { 0, 0 };			// prior target point
volatile _Bool start_state;	// starting condition
volatile _Bool ready_state;  	// Ready to start
volatile _Bool vehicle_stop_state;	// Vehicle Stopped : 1  /  Vehicle Moving : 0

volatile _Bool StopLine_detect_flag;			// StopLine Detect State	// Detected : 1  /  Not detected : 0
volatile _Bool StopSign_detect_flag;			// StopSign Detect State	// Detected : 1  /  Not detected : 0
volatile _Bool ReadyToParking_flag;
volatile char VerticalParking_detect_flag;	// Vertical Parking Lot  Detect State	// left : 1 / right : 2 / Not detected : 0
volatile char ParallelParking_detect_flag;	// Parallel Parking Lot  Detect State	// left : 1 / right : 2 / Not detected : 0
volatile char stopSign_Cnt;

volatile _Bool Misson_StopSign_Done;
volatile _Bool Misson_HillDrive_Done;
volatile _Bool Misson_Parkikng_Done;
volatile _Bool Misson_RotaryDrive_Done;
volatile _Bool Misson_LineChange_Done;
volatile _Bool Misson_TrafficLight_Done;
volatile _Bool Misson_ENDLine_Done;

volatile unsigned long long current_time;
volatile unsigned long long my_timer;
volatile _Bool reset_Mtimer;
volatile _Bool reset_Stimer;

volatile _Bool JustOnce_VP_flag;	// condition for VP once	// excuted : 1  /  Not yet  : 0
volatile _Bool JustOnce_PP_flag;  // condition for PP once	// excuted : 1  /  Not yet  : 0
volatile _Bool rotary_entered_state;

extern volatile char leftParkingSequence[];
extern volatile char rightParkingSequence[];
extern volatile short leftSequenceCnt[][2];
extern volatile short rightSequenceCnt[][2];
extern volatile int ChDistanceParameter[][3];			// pre-determinded distance parameter
extern volatile int MeasuredDistance[];				// updated distance value

extern volatile unsigned char lane_detect_state; 			// Lane detection state

extern volatile _Bool obs_detect_state[6];

volatile unsigned char sequence;		

// Misson sequence	1: StopSign or HillDrive  2: Parking  3: RotaryDrive  4: LineChange  5: Traffic Light  6: ENDLine


/* Basic Global Variable */
int table_298[256];
int table_409[256];
int table_100[256];
int table_208[256];
int table_516[256];

/* Global Object */
pthread_cond_t      cond = PTHREAD_COND_INITIALIZER;
pthread_cond_t      Sensorcond = PTHREAD_COND_INITIALIZER;

pthread_mutex_t     mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t     sensor_mutex = PTHREAD_MUTEX_INITIALIZER;

#endif	/* Global things and Structure */

#if 1

/* About thread */
static unsigned int CaptureThread(void *params);
void *ControlThread(void *unused);
void *SensorDetectThread(void *unused);

/* About frame */
static int Frame2Ipl(IplImage* img);
static int DumpFrame(FILE *fout, NvMediaVideoSurface *surf);

/* About display */
static void CheckDisplayDevice(NvMediaVideoOutputDevice deviceType, NvMediaBool *enabled, unsigned int *displayId);

/* About time-check */
static void GetTime(NvMediaTime *time);
static void AddTime(NvMediaTime *time, NvU64 uSec, NvMediaTime *res);
static NvBool SubTime(NvMediaTime *time1, NvMediaTime *time2);

/* etc */
static void DisplayUsage(void);
static int ParseOptions(int argc, char *argv[], TestArgs *args);
static void SignalHandler(int signal);

#endif	/* Fuction prototype */


/***************************** Main Function *****************************/

int main(int argc, char *argv[])
{

#if 1
    int err = -1;
    TestArgs testArgs;

    CaptureInputHandle handle;

    NvMediaVideoCapture *vipCapture = NULL;
    NvMediaDevice *device = NULL;
    NvMediaVideoMixer *vipMixer = NULL;
    NvMediaVideoOutput *vipOutput[2] = {NULL, NULL};
    NvMediaVideoOutput *nullOutputList[1] = {NULL};
    FILE *vipFile = NULL;

    NvSemaphore *vipStartSem = NULL, *vipDoneSem = NULL;
    NvThread *vipThread = NULL;

    CaptureContext vipCtx;
    NvMediaBool deviceEnabled = NVMEDIA_FALSE;
    unsigned int displayId;
    
    pthread_t cntThread, senThread;
    
#endif	/* Variable Declaration and Creat Object for using in main */
	
    signal(SIGINT, SignalHandler);				// if Interrupt occurs, set stop = TRUE
	memset(&testArgs, 0, sizeof(TestArgs));		// memory allocation

    if(!ParseOptions(argc, argv, &testArgs))	// distinguish the program option
        return -1;	// Error occur

#if 1
    printf("1. Create NvMedia capture \n");
    // Create NvMedia capture(s)
    switch (testArgs.vipDeviceInUse)
    {
        case AnalogDevices_ADV7180:
            break;
        case AnalogDevices_ADV7182:
        {
            CaptureInputConfigParams params;

            params.width = testArgs.vipInputWidth;
            params.height = testArgs.vipInputHeight;
            params.vip.std = testArgs.vipInputtVideoStd;

            if(testutil_capture_input_open(testArgs.i2cDevice, testArgs.vipDeviceInUse, NVMEDIA_TRUE, &handle) < 0)
            {
                MESSAGE_PRINTF("Failed to open VIP device\n");
                goto fail;
            }

            if(testutil_capture_input_configure(handle, &params) < 0)
            {
                MESSAGE_PRINTF("Failed to configure VIP device\n");
                goto fail;
            }

            break;
        }
        default:
            MESSAGE_PRINTF("Bad VIP device\n");
            goto fail;
    }	// Device setting as device
    
    if(!(vipCapture = NvMediaVideoCaptureCreate(testArgs.vipInputtVideoStd, // interfaceFormat
                                                NULL, // settings
                                                VIP_BUFFER_SIZE)))// numBuffers
    {
        MESSAGE_PRINTF("NvMediaVideoCaptureCreate() failed for vipCapture\n");
        goto fail;
    }
#endif	// Step1 : Ready for frame capture

#if 1
    printf("2. Create NvMedia device \n");
    // Create NvMedia device
    if(!(device = NvMediaDeviceCreate()))
    {
        MESSAGE_PRINTF("NvMediaDeviceCreate() failed\n");
        goto fail;
    }
#endif	// Step2 : Create device object

#if 1
    printf("3. Create NvMedia mixer(s) and output(s) and bind them \n");
    // Create NvMedia mixer(s) and output(s) and bind them

	unsigned int features = 0;

    features |= NVMEDIA_VIDEO_MIXER_FEATURE_VIDEO_SURFACE_TYPE_YV16X2;
    features |= NVMEDIA_VIDEO_MIXER_FEATURE_PRIMARY_VIDEO_DEINTERLACING; // Bob the 16x2 format by default
    
	if(testArgs.vipOutputType != NvMediaVideoOutputType_OverlayYUV)
        features |= NVMEDIA_VIDEO_MIXER_FEATURE_DVD_MIXING_MODE;

    if(!(vipMixer = NvMediaVideoMixerCreate(device, // device
                                            testArgs.vipMixerWidth, // mixerWidth
                                            testArgs.vipMixerHeight, // mixerHeight
                                            testArgs.vipAspectRatio, //sourceAspectRatio
                                            testArgs.vipInputWidth, // primaryVideoWidth
                                            testArgs.vipInputHeight, // primaryVideoHeight
                                            0, // secondaryVideoWidth
                                            0, // secondaryVideoHeight
                                            0, // graphics0Width
                                            0, // graphics0Height
                                            0, // graphics1Width
                                            0, // graphics1Height
                                            features , // features
                                            nullOutputList))) // outputList
    {
        MESSAGE_PRINTF("NvMediaVideoMixerCreate() failed for vipMixer\n");
        goto fail;
    }
#endif	// Step3 : Setting Mixer and Output

#if 1
    printf("4. Check that the device is enabled (initialized) \n");
    // Check that the device is enabled (initialized)
    CheckDisplayDevice(
        testArgs.vipOutputDevice[0],
        &deviceEnabled,
        &displayId);

    if((vipOutput[0] = NvMediaVideoOutputCreate(testArgs.vipOutputType, // outputType
                                                testArgs.vipOutputDevice[0], // outputDevice
                                                NULL, // outputPreference
                                                deviceEnabled, // alreadyCreated
                                                displayId, // displayId
                                                NULL))) // displayHandle
    {
        if(NvMediaVideoMixerBindOutput(vipMixer, vipOutput[0], NVMEDIA_OUTPUT_DEVICE_0) != NVMEDIA_STATUS_OK)
        {
            MESSAGE_PRINTF("Failed to bind VIP output to mixer\n");
            goto fail;
        }
    }
    else
    {
        MESSAGE_PRINTF("NvMediaVideoOutputCreate() failed for vipOutput\n");
        goto fail;
    }
#endif	// Step4 : Check whether device is usable
 
#if 1
    printf("5. Open output file(s) \n");
    // Open output file(s)
    if(testArgs.vipFileDumpEnabled)
    {
        vipFile = fopen(testArgs.vipOutputFileName, "w");
        if(!vipFile || ferror(vipFile))
        {
            MESSAGE_PRINTF("Error opening output file for VIP\n");
            goto fail;
        }
    }
#endif	// Step5 : Connect to output file stream to save 

#if 1
    printf("6. Create vip pool(s), queue(s), fetch threads and stream start/done semaphores \n");
    // Create vip pool(s), queue(s), fetch threads and stream start/done semaphores
    if(NvSemaphoreCreate(&vipStartSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipStartSem\n");
        goto fail;
    }

    if(NvSemaphoreCreate(&vipDoneSem, 0, 1) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvSemaphoreCreate() failed for vipDoneSem\n");
        goto fail;
    }

    vipCtx.name = VIP_NAME;

    vipCtx.semStart = vipStartSem;
    vipCtx.semDone = vipDoneSem;

    vipCtx.capture = vipCapture;
    vipCtx.mixer = vipMixer;
    vipCtx.fout = vipFile;

    vipCtx.inputWidth = testArgs.vipInputWidth;
    vipCtx.inputHeight = testArgs.vipInputHeight;

    vipCtx.timeout = VIP_FRAME_TIMEOUT_MS;

    vipCtx.displayEnabled = testArgs.vipDisplayEnabled;
    vipCtx.fileDumpEnabled = testArgs.vipFileDumpEnabled;

    if(testArgs.vipCaptureTime)
    {
        vipCtx.timeNotCount = NVMEDIA_TRUE;
        vipCtx.last = testArgs.vipCaptureTime;
    }
    else
    {
        vipCtx.timeNotCount = NVMEDIA_FALSE;
        vipCtx.last = testArgs.vipCaptureCount;
    }

    if(NvThreadCreate(&vipThread, CaptureThread, &vipCtx, NV_THREAD_PRIORITY_NORMAL) != RESULT_OK)
    {
        MESSAGE_PRINTF("NvThreadCreate() failed for vipThread\n");
        goto fail;
    }

    printf("wait for ADV7182 ... one second\n");
    sleep(1);
#endif	// Step6 : Create thread and semaphore

#if 1
    printf("7. Kickoff \n");
    // Kickoff
    NvMediaVideoCaptureStart(vipCapture);
    NvSemaphoreIncrement(vipStartSem);
#endif	// Step7 : Start to capture the camera frame and use semaphore

#if 1
    printf("8. Vehicle Initialization & Control Thread\n");
	
	/* Vehicle Initialization */
	vehicleInit();

	/* Vehicle Control thread */
    	pthread_create(&cntThread, NULL, &ControlThread, NULL); 
	
	/* Sensor state update thread */
	pthread_create(&senThread, NULL, &SensorDetectThread, NULL);

#endif	// Step8 : Vehicle Initialization and Crate Thread (Insert another thread you made here for multi-thread)

#if 1
    printf("9. Wait for completion \n");
    // Wait for completion
    NvSemaphoreDecrement(vipDoneSem, NV_TIMEOUT_INFINITE);

    err = 0;
#endif	// Step9 : Check the Operation

/* Move to this point if there is an error in above operation */
#if 1
fail: // Run down sequence
    // Destroy vip threads and stream start/done semaphores
    if(vipThread)
        NvThreadDestroy(vipThread);
    if(vipDoneSem)
        NvSemaphoreDestroy(vipDoneSem);
    if(vipStartSem)
        NvSemaphoreDestroy(vipStartSem);

    printf("10. Close output file(s) \n");
	
	start_state = NVMEDIA_FALSE;
	vehicleStop();	// STOP the Vehicle! ----------------------------------------------------------------------------
	BuzzerRing(2, 300);

    // Close output file(s)
    if(vipFile)
        fclose(vipFile);
        
    // Unbind NvMedia mixer(s) and output(s) and destroy them
    if(vipOutput[0])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[0], NULL);
        NvMediaVideoOutputDestroy(vipOutput[0]);
    }
    if(vipOutput[1])
    {
        NvMediaVideoMixerUnbindOutput(vipMixer, vipOutput[1], NULL);
        NvMediaVideoOutputDestroy(vipOutput[1]);
    }
    if(vipMixer)
        NvMediaVideoMixerDestroy(vipMixer);


    // Destroy NvMedia device
    if(device)
        NvMediaDeviceDestroy(device);

    // Destroy NvMedia capture(s)
    if(vipCapture)
    {
        NvMediaVideoCaptureDestroy(vipCapture);

        // Reset VIP settings of the board
        switch (testArgs.vipDeviceInUse)
        {
            case AnalogDevices_ADV7180: // TBD
                break;
            case AnalogDevices_ADV7182: // TBD
                //testutil_capture_input_close(handle);
                break;
            default:
                break;
        }
    }
#endif	// Step10 : Release memory and disconect the outfile stream

    return err;
}


/*********************** Above Fuction Description ***********************/
static void SignalHandler(int signal)
{
	stop = NVMEDIA_TRUE;
	MESSAGE_PRINTF("%d signal received\n", signal);
}


static void GetTime(NvMediaTime *time)
{
	struct timeval t;

	gettimeofday(&t, NULL);

	time->tv_sec = t.tv_sec;
	time->tv_nsec = t.tv_usec * 1000;
}

static void AddTime(NvMediaTime *time, NvU64 uSec, NvMediaTime *res)
{
	NvU64 t, newTime;

	t = (NvU64)time->tv_sec * 1000000000LL + (NvU64)time->tv_nsec;
	newTime = t + uSec * 1000LL;
	res->tv_sec = newTime / 1000000000LL;
	res->tv_nsec = newTime % 1000000000LL;
}

//static NvS64 SubTime(NvMediaTime *time1, NvMediaTime *time2)
static NvBool SubTime(NvMediaTime *time1, NvMediaTime *time2)
{
	NvS64 t1, t2, delta;

	t1 = (NvS64)time1->tv_sec * 1000000000LL + (NvS64)time1->tv_nsec;
	t2 = (NvS64)time2->tv_sec * 1000000000LL + (NvS64)time2->tv_nsec;
	delta = t1 - t2;

	//    return delta / 1000LL;
	return delta > 0LL;
}

static void DisplayUsage(void)
{
	printf("Usage : nvmedia_capture [options]\n");
	printf("Brief: Displays this help if no arguments are given. Engages the respective capture module whenever a single \'c\' or \'v\' argument is supplied using default values for the missing parameters.\n");
	printf("Options:\n");
	printf("-va <aspect ratio>    VIP aspect ratio (default = 1.78 (16:9))\n");
	printf("-vmr <width>x<height> VIP mixer resolution (default 800x480)\n");
	printf("-vf <file name>       VIP output file name; default = off\n");
	printf("-vt [seconds]         VIP capture duration (default = 10 secs); overridden by -vn; default = off\n");
	printf("-vn [frames]          # VIP frames to be captured (default = 300); default = on if -vt is not used\n");
}

static int ParseOptions(int argc, char *argv[], TestArgs *args)
{
	int i = 1;

	// Set defaults if necessary - TBD
	args->i2cDevice = I2C4;     // i2c chnnel

	args->vipDeviceInUse = AnalogDevices_ADV7182;
	args->vipInputtVideoStd = NVMEDIA_VIDEO_CAPTURE_INTERFACE_FORMAT_VIP_NTSC;
	args->vipInputWidth = 720;
	args->vipInputHeight = 480;
	args->vipAspectRatio = 0.0f;

	args->vipMixerWidth = 800;
	args->vipMixerHeight = 480;

	args->vipDisplayEnabled = NVMEDIA_FALSE;
	args->vipOutputType = NvMediaVideoOutputType_OverlayYUV; ; // NvMediaVideoOutputType_OverlayRGB;
	args->vipOutputDevice[0] = NvMediaVideoOutputDevice_LVDS;
	args->vipFileDumpEnabled = NVMEDIA_FALSE;
	args->vipOutputFileName = NULL;

	args->vipCaptureTime = 0;
	args->vipCaptureCount = 0;


	if (i < argc && argv[i][0] == '-')
	{
		while (i < argc && argv[i][0] == '-')
		{
			if (i > 1 && argv[i][1] == '-')
			{
				MESSAGE_PRINTF("Using basic and custom options together is not supported\n");
				return 0;
			}

			// Get options
			if (!strcmp(argv[i], "-va"))
			{
				if (++i < argc)
				{
					if (sscanf(argv[i], "%f", &args->vipAspectRatio) != 1 || args->vipAspectRatio <= 0.0f) // TBC
					{
						MESSAGE_PRINTF("Bad VIP aspect ratio: %s\n", argv[i]);
						return 0;
					}
				}
				else
				{
					MESSAGE_PRINTF("Missing VIP aspect ratio\n");
					return 0;
				}
			}
			else if (!strcmp(argv[i], "-vmr"))
			{
				if (++i < argc)
				{
					if (sscanf(argv[i], "%ux%u", &args->vipMixerWidth, &args->vipMixerHeight) != 2)
					{
						MESSAGE_PRINTF("Bad VIP mixer resolution: %s\n", argv[i]);
						return 0;
					}
				}
				else
				{
					MESSAGE_PRINTF("Missing VIP mixer resolution\n");
					return 0;
				}
			}
			else if (!strcmp(argv[i], "-vf"))
			{
				args->vipFileDumpEnabled = NVMEDIA_TRUE;
				if (++i < argc)
					args->vipOutputFileName = argv[i];
				else
				{
					MESSAGE_PRINTF("Missing VIP output file name\n");
					return 0;
				}
			}
			else if (!strcmp(argv[i], "-vt"))
			{
				if (++i < argc)
					if (sscanf(argv[i], "%u", &args->vipCaptureTime) != 1)
					{
						MESSAGE_PRINTF("Bad VIP capture duration: %s\n", argv[i]);
						return 0;
					}
			}
			else if (!strcmp(argv[i], "-vn"))
			{
				if (++i < argc)
					if (sscanf(argv[i], "%u", &args->vipCaptureCount) != 1)
					{
						MESSAGE_PRINTF("Bad VIP capture count: %s\n", argv[i]);
						return 0;
					}
			}
			else
			{
				MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
				return 0;
			}

			i++;
		}
	}

	if (i < argc)
	{
		MESSAGE_PRINTF("%s is not a supported option\n", argv[i]);
		return 0;
	}

	// Check for consistency
	if (i < 2)
	{
		DisplayUsage();
		return 0;
	}


	if (args->vipAspectRatio == 0.0f)
		args->vipAspectRatio = 1.78f;

	if (!args->vipDisplayEnabled && !args->vipFileDumpEnabled)
		args->vipDisplayEnabled = NVMEDIA_TRUE;


	if (!args->vipCaptureTime && !args->vipCaptureCount)
		args->vipCaptureCount = 300;
	else if (args->vipCaptureTime && args->vipCaptureCount)
		args->vipCaptureTime = 0;



	return 1;
}

static int DumpFrame(FILE *fout, NvMediaVideoSurface *surf)
{
	NvMediaVideoSurfaceMap surfMap;
	unsigned int width, height;

	if (NvMediaVideoSurfaceLock(surf, &surfMap) != NVMEDIA_STATUS_OK)
	{
		MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in DumpFrame()\n");
		return 0;
	}

	width = surf->width;
	height = surf->height;

	unsigned char *pY[2] = { surfMap.pY, surfMap.pY2 };
	unsigned char *pU[2] = { surfMap.pU, surfMap.pU2 };
	unsigned char *pV[2] = { surfMap.pV, surfMap.pV2 };
	unsigned int pitchY[2] = { surfMap.pitchY, surfMap.pitchY2 };
	unsigned int pitchU[2] = { surfMap.pitchU, surfMap.pitchU2 };
	unsigned int pitchV[2] = { surfMap.pitchV, surfMap.pitchV2 };
	unsigned int i, j;

	for (i = 0; i < 2; i++)
	{
		for (j = 0; j < height / 2; j++)
		{
			fwrite(pY[i], width, 1, fout);
			pY[i] += pitchY[i];
		}
		for (j = 0; j < height / 2; j++)
		{
			fwrite(pU[i], width / 2, 1, fout);
			pU[i] += pitchU[i];
		}
		for (j = 0; j < height / 2; j++)
		{
			fwrite(pV[i], width / 2, 1, fout);
			pV[i] += pitchV[i];
		}
	}


	NvMediaVideoSurfaceUnlock(surf);

	return 1;
}

static int Frame2Ipl(IplImage* img)
{
	NvMediaVideoSurfaceMap surfMap;
	unsigned int resWidth, resHeight;
	int r, g, b;
	unsigned char y, u, v;
	int num;

	if (NvMediaVideoSurfaceLock(capSurf, &surfMap) != NVMEDIA_STATUS_OK)
	{
		MESSAGE_PRINTF("NvMediaVideoSurfaceLock() failed in Frame2Ipl()\n");
		return 0;
	}

	unsigned char *pY[2] = { surfMap.pY, surfMap.pY2 };
	unsigned char *pU[2] = { surfMap.pU, surfMap.pU2 };
	unsigned char *pV[2] = { surfMap.pV, surfMap.pV2 };
	unsigned int pitchY[2] = { surfMap.pitchY, surfMap.pitchY2 };
	unsigned int pitchU[2] = { surfMap.pitchU, surfMap.pitchU2 };
	unsigned int pitchV[2] = { surfMap.pitchV, surfMap.pitchV2 };
	unsigned int i, j, k, x;
	unsigned int stepY, stepU, stepV;

	resWidth = RESIZE_WIDTH;
	resHeight = RESIZE_HEIGHT;

	// Frame2Ipl
	img->nSize = 112;
	img->ID = 0;
	img->nChannels = 3;
	img->alphaChannel = 0;
	img->depth = IPL_DEPTH_8U;    // 8
	img->colorModel[0] = 'R';
	img->colorModel[1] = 'G';
	img->colorModel[2] = 'B';
	img->channelSeq[0] = 'B';
	img->channelSeq[1] = 'G';
	img->channelSeq[2] = 'R';
	img->dataOrder = 0;
	img->origin = 0;
	img->align = 4;
	img->width = resWidth;
	img->height = resHeight;
	img->imageSize = resHeight*resWidth * 3;
	img->widthStep = resWidth * 3;
	img->BorderMode[0] = 0;
	img->BorderMode[1] = 0;
	img->BorderMode[2] = 0;
	img->BorderMode[3] = 0;
	img->BorderConst[0] = 0;
	img->BorderConst[1] = 0;
	img->BorderConst[2] = 0;
	img->BorderConst[3] = 0;

	stepY = 0;
	stepU = 0;
	stepV = 0;
	i = 0;

	for (j = 0; j < resHeight; j++)
	{
		for (k = 0; k < resWidth; k++)
		{
			x = ResTableX_720To320[k];
			y = pY[i][stepY + x];
			u = pU[i][stepU + x / 2];
			v = pV[i][stepV + x / 2];

			// YUV to RGB
			r = y + 1.4075*(v - 128);
			g = y - 0.34455*(u - 128) - 0.7169*(v - 128);
			b = y + 1.779*(u - 128);


			r = r>255 ? 255 : r<0 ? 0 : r;
			g = g>255 ? 255 : g<0 ? 0 : g;
			b = b>255 ? 255 : b<0 ? 0 : b;


			num = 3 * k + 3 * resWidth*(j);
			img->imageData[num] = b;
			img->imageData[num + 1] = g;
			img->imageData[num + 2] = r;
			//img->imageDataOrigin[num] = b;
			//img->imageDataOrigin[num+1] = g;
			//img->imageDataOrigin[num+2] = r;


		}
		stepY += pitchY[i];
		stepU += pitchU[i];
		stepV += pitchV[i];
	}


	NvMediaVideoSurfaceUnlock(capSurf);

	return 1;
}

static unsigned int CaptureThread(void *params)
{
	int i = 0;
	NvU64 stime, ctime, mytime;
	NvMediaTime t1 = { 0 }, t2 = { 0 }, st = { 0 }, ct = { 0 };
	CaptureContext *ctx = (CaptureContext *)params;
	NvMediaVideoSurface *releaseList[4] = { NULL }, **relList;
	NvMediaRect primarySrcRect;
	NvMediaPrimaryVideo primaryVideo;

	primarySrcRect.x0 = 0;
	primarySrcRect.y0 = 0;
	primarySrcRect.x1 = ctx->inputWidth;
	primarySrcRect.y1 = ctx->inputHeight;

	primaryVideo.next = NULL;
	primaryVideo.previous = NULL;
	primaryVideo.previous2 = NULL;
	primaryVideo.srcRect = &primarySrcRect;
	primaryVideo.dstRect = NULL;


	NvSemaphoreDecrement(ctx->semStart, NV_TIMEOUT_INFINITE);

	if (ctx->timeNotCount)
	{
		GetTime(&t1);
		AddTime(&t1, ctx->last * 1000000LL, &t1);
		GetTime(&t2);
		printf("timeNotCount\n");
	}
	GetTime(&st);
	stime = (NvU64)st.tv_sec * 1000000000LL + (NvU64)st.tv_nsec;
	mytime = stime;

	while ((ctx->timeNotCount ? (SubTime(&t1, &t2)) : ((unsigned int)i < ctx->last)) && !stop)
	{
		GetTime(&ct);
		ctime = (NvU64)ct.tv_sec * 1000000000LL + (NvU64)ct.tv_nsec;
		
		if (reset_Stimer)
		{
			stime = ctime;
			reset_Stimer = 0;
		}

		if (reset_Mtimer)
		{
			mytime = ctime;
			reset_Mtimer = 0;
		}
		current_time = ((ctime - stime) / 100000000LL);
		my_timer = ((ctime - mytime) / 100000000LL); 

//		printf(" - - - Current Time : %d.%d My Time : %d.%d \n", (int)current_time / 10, (int)current_time % 10, (int)my_timer / 10, (int)my_timer % 10);
		//printf("frame=%3d, time=%llu.%09llu[s] \n", i, (ctime - stime) / 1000000000LL, (ctime - stime) % 1000000000LL);

		pthread_mutex_lock(&mutex);            // for ControlThread()

		if (!(capSurf = NvMediaVideoCaptureGetFrame(ctx->capture, ctx->timeout)))
		{ // TBD
			MESSAGE_PRINTF("NvMediaVideoCaptureGetFrame() failed in %sThread\n", ctx->name);
			stop = NVMEDIA_TRUE;
			break;
		}

		if (i % 2 == 0) // 3                       // once in three loop = 10 Hz
			pthread_cond_signal(&cond);        // ControlThread() is called	//////////////////////////////////////////////////////////////////////

		pthread_mutex_unlock(&mutex);        // for ControlThread()

		primaryVideo.current = capSurf;
		primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_TOP_FIELD;

		if (NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
														 NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
														 NULL, // background
														 &primaryVideo, // primaryVideo
														 NULL, // secondaryVideo
														 NULL, // graphics0
														 NULL, // graphics1
														 releaseList, // releaseList
														 NULL)) // timeStamp
		{ // TBD
			MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the top field in %sThread\n", ctx->name);
			stop = NVMEDIA_TRUE;
		}

		primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_BOTTOM_FIELD;
		if (NVMEDIA_STATUS_OK != NvMediaVideoMixerRender(ctx->mixer, // mixer
														 NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
														 NULL, // background
														 &primaryVideo, // primaryVideo
														 NULL, // secondaryVideo
														 NULL, // graphics0
														 NULL, // graphics1
														 releaseList, // releaseList
														 NULL)) // timeStamp
		{ // TBD
			MESSAGE_PRINTF("NvMediaVideoMixerRender() failed for the bottom field in %sThread\n", ctx->name);
			stop = NVMEDIA_TRUE;
		}

		if (ctx->fileDumpEnabled)
		{
			if (!DumpFrame(ctx->fout, capSurf))
			{ // TBD
				MESSAGE_PRINTF("DumpFrame() failed in %sThread\n", ctx->name);
				stop = NVMEDIA_TRUE;
			}

			if (!ctx->displayEnabled)
				releaseList[0] = capSurf;
		}

		relList = &releaseList[0];

		while (*relList)
		{
			if (NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
			{ // TBD
				MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);
				stop = NVMEDIA_TRUE;
				break;
			}
			relList++;
		}

		if (ctx->timeNotCount)
			GetTime(&t2);

		i++;
	} // while end

	  // Release any left-over frames
	  //    if(ctx->displayEnabled && capSurf && capSurf->type != NvMediaSurfaceType_YV16x2) // To allow returning frames after breaking out of the while loop in case of error
	if (ctx->displayEnabled && capSurf)
	{
		NvMediaVideoMixerRender(ctx->mixer, // mixer
								NVMEDIA_OUTPUT_DEVICE_0, // outputDeviceMask
								NULL, // background
								NULL, // primaryVideo
								NULL, // secondaryVideo
								NULL, // graphics0
								NULL, // graphics1
								releaseList, // releaseList
								NULL); // timeStamp

		relList = &releaseList[0];

		while (*relList)
		{
			if (NvMediaVideoCaptureReturnFrame(ctx->capture, *relList) != NVMEDIA_STATUS_OK)
				MESSAGE_PRINTF("NvMediaVideoCaptureReturnFrame() failed in %sThread\n", ctx->name);

			relList++;
		}
	}

	NvSemaphoreIncrement(ctx->semDone);
	return 0;
}

static void CheckDisplayDevice(NvMediaVideoOutputDevice deviceType, NvMediaBool *enabled, unsigned int *displayId)
{
	int outputDevices;
	NvMediaVideoOutputDeviceParams *outputParams;
	int i;

	// By default set it as not enabled (initialized)
	*enabled = NVMEDIA_FALSE;
	*displayId = 0;

	// Get the number of devices
	if (NvMediaVideoOutputDevicesQuery(&outputDevices, NULL) != NVMEDIA_STATUS_OK) {
		return;
	}

	// Allocate memory for information for all devices
	outputParams = malloc(outputDevices * sizeof(NvMediaVideoOutputDeviceParams));
	if (!outputParams) {
		return;
	}

	// Get device information for acll devices
	if (NvMediaVideoOutputDevicesQuery(&outputDevices, outputParams) != NVMEDIA_STATUS_OK) {
		free(outputParams);
		return;
	}

	// Find desired device
	for (i = 0; i < outputDevices; i++) {
		if ((outputParams + i)->outputDevice == deviceType) {
			// Return information
			*enabled = (outputParams + i)->enabled;
			*displayId = (outputParams + i)->displayId;
			break;
		}
	}

	// Free information memory
	free(outputParams);
}



/* Vehicle Control thread */
void *ControlThread(void *unused)
{
	int i = 0;

//#ifdef IWANNACAPTURE_ORG
	char fileName[30];
//#endif

#ifdef IWANNATIMECHECK
	NvMediaTime pt1 = { 0 }, pt2 = { 0 };
	NvU64 ptime1, ptime2;
	struct timespec;
#endif

	IplImage *imgOrigin, *imgOrigin_HSV, *imgLaneBinarized;
	imgOrigin = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 
	imgOrigin_HSV = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// HSV 
	imgLaneBinarized = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 1);	// Grayscale
	DisplayResult(imgLaneBinarized, 101);	// 404 Not found


	IplImage *imgLeft, *imgFront, *imgRight, *imgBase, *imgBase2, *imgDiff;

	imgLeft = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 
	imgFront = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 
	imgRight = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3); // BGR 
	imgBase = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 
	imgBase2 = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 
	imgDiff = cvCreateImage(cvSize(RESIZE_WIDTH, RESIZE_HEIGHT), IPL_DEPTH_8U, 3);	// BGR 

	sleep(2);
	start_state = 1;	// Auto Start

	int RA = 0;
	int RAset = 0;
	int diff = 0;
	int l, f ,r;
	int tRCount = 0, tLCount = 0;
	volatile int dodgeD = 0;
	volatile char TL_result = 0;
	tPoint.x = 160;

	_Bool en_SetDirection;	// Direction control enable flag
	unsigned char current_sequence = 0;

	//CameraYServoControl_Write(1500);
	//CameraXServoControl_Write(1200);

	while (1)
	{
		pthread_mutex_lock(&mutex);	// lock the thread mutex to get the vedio frame from camera
		pthread_cond_wait(&cond, &mutex);

#ifdef IWANNATIMECHECK
		GetTime(&pt1);
		ptime1 = (NvU64)pt1.tv_sec * 1000000000LL + (NvU64)pt1.tv_nsec;
#endif
		Frame2Ipl(imgOrigin); // save image to IplImage structure & resize image from 720x480 to 320x240
		/* Resize the original camera image and get the resized frame */

		pthread_mutex_unlock(&mutex);	// unlock the thread mutex

#ifdef IWANNACAPTURE_ORG
		/*	Save the origin Image for processing  */
		sprintf(fileName, "captureImage/1113/imgOrg_%d.png", i);
		cvSaveImage(fileName, imgOrigin, 0);				
#endif

		/* Image Processing Algorithm */
		// TODO : Image Processing based on captured image ---------------------
		if (!start_state) 
		{
			cvCvtColor(imgOrigin, imgOrigin_HSV, CV_BGR2HSV);	// convert BGR -> HSV
			StartSignalDetect(imgOrigin_HSV);			// Find the Start signal
		}
		else 
		{
			cvCvtColor(imgOrigin, imgOrigin_HSV, CV_BGR2HSV);
			// convert BGR -> HSV
			cvLaneBinariztionAndSignDetect(imgOrigin_HSV, imgLaneBinarized);	
			// Lane Binariaztion		// Check This First! if This works well then 
			cvDetectLaneVPoint(imgOrigin,imgLaneBinarized, &tPoint);		
			// Extract target point		// Simulate This! In Window Debugging Service

#ifdef IWANNACAPTURE_BNY
			/*	Save the Processed Image for processing  */
			sprintf(fileName, "captureImage/1113/imgBiz_%d.png", i);
			cvSaveImage(fileName, imgLaneBinarized, 0);
#endif
#ifdef IWANNACAPTURE_PRC
			sprintf(fileName, "captureImage/1113/imgPro_%d.png", i);
			cvSaveImage(fileName, imgOrigin, 0);
#endif
		}
		// ---------------------------------------------------------------------


#ifdef JUST_TEST
//----------------------------------------------------------------------------

#ifdef DRIVING_TEST	// JUST DRIVING TEST
		setVehicleSpeed(100);
		//setVehicleDirection(-100);
		directionControl(tPoint.x);
#endif


#else

		/* Contest Load Sequence Check */
		// TODO : update the driving sequence only for Contest ---------------
		if (start_state)
		{
			switch(sequence)
			{
			case 0:
				if(start_state) sequence = 1;
				break;
			case 1:
				if(Misson_HillDrive_Done) sequence = 8; 
				break;
			case 2:
				if(Misson_Parkikng_Done) sequence = 3; 
				break;
			case 3:
				if(Misson_RotaryDrive_Done) sequence = 4; 
				break;
			case 4:
				if(Misson_LineChange_Done) sequence = 5; 
				break;
			case 5:
				if(Misson_TrafficLight_Done) sequence = 7; //sequence = 6;
				break;
			case 6:
				if(Misson_ENDLine_Done) sequence = 7;
				break;
			case 7:
				break;
			case 8: 
				if(Misson_StopSign_Done) sequence = 2;
				break;			
			default: 

				printf("Worng sequence!");

				break;
			}
		}

		/* Vehicle Control Algorithm */
		// TODO : control steering angle based on captured image -----------------------------------------

		if (start_state)
		{
			en_SetDirection = 0;

			if (sequence == 8 && (StopSign_detect_flag || stopSign_Cnt > 0) && !Misson_StopSign_Done) // When StopSign detected!
			{
				printf("StopSign detected!\n");
				if(!vehicle_stop_state) vehicleStop();
				reset_Mtimer = 1;
				stopSign_Cnt--;
				if(stopSign_Cnt == 0) Misson_StopSign_Done = 1;
			}


			else if (sequence == 2 && ReadyToParking_flag) 
			{
				if (VerticalParking_detect_flag && !JustOnce_VP_flag)	// When Vertical Parking State detected!
				{
					if (VerticalParking_detect_flag == 1)
					{
						parking_Vertical(1);
					}
					else if (VerticalParking_detect_flag == 2)
					{
						parking_Vertical(2);
					}
					JustOnce_VP_flag = NVMEDIA_TRUE;
				}
				else if (ParallelParking_detect_flag && !JustOnce_PP_flag)	// When Parallel Parking State detected!
				{
					if (ParallelParking_detect_flag == 1)
					{
						parking_Parallel(1);
					}
					else if (ParallelParking_detect_flag == 2)
					{
						parking_Parallel(2);
					}
					JustOnce_PP_flag = NVMEDIA_TRUE;
				}
				if(JustOnce_VP_flag && JustOnce_PP_flag) Misson_Parkikng_Done = 1;
				ReadyToParking_flag = 0;
				reset_sequence();
				reset_Mtimer = 1;

				setVehicleSpeed(50);
			}


			else if (sequence == 3 && StopLine_detect_flag && !rotary_entered_state)	
			{
				if (RA % 2 == 0) cvCvtColor(imgOrigin, imgBase, CV_RGB2GRAY);
				else cvCvtColor(imgOrigin, imgBase2, CV_RGB2GRAY);
	
				cvAbsDiff(imgBase, imgBase2, imgDiff);
	
					diff = checkDiff(imgDiff);
		
					if (sequence == 5 && !Misson_TrafficLight_Done) TL_result = checkTL(imgOrigin_HSV);
		
					en_SetDirection = 0;
		
					if (!vehicle_stop_state){ 
						printf("stoped.\n");				
						vehicleStop(); // until car passed
					}				
					if (diff > 800) {
						RAset++;
						printf("(over 800) diff is : %d ",diff);
					}else {
						printf("RAset was : %d \n " , RAset);
						RAset = 0;
					}

					printf("below 1000 diff is : %d ",diff);
					printf("   RASET is : %d  \n",RAset);
					if(RAset>12) //start after moving object detacted for 10++ frames.
					{
						//CameraYServoControl(down)
						setVehicleSpeed(50);
						rotary_entered_state = 150;
					}
					
					
					RA++;
			}


			else if (sequence == 4 && obs_detect_state[0] && !Misson_LineChange_Done) 
			{
				if (!vehicle_stop_state) vehicleStop();

				switch (dodgeD)
				{
				case 0:
					CameraXServoControl_Write(1800);
					sleep(1);
					dodgeD++;
					break;
				case 1:
					imgLeft = imgOrigin;
					dodgeD++;
					break;
				case 2:
					CameraXServoControl_Write(1500);
					sleep(1);
					dodgeD++;
					break;
				case 3:
					imgFront = imgOrigin;
					dodgeD++;
					break;
				case 4:
					CameraXServoControl_Write(1200);
					sleep(1);
					dodgeD++;
					break;
				case 5:
					imgRight = imgOrigin;
					dodgeD++;
					break;
				case 6:
					CameraXServoControl_Write(1500);
					l = countlW(imgLeft,1);
					f = countlW(imgFront,2);
					r = countlW(imgRight,3);
					sleep(1);
					if (l < f && l < r) 
					{
						printf("to left");
						positionControl(-50, 20);
						setVehicleDirection_Value(1950);		
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1100);
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1520);
						sleep(1);
						positionControl(100, 20);
						sleep(1);
						setVehicleDirection_Value(1100);
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1950);
						sleep(1);
						positionControl(100, 40);
						sleep(50);
					}
					else if (f < l && f < r) 
					{
						printf("to front");
						setVehicleDirection_Value(1520);
						sleep(1);
						positionControl(100, 80);
					}
					else 
					{
						printf("to right");
						positionControl(-50, 20);
						setVehicleDirection_Value(1100);
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1950);
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1520);
						sleep(1);
						positionControl(100, 20);
						sleep(1);
						setVehicleDirection_Value(1950);
						sleep(1);
						positionControl(100, 40);
						setVehicleDirection_Value(1080);
						sleep(1);
						positionControl(100, 40);
						sleep(50);
					}
					Misson_LineChange_Done = 1;
					printf(" l, f, r  : %d %d %d \n " ,l,f,r);
					sleep(50);
					break;
				default: break;
				}	
				}


			else if (sequence == 5 && StopLine_detect_flag && !Misson_TrafficLight_Done)	
			{
		


				TL_result = checkTL(imgOrigin_HSV);
				switch(TL_result)
				{
		case 1: 
				printf("RED LIGHT DETECTED!");
			break;
		case 2:
			printf("YELLOW LIGHT DETECTED!");
			break;
		case 3:
			printf("GREEN LIGHT (RIGHT) DETECTED!"); 
			tRCount=0;
			tLCount++;
			sleep(1);
			break;
		case 4:
			printf("GREEN LIGHT (LEFT) DETECTED!"); 
			tLCount = 0;
			tRCount++;
			sleep(1);
			break;
		default:
			printf("Can't find trafficlight state!"); 
			break;
		}
		
		if (tLCount >= 3) {
			setVehicleDirection_Value(1800);
			positionControl(100,40);
			printf("toLeft \n");
			sleep(50);
		}
		if (tRCount >= 3) {
			setVehicleDirection_Value(1200);
			positionControl(100,40);
			printf("toRight \n");
			sleep(50);
		}
			}


			else if (sequence == 6 && !Misson_ENDLine_Done) 
			{
				;
			}


			else if (sequence == 7) 
			{
				//vehicleStop();
			}


			else
			{
				en_SetDirection = 1;
				speedControl(tPoint.x, tPoint.y);
			}

			if(en_SetDirection) directionControl(tPoint.x);

			if(current_sequence != sequence)
			{
				printf("Current Sequence : %d \n", sequence);
				current_sequence = sequence;
			}
		}
		// ---------------------------------------------------------------------
#endif

#ifdef IWANNATIMECHECK
		GetTime(&pt2);
		ptime2 = (NvU64)pt2.tv_sec * 1000000000LL + (NvU64)pt2.tv_nsec;
		printf("--------------------ControlThread operation time=%llu.%09llu[s]\n", (ptime2 - ptime1) / 1000000000LL, (ptime2 - ptime1) % 1000000000LL);
#endif
		i++;	// frame count
	}
}
	

/* Sensor state update thread */
void *SensorDetectThread(void *unused)
{

#ifdef IWANNATIMECHECK2
	NvMediaTime pt1 = { 0 }, pt2 = { 0 };
	NvU64 ptime1, ptime2;
	struct timespec;
#endif

	while (1)
	{



#ifdef IWANNATIMECHECK2
		GetTime(&pt1);
		ptime1 = (NvU64)pt1.tv_sec * 1000000000LL + (NvU64)pt1.tv_nsec;
#endif

		/* Sensor Detect Sequence */
		// TODO : Load Sensor Detected Value-----------------------------------
		if (start_state)
		{
			if(pthread_mutex_lock(&sensor_mutex)) perror("lock");	// lock the thread mutex to get the Sensor data
			// To Do : Sensor & Situration Check ----------------------------------

#ifdef JUST_TEST
			updateDistanceState();
			if((!JustOnce_VP_flag || !JustOnce_PP_flag) && my_timer > 20 && !ReadyToParking_flag && sequence == 2) 
				update_ParkingSignalSequenceState();
			parking_Distance_Check();

#else
			// Sensor state update
			updateDistanceState();
			update_LineState();

			// for Parking
			if((!JustOnce_VP_flag || !JustOnce_PP_flag) && my_timer > 20 && !ReadyToParking_flag && sequence == 2) 
				update_ParkingSignalSequenceState();

			parking_Distance_Check();

			// for Obstacle Check
			if(sequence == 1 || sequence == 3 || sequence == 4) Obstacle_Checking();

//			printf("\nSensor 1 : %d\t 4 : %d\t 2 : %d\t 3: %d\t 6 : %d\t 5 : %d\n\n",MeasuredDistance[0],MeasuredDistance[3], MeasuredDistance[1], MeasuredDistance[2], MeasuredDistance[4], MeasuredDistance[5]);
#endif

			//---------------------------------------------------------------------
			if(pthread_mutex_unlock(&sensor_mutex)) perror("unlock");	// unlock the thread sensor mutex
		}
		// ---------------------------------------------------------------------

#ifdef IWANNATIMECHECK2
		GetTime(&pt2);
		ptime2 = (NvU64)pt2.tv_sec * 1000000000LL + (NvU64)pt2.tv_nsec;
		printf("--------------------SensorDetectThread operation time=%llu.%09llu[s]\n", (ptime2 - ptime1) / 1000000000LL, (ptime2 - ptime1) % 1000000000LL);
#endif

	}
}

//NVMEDIA_FALSE
//NVMEDIA_TRUE
