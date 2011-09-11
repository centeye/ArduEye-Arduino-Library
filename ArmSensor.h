// ArmSensor.h

#ifndef ARM_SENSOR_H
#define ARM_SENSOR_H

// Comm Sizes
#define HEAD_DAT_SIZE 5 
#define FULL_HEAD_SIZE 5 

#define MAX_PACKET_SIZE 512
#define MAX_DATASETS 6

//max data set sizes
#define ARDUEYE_RAW_SIZE 512
#define ARDUEYE_OF_SIZE	64
#define ARDUEYE_FPS_SIZE 2
#define ARDUEYE_VAL_SIZE 2
#define ARDUEYE_MAXES_SIZE 84

// data set ids
#define ARDUEYE_ID_RAW 48
#define ARDUEYE_ID_OFX 50
#define ARDUEYE_ID_OFY 52
#define ARDUEYE_ID_FPS 54
#define ARDUEYE_ID_CMD 56
#define ARDUEYE_ID_MAXES 58

// ArduEye Commands
#define CMD_CALIBRATE 70
#define CMD_RESOLUTION 71
#define CMD_OF_RESOLUTION 72
#define CMD_OF_SMOOTHING 73

#endif