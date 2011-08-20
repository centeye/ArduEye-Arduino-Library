/*
  ArduEye.h - Library for interfacing with the ArduEye Sensor
  Centeye, Inc
  Created by Alison Leonard. August, 2011
  License Info here
*/

#ifndef ARDUEYE_H
#define ARDUEYE_H

#include <WProgram.h>

// cmd definitions
#define WRITE_CMD  32
#define DISPLAY_CMD 33
#define STOP_CMD 35
#define ACK_CMD 34

// start packet flags
#define ESC_CHAR	  38 //0xFF
#define SOC_CHAR      38
#define SOD_CHAR      0xFE
#define SOH_CHAR      0xFD
#define NULL_CHAR     0

#define NULL_DS       -1

#define HEAD_DAT_SIZE 8
#define FULL_HEAD_SIZE 11

#define MAX_PACKET_SIZE 512
#define MAX_DATASETS 4
#define MAX_IN_SERIAL	40

// max data set sizes
#define ARDUEYE_RAW_SIZE	512
#define ARDUEYE_OF_SIZE		64
#define ARDUEYE_FPS_SIZE	2

// data set ids
#define ARDUEYE_ID_RAW   48
#define ARDUEYE_ID_OFX   49
#define ARDUEYE_ID_OFY   50
#define ARDUEYE_ID_FPS   51

// ArduEye Commands
#define CMD_CALIBRATE 70  
#define CMD_RESOLUTION 71
#define CMD_OF_RESOLUTION 72
#define CMD_OF_SMOOTHING 73

typedef struct DSRecord{
  
  boolean Active;
  int DSID, MaxSize;
  char * Array;
  
  DSRecord()
  {
    Active = false;
    DSID = NULL_DS;
    Array = 0;
    MaxSize = 0;
  }
} DSRecord;


class ArduEye{

public:
	ArduEye();
	void begin(int RdyPin, int CSPin);
	
	void startDataStream(int DataSet);
	void stopDataStream(int DataSet);
	void getData();
	
	boolean dataRdy();
	
	void calibrate();
	void setResolution(int rows, int cols);
	void setOFResolution(int rows, int cols);
	void setOFSmoothing(float level);
	void sendCommand(char Cmd, char * Value, int Size);
	
	void checkUIData();
	
	void enableSerialTx(boolean Enable);
	void setSerialMonitorMode(boolean Enable);
	
	char RawImage[ARDUEYE_RAW_SIZE];
	char OpticFlowX[ARDUEYE_OF_SIZE];
	char OpticFlowY[ARDUEYE_OF_SIZE];
	short FPS();	
	
private:
	int _dataReadyPin;
	int _chipSelectPin;
	DSRecord _DS[MAX_DATASETS];
	char _FPS[ARDUEYE_FPS_SIZE];
	boolean _SerialTx;
	boolean _SerialMonitorMode;
		
};

#endif