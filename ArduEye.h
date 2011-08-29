/*
  ArduEye.h - Library for interfacing with the ArduEye Sensor
  Centeye, Inc
  Created by Alison Leonard. August, 2011
  License Info here
*/

#ifndef ARDUEYE_H
#define ARDUEYE_H

#include <WProgram.h>
#include "ArmSensor.h"

// cmd definitions
#define WRITE_CMD  32
#define DISPLAY_CMD 33
#define STOP_CMD 35
#define ACK_CHAR 34
#define GO_CHAR 36

#define MAX_IN_SERIAL	40

// start packet flags
#define ESC_CHAR	  38 //0xFF
#define EOD_CHAR 0xFC
#define SOD_CHAR      0xFE
#define SOH_CHAR      0xFD
#define NULL_CHAR     0

// Comm Commands
#define START_PCKT 90
#define END_PCKT 91
#define END_FRAME 92
#define WRITE_CHAR 93
#define READ_CHAR 94

// Display Commands
#define DISPLAY_NONE  0
#define DISPLAY_GRAYSCALE_IMAGE 1
#define DISPLAY_CHARTX 2
#define DISPLAY_CHARTY 3
#define DISPLAY_TEXT 4

#define NULL_DS       -1


typedef struct DSRecord{
  
  boolean Active;
  int DSID, MaxSize;
  char * Array;
  int DisplayType;
  
  DSRecord()
  {
    Active = false;
    DSID = NULL_DS;
    Array = 0;
    MaxSize = 0;
    DisplayType = DISPLAY_NONE;
  }
} DSRecord;


class ArduEye{

public:
	ArduEye();
	void begin(int RdyPin, int CSPin);
	
	void startDataStream(char DataSet);
	void stopDataStream(char DataSet);
	void getData();
	
	boolean dataRdy();
    boolean CheckBufferFull();
	
	void calibrate();
	void setResolution(int rows, int cols);
	void setOFResolution(int rows, int cols);
	void setOFSmoothing(float level);
	void sendCommand(char Cmd, char * Value, int Size);
    
    void SetDisplayType(int DSID, int DisplayType);
	
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
    char _ActiveSets[MAX_DATASETS];
    int _NumActiveSets;
	char _FPS[ARDUEYE_FPS_SIZE];
	boolean _SerialTx;
	boolean _SerialMonitorMode;
		
};

#endif