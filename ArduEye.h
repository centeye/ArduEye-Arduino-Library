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
#define CMD_ACK 37

#define MAX_SPI_PCKT_SIZE   512
#define MAX_IN_SERIAL	40
#define MAX_CMD_SIZE    10

// start packet flags
#define ESC_CHAR	  38 //0xFF
#define EOD_CHAR      0xFC
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
#define DISPLAY_DUMP 5
#define DISPLAY_POINTS 6

#define NULL_DS       -1


typedef struct DSRecord{
  
  boolean Active;
  int DSID;
  int DisplayType;
  
  DSRecord()
  {
    Active = false;
    DSID = NULL_DS;
    DisplayType = DISPLAY_NONE;
  }
} DSRecord;


class ArduEye{

public:
	ArduEye();
    // intialization function : must be called in the setup loop to start the ArduEye library
	void begin(int RdyPin, int CSPin);
    
    ///////// ArduEye Data Acquire Functions //////////////////////////
    
    // Data Streaming Fuctions for UI interface
	void startDataStream(char DataSet);
	void stopDataStream(char DataSet);
	void getData();
    
    // Embedded Data Set acquire functions 
    // (either data streaming or embedded dataset methods can be used
    // with the UI, however, embedded data set methods allow the user to
    // process the received data sets on the arduino instead of just passing
    // data directly to the UI)
    // Buf has a maximum size of MAX_SPI_PCKT_SIZE.  If the dataset is larger than this, Buf will contain a partial datse
	void getDataSet(char DataSet, char *Buf);
    // when used embedded dataset acquire, endFrame must be called each loop after all datasets have been read
    // endFrame alerts the ArduEye that data read is finished, and alerts the serial UI (if active)
    void endFrame();

	////////// ArduEye settings /////////////////////////
    
    // generate a new fixed pattern noise mask for all resolution levels
	void calibrate();
    // set resolution of rawImage (Valid options are sensor dependent, see .h file)
	void setResolution(int rows, int cols);
    // set OpticFlow Resolution (Valid options are sensor dependent, see .h file)
	void setOFResolution(int rows, int cols);
    // generic send command fucntion.  Size paramater is the length of the Value Array
	void sendCommand(char Cmd, char * Value, int Size);
    
    // Set Display Type associate with a particular dataset.  This type will be used in Tx to the UI
    // The UI reads the Display Type variable to know how to display data
    void setDisplayType(int DSID, int DisplayType);
    // find display type setting for a given dataset
    int getDisplayType(char DataSet);
    
    ///////// Communications Functions ///////////////
    
    // check if data is ready on the ArduEye
    boolean dataRdy();
    // check that sensor is booted and ready to receive commands
    boolean sensorRdy();

	
    // check if serial data has been received from the UI
	bool checkUIData();
	
    // turn serial transmit on or off. Serial Tx is off by default
	void enableSerialTx(boolean Enable);
    // turn serial monitor on or off, if on, serial data will be formated to be displayed
    // on the serial monitor instead of the Qt UI.  Serial Monitor mode is used primarily for debugging.
	void setSerialMonitorMode(boolean Enable);
	
    // debug variables 
    bool Toggle, Toggle2;	
    
private:
    
    // check is serial buffer is clear and OK to send data
    boolean checkBufferFull();
    // parse cmd received from the UI and send to ArduEye
    void parseCmd(int StartIdx, int EndIdx);
    
    
    char _ReceiveBuffer[MAX_SPI_PCKT_SIZE];
	int _dataReadyPin;
	int _chipSelectPin;
	DSRecord _DS[MAX_DATASETS];
    char _ActiveSets[MAX_DATASETS];
    int _NumActiveSets;
	boolean _SerialTx;
	boolean _SerialMonitorMode;
    char _InBuffer[MAX_IN_SERIAL];
    int _InSIdx, _ESCReceived, _InBufIdx, _BufEnd;
		
};

#endif