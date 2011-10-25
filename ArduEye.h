/*
  ArduEye.h - Library for interfacing with the ArduEye Sensor
  Centeye, Inc
  Created by Alison Leonard. August, 2011
  
 ===============================================================================
 Copyright (c) 2011, Centeye, Inc.
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of Centeye, Inc. nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL CENTEYE, INC. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ===============================================================================
*/

#ifndef ARDUEYE_H
#define ARDUEYE_H

#include <WProgram.h>
#include "ArmSensor.h"

// high level cmd definitions
#define WRITE_CMD  32
#define DISPLAY_CMD 33
#define STOP_CMD 35
#define READ_CMD 40
#define SERIAL_START 39

// flow control byte definitions
#define ACK_CHAR 34
#define GO_CHAR 36
#define CMD_ACK 37

// timeout on waiting for ack in milliseconds
#define ACK_TIMEOUT 1000

// special bytes - comm packet flags (SPI & Serial)
#define ESC_CHAR	  38
#define START_PCKT 90
#define END_PCKT 91
#define END_FRAME 92

// special bytes - spi mode flags
#define WRITE_CHAR 93
#define READ_CHAR 94
#define SOD_CHAR      95
#define SOH_CHAR      96

// special bytes - NULL character	
#define NULL_CHAR -1

// max array sizes (Arduino pro mini has 1kB SRAM)
#define MAX_SPI_PCKT_SIZE   512
#define MAX_IN_SERIAL	40
#define MAX_CMD_SIZE    10

// Display Commands
#define DISPLAY_NONE  0
#define DISPLAY_GRAYSCALE_IMAGE 1
#define DISPLAY_CHARTS 2
#define DISPLAY_TEXT 4
#define DISPLAY_DUMP 5
#define DISPLAY_POINTS 6

// null flag
#define NULL_DS       -1

// DSRecord structure keeps track of dataset display types and
// active/inactive status
typedef struct DSRecord{
  
  boolean Active;
  int DSID;
  int DisplayType;
  char * name;
  
  DSRecord()
  {
    Active = false;
    DSID = NULL_DS;
    DisplayType = DISPLAY_NONE;
  }
} DSRecord;

// main ArduEye class
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
	void sendCommand(char Cmd, char * Value = 0, int Size = 0);
    
    // Set Display Type associate with a particular dataset.  This type will be used in Tx to the UI
    // The UI reads the Display Type variable to know how to display data
    void setDisplayType(int DSID, int DisplayType);
    // find display type setting for a given dataset
    int getDataIndex(char DataSet);
    
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
    //FUNCTIONS
    // check is serial buffer is clear and OK to send data
    boolean checkBufferFull();
    // parse cmd received from the UI and send to ArduEye
    void parseCmd(int StartIdx, int EndIdx);
    
    //DATA ARRAYS
    // data from spi is stored in _ReceiveBuffer
    char _ReceiveBuffer[MAX_SPI_PCKT_SIZE];
    // data from serial port is stored in _InBuffer
    char _InBuffer[MAX_IN_SERIAL];
    
    //DATASET TRACKING
	DSRecord _DS[MAX_DATASETS];
        // list of active sets
    char _ActiveSets[MAX_DATASETS];
        // number of active sets
    int _NumActiveSets;
	// Flag to process single request dataset
	int _TemporaryDataSet;
    
    //COMMUNICATIONS
    // io pins
	int _dataReadyPin;
	int _chipSelectPin;
    
    // serial comm flags
	boolean _SerialTx;
	boolean _SerialMonitorMode;
    
    // tracking variables for parsing serial input
    int _InSIdx, _ESCReceived, _InBufIdx, _BufEnd;
		
};

#endif