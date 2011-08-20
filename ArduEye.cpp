/*
  ArduEye.cpp - Library for interfacing with the ArduEye Sensor
  Centeye, Inc
  Created by Alison Leonard. August, 2011
  License Info here
*/

#include <ArduEye.h>
#include <SPI.h>

ArduEye::ArduEye()
{
	_SerialTx = true;
	_SerialMonitorMode = false;
}
// Initialize Ardue Eye Interface
//
void ArduEye::begin(int RdyPin, int CSPin)
{
	_dataReadyPin = RdyPin;
	_chipSelectPin = CSPin;
	
	Serial.begin(115200);
	
	// start the SPI library:
	  SPI.setClockDivider(SPI_CLOCK_DIV4);
	  SPI.setDataMode(SPI_MODE3);
	  SPI.setBitOrder(MSBFIRST);
	  SPI.begin();

  // initalize the  data ready and chip select pins:
	  pinMode(_dataReadyPin, INPUT);
	  pinMode(_chipSelectPin, OUTPUT);
	  digitalWrite(_chipSelectPin, HIGH);
	  
	  // initialize data set record
	  _DS[0].DSID = ARDUEYE_ID_RAW;
	  _DS[0].Array = RawImage;
	  _DS[0].MaxSize = ARDUEYE_RAW_SIZE;
	  
	  _DS[1].DSID = ARDUEYE_ID_OFX;
	  _DS[1].Array = OpticFlowX;
	  _DS[1].MaxSize = ARDUEYE_OF_SIZE;
	  
	  _DS[2].DSID = ARDUEYE_ID_OFY;
	  _DS[2].Array = OpticFlowY;
	  _DS[2].MaxSize = ARDUEYE_OF_SIZE;
	  
	  _DS[3].DSID = ARDUEYE_ID_FPS;
	  _DS[3].Array = _FPS;
	  _DS[3].MaxSize = ARDUEYE_FPS_SIZE;
	 	
}

// Start streaming a new dataset from the ArduEye
// multiple datasets can be active at the same time
void ArduEye::startDataStream(int DataSet)
{
  int i;  
  char StartCommand[5];
  //StartCommand[0] = ESC_CHAR; // escape char
  StartCommand[0] = SOC_CHAR; // start command byte
  StartCommand[1] = 2; // cmd size
  StartCommand[2] = DISPLAY_CMD; // start command
  StartCommand[3] = DataSet; // data id 
  
  // update DSRecord
  for (i = 0; i < MAX_DATASETS; i++)
  {   
    if(_DS[i].DSID == DataSet)
    {
      _DS[i].Active = true;
      break;
    }
  }
 
  // send start command to ArduEye
  digitalWrite(_chipSelectPin, LOW);
  for (i = 0; i < 4; i++)
  	SPI.transfer(StartCommand[i]);
  digitalWrite(_chipSelectPin, HIGH);
  	
}

// Stop streaming a dataset from the ArduEye
// Any other active datasets will continue to stream
void ArduEye::stopDataStream(int DataSet)
{  
  int i;  
  char StartCommand[5];
//  StartCommand[0] = ESC_CHAR; // escape char
  StartCommand[0] = SOC_CHAR; // start command byte
  StartCommand[1] = 2; // cmd size
  StartCommand[2] = STOP_CMD; // stop command
  StartCommand[3] = DataSet; // data id
  
  // update DSRecord
  for (i = 0; i < MAX_DATASETS; i++)
  {   
    if(_DS[i].DSID == DataSet)
    {
      _DS[i].Active = false;
      break;
    }
  }
  
  // send stop command to ArduEye
  digitalWrite(_chipSelectPin, LOW);
  for (i = 0; i < 4; i++)
  	SPI.transfer(StartCommand[i]);
  digitalWrite(_chipSelectPin, HIGH);
}
// get data from active datasets
// this command can be run each loop to update
// dataRdy() must be checked before running getData()
void ArduEye::getData()
{
	int i, k, InSize, Rows, Cols, c, Idx;
	boolean EODS = false;
	unsigned char Header[FULL_HEAD_SIZE];
   //Header: ESC, 2bytes pckt size, 1 byte DataId, 2 byte rows, 2 byte cols , 2Bytes dataIndex, 1 Byte EODS
  
  // loop through active datasets
  for (k = 0; k < MAX_DATASETS; k++)
  {
    if(_DS[k].Active)
    {
      // get data until End of Dataset flag received
      while(EODS == false)
      {
      	// read data packet header
        digitalWrite(_chipSelectPin, LOW);
        delayMicroseconds(1);
        SPI.transfer(SOH_CHAR);
        delayMicroseconds(1);
        for (i = 0; i < FULL_HEAD_SIZE; i++)
          Header[i] = SPI.transfer(0x00);
        digitalWrite(_chipSelectPin, HIGH);
        
        // assign relevant header data       
        InSize = (Header[1] << 8) + Header[2] - HEAD_DAT_SIZE;
        Rows = (Header[4] << 8) + Header[5];
        Cols = (Header[6] << 8) + Header[7];
        EODS = Header[10];

		// write header data to serial monitor if active
		if(_SerialTx && _SerialMonitorMode)
		{    
            
	        Serial.println(Header[0], DEC);
            Serial.println(Header[1],DEC);
            Serial.println(Header[2],DEC);
            Serial.println(InSize, DEC);
            Serial.println(Header[3], DEC);
            Serial.println(Rows, DEC);
            Serial.println(Cols, DEC);
            Serial.println((Header[8] << 8) + Header[9], DEC);
            Serial.println(EODS, DEC);
            Serial.println(" ");
		}
		// update cols for serial monitor display if necessary
		//if(Rows * Cols > InSize)
		//	Cols = InSize  / Rows;
        
        // abort read if size data is incorrect
        if((InSize <= 0) || (InSize > _DS[k].MaxSize))
           break;
          
        // read data packet 
         digitalWrite(_chipSelectPin, LOW);
         SPI.transfer(SOD_CHAR); 
         delayMicroseconds(1);
         for(i = 0; i < InSize; i++)
           _DS[k].Array[i] = SPI.transfer(0x00);
          digitalWrite(_chipSelectPin, HIGH);

		if (_SerialTx)
		{   	
			if(!_SerialMonitorMode)
			{  
		        for (i = 0; i < FULL_HEAD_SIZE; i++)
		          Serial.print(Header[i]);
            }  
		    for (i = 0; i < InSize; i++)
                Serial.print(_DS[k].Array[i]);
            if(_SerialMonitorMode)
                Serial.println(" ");
		     
		} 
      }
    }
  } 
}

// check data ready pin to see if ArduEye Data is ready
// dataRdy() must be called before getData()
boolean ArduEye::dataRdy()
{
	return (digitalRead(_dataReadyPin) == HIGH);
}

// send calibrate command to ArduEye
// calibrate generates a new Fixed Pattern noise mask for the vison chip
void ArduEye::calibrate()
{
	sendCommand(CMD_CALIBRATE, 0, 0);
}
// change raw image resolution
//
void ArduEye::setResolution(int rows, int cols)
{
	char Cmd[2] = {rows,cols};
	sendCommand(CMD_RESOLUTION, Cmd, 2);
}
// change optic flow array resolution - raw image resolution will
// not be changed, this command sets the number of optic flow 
// computation regions in the image
void ArduEye::setOFResolution(int rows, int cols)
{
	char Cmd[2] = {rows,cols};
	sendCommand(CMD_OF_RESOLUTION, Cmd, 2);
}
// change the smoothing rate for the optic flow calculation
// depending on the sensor read speed, optic flow smoothing
// can provide a less noisy result
void ArduEye::setOFSmoothing(float level)
{
	char Cmd[1];
	Cmd[0] = 100/level;
	sendCommand(CMD_OF_SMOOTHING, Cmd, 1);
}
// send a user defined command to the ArduEye
//
void ArduEye::sendCommand(char Cmd, char * Value, int Size)
{
	int i;
	char StartCommand[4];
//	StartCommand[0] = ESC_CHAR; // escape char
	StartCommand[0] = SOC_CHAR; // start command byte
	StartCommand[1] = Size+1; // cmd size
	StartCommand[2] = Cmd; // stop command
	digitalWrite(_chipSelectPin, LOW);
	for (i = 0; i < 3; i++)
		SPI.transfer(StartCommand[i]);
	for (i = 0; i < Size; i++)
	  	SPI.transfer(Value[i]);
	digitalWrite(_chipSelectPin, HIGH);
	
}

// retrieve FPS value from dataset
//
short ArduEye::FPS()
{
	  short fps;
	  for (int i = 0; i < MAX_DATASETS; i++)
	  {   
	    if(_DS[i].DSID == ARDUEYE_ID_FPS)
	    {
			fps = (_DS[i].Array[0] << 8) + _DS[i].Array[1];
			return fps;
		}	
	  }
}

// check for communication from UI and pass commands on to ArduEye
//
void ArduEye::checkUIData()
{
	int i, Idx, CmdBytes;
	int BytesReceived = Serial.available();
	char InByte[MAX_IN_SERIAL];
  
	if(BytesReceived)
	{
		delay(50);
		BytesReceived = Serial.available();
		for (i = 0; i < BytesReceived; i++)
	    	InByte[i] = Serial.read();
        
	  Idx = 0;
	  while(Idx < BytesReceived)
	  {
	    // look for ESC char, signal for start of command  
	    while((InByte[Idx] != ESC_CHAR) && (Idx < BytesReceived))
	      Idx++;
	    
	    if(Idx >= BytesReceived-1)
	      return;
	
		if(_SerialMonitorMode)   
		{
		  sscanf((const char *)InByte+Idx+1, "%x", &CmdBytes);
		  InByte[Idx + 1] = CmdBytes;
		}
		else
	    	CmdBytes = InByte[Idx+1];
	    CmdBytes +=2; 
	       
	    switch(InByte[Idx+2])
	    {
	      case DISPLAY_CMD:
	        startDataStream(InByte[Idx+3]);
	        break;
	      case STOP_CMD:
	        stopDataStream(InByte[Idx+3]);
	        break;
	      case WRITE_CMD:
	          digitalWrite(_chipSelectPin, LOW);
	          for(i = 0; i < CmdBytes; i++)
	             SPI.transfer(InByte[Idx + i]);
	          digitalWrite(_chipSelectPin, HIGH);
	      
			  if(_SerialMonitorMode)  
			  {          
	            for(i = 0; i < CmdBytes; i++)
	              Serial.print(InByte[Idx + i]);
	            Serial.println("cmd sent");
			  }
	          break;
	       default:
	          break; 
	    }
	    Idx += CmdBytes;
	  }
	}	
}	
// Enable Serial Communication.  Enabled by default
// When enabled, all data sets will be sent out over serial to either the 
// ArduEye UI or the Serial Monitor
void ArduEye::enableSerialTx(boolean Enable)
{
	_SerialTx = Enable;
}
// SerialMonitorMode is disabled by default.  When enabled
// serial data will be formated for display on the serial monitor
// SerialTx must be enabled for SerialMonitorMode to display data 
void ArduEye::setSerialMonitorMode(boolean Enable)
{
	_SerialMonitorMode = Enable;
}
	

