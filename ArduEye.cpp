/*
  ArduEye.cpp - Library for interfacing with the ArduEye Sensor
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

#include <ArduEye.h>
#include <SPI.h>
/*---------------------------------------------------
 ArduEye: Constructor
 ---------------------------------------------------*/
ArduEye::ArduEye()
{
    // set flag defaults
	_SerialTx = false;
	_SerialMonitorMode = false;
    _NumActiveSets = 0;
    _InSIdx = _InBufIdx = 0;
    _ESCReceived = false;
    _BufEnd = 0;
    Toggle = Toggle2 = false;
}

/*---------------------------------------------------
 begin : Initialize ArduEye Interface
 Input:     RdyPin: DataReady pin (default is 9 on ArduEye)
            CSPin: ChipSelect pin  (default is 10 on ArduEye)
 ---------------------------------------------------*/
void ArduEye::begin(int RdyPin, int CSPin)
{
    // initalize the  data ready and chip select pins:
	_dataReadyPin = RdyPin;
	_chipSelectPin = CSPin;
    
    pinMode(_dataReadyPin, INPUT);
    pinMode(_chipSelectPin, OUTPUT);
    digitalWrite(_chipSelectPin, HIGH);
    
    // debug pins
    pinMode(6, OUTPUT);
    digitalWrite(6,LOW);
    
    pinMode(7, OUTPUT);
    digitalWrite(7,LOW);
	
    // initialize serial link
	Serial.begin(115200);
    	
	// initialize spi link
	  SPI.setClockDivider(SPI_CLOCK_DIV8);
	  SPI.setDataMode(SPI_MODE3);
	  SPI.setBitOrder(MSBFIRST);
	  SPI.begin();

    // set default display types for datasets
	  _DS[0].DSID = ARDUEYE_ID_RAW;
      _DS[0].DisplayType = DISPLAY_GRAYSCALE_IMAGE;
    
	  _DS[1].DSID = ARDUEYE_ID_OF;
      _DS[1].DisplayType = DISPLAY_CHARTS;
	  
	  _DS[2].DSID = ARDUEYE_ID_FPS;
      _DS[2].DisplayType = DISPLAY_TEXT;
      char * name = "FPS Sensor: ";
     _DS[2].name = name;
    
      _DS[3].DSID = ARDUEYE_ID_CMD;
      _DS[3].DisplayType = DISPLAY_DUMP;
        
      _DS[4].DSID = ARDUEYE_ID_MAXES;
      _DS[4].DisplayType = DISPLAY_POINTS;
	 	
}

/*---------------------------------------------------
 startDataStream: Start streaming a new dataset from the ArduEye
 multiple datasets can be active at the same time
 datasets set to Active will be acquired by getData()
 Input:   Dataset: Any of the values defined as "Dataset IDs"
            in the Sensor header file (ie ArmSensor.h)
 ---------------------------------------------------*/
void ArduEye::startDataStream(char DataSet)
{
  int i;  
  sendCommand((char)DISPLAY_CMD, &DataSet, 1);
  
  // update DSRecord
  for (i = 0; i < MAX_DATASETS; i++)
  {   
    if(_DS[i].DSID == DataSet)
    {
        // if dataset is not already active, add to _ActiveSets list
        // and increase NumActiveSets count
        if(_DS[i].Active == false)
        {
            _ActiveSets[_NumActiveSets] = i;
            _NumActiveSets++;
        }
        _DS[i].Active = true;
      break;
    }
  }
    // Print Dataset status to serial monitor if active
    if(_SerialTx && _SerialMonitorMode)
    {
        for(i = 0; i < _NumActiveSets; i++)
            Serial.print(_ActiveSets[i], DEC);
        Serial.println(" ");
    }
}

/*---------------------------------------------------
 stopDataStream: Stop streaming a dataset from the ArduEye
 Any other active datasets will continue to stream
 Input:   Dataset: Any of the values defined as "Dataset IDs"
        in the Sensor header file (ie ArmSensor.h)
 ---------------------------------------------------*/
void ArduEye::stopDataStream(char DataSet)
{  
    int i, m;  
    sendCommand((char)STOP_CMD, &DataSet, 1);
    
      // update DSRecord
      for (i = 0; i < MAX_DATASETS; i++)
      {   
        // set active flag to false  
        if(_DS[i].DSID == DataSet)
        {
          _DS[i].Active = false;
          break;
        }
      }
  
    // remove the datset from the ActiveSets list and 
    // decrease the NumActiveSets count
    for(i = 0; i < _NumActiveSets; i++)
    {
        if(_DS[_ActiveSets[i]].DSID == DataSet)
        {
            for(m = i; m < _NumActiveSets-1; m++)
                _ActiveSets[m] = _ActiveSets[m+1];
            _NumActiveSets--;
            break;
        }
    }
    
    // print datset information to Serial Monitor if active
    if(_SerialTx && _SerialMonitorMode)
    {
        for(i = 0; i < _NumActiveSets; i++)
            Serial.print(_ActiveSets[i],DEC);
        Serial.println(" ");
    }
    
}

/*---------------------------------------------------
 checkBufferFull: check that serial buffer is not full, 
 so it is OK to send data.  When serial buffer is not full, the
 UI will send an OK_CHAR in response to receiveing a GO_CHAR
 from the ARDUINO
 returns: true if Buffer is clear and false if buffer is full
 This function is called by getData() and getDataset()
 ---------------------------------------------------*/
boolean ArduEye::checkBufferFull()
{
    int BytesReceived = 0;
    int Count = 0, CycleCount = 0;
    char inByte;
    
    // Send GO_CHAR up to 50 times if no ACK is received
    while((Count < 50) && (CycleCount < 10))
    {
        // send GO_CHAR
        Serial.print((char)ESC_CHAR);
        Serial.print((char)GO_CHAR);
        
        // wait until serial data is received
        while(BytesReceived == 0)
            BytesReceived = Serial.available();
    
        // process incoming bytes (checkUIData returns true if an ACK_CHAR is received)
        // return true if ACK_CHAR received
        if(checkUIData())
            return true; 
        
        // if no ACK_CHAR received, send GO_CHAR again
        BytesReceived = 0;
        Count++;
        
        if(Count == 50)
        {
            Count = 0;
            CycleCount++;
            delay(1000);
        }
    }
   
    // if no ack has be received, turn off serial communication and return false
    _SerialTx = false;
    digitalWrite(6,HIGH); // for debugging
    return false;
        
}

/*---------------------------------------------------
 getData: get data from active datasets
 this command can be run each loop to acquire data
 dataRdy() must be checked before running getData()
 ---------------------------------------------------*/
void ArduEye::getData()
{
	int i, k, InSize, Rows, Cols, c, Idx, remaining;
	unsigned char Header[FULL_HEAD_SIZE];
   //Header: 1 byte DataId, 2 byte rows, 2 byte cols
      
    // loop through active datasets
    for (k = 0; k < _NumActiveSets; k++)
    {
      	// read data packet header
        digitalWrite(_chipSelectPin, LOW);
        // set SPI link to Write mode
        SPI.transfer(ESC_CHAR);
        SPI.transfer(WRITE_CHAR);
        
        // request dataset header
        SPI.transfer(ESC_CHAR);
        SPI.transfer(START_PCKT);
        SPI.transfer(SOH_CHAR); 
        SPI.transfer(_DS[_ActiveSets[k]].DSID);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(END_PCKT);
        
        // set spi link to read mode
        SPI.transfer(ESC_CHAR);
        SPI.transfer(READ_CHAR);
        //delay to allow ArduEye time to prepare header
        delayMicroseconds(1);
        // read header data
        for (i = 0; i < FULL_HEAD_SIZE; i++)
          Header[i] = SPI.transfer(0x00);
        digitalWrite(_chipSelectPin, HIGH);
        
        // assign row and colum data for serial display      
        Rows = (Header[1] << 8) + Header[2];
        Cols = (Header[3] << 8) + Header[4];
        // assign incoming dataset size
        InSize = Rows * Cols;
       
		// write header data to serial monitor or UI if active
		if(_SerialTx)
        {
           if(_SerialMonitorMode)
           { 
               // serial monitor mode is used primarily for debugging
               // print header info in a legible way
               if(checkBufferFull())
               {
                   Serial.print(_DS[_ActiveSets[k]].DSID); Serial.print(" ");
                   Serial.print(Rows, DEC); Serial.print(" ");
                   Serial.print(Cols, DEC);Serial.print(" ");
                   Serial.print(_DS[_ActiveSets[k]].DisplayType); Serial.print(" ");
                   Serial.println(" ");
               }
            }
            else
            {
                // Send header data packet to UI if buffer is clear
                if(checkBufferFull())
                {
                    Serial.print((char)ESC_CHAR);
                    Serial.print((char)START_PCKT);  // send start of packet byte
                    for(i = 0; i < FULL_HEAD_SIZE; i++) // send header packet data
                    {
                        Serial.print(Header[i]);
                        if(Header[i] == ESC_CHAR)
                            Serial.print(Header[i]);
                    }
                    Serial.print((char)(_DS[_ActiveSets[k]].DisplayType)); // append display type
                    Serial.print((char)ESC_CHAR);
                    Serial.print((char)END_PCKT);  //send end of packet byte   
                }
            }
         }
        
        // abort read if size data is incorrect
        if(InSize <= 0)
            break;
          
          // send start of packet bytes via serial if _SerialTx is active
          if(_SerialTx)
          {
              if(_SerialMonitorMode) // print to serial monitor mode
              {
                  Serial.print(ESC_CHAR);
                  Serial.print(START_PCKT);
                  Serial.println(_DS[_ActiveSets[k]].DSID,DEC);
              }
              else  // print to UI mode
              {
                  Serial.print((char)ESC_CHAR); 
                  Serial.print((char)START_PCKT); // send start of packet byte
                  Serial.print((char)(_DS[_ActiveSets[k]].DSID)); // send dataset ID
                  // text display has a different format to tell UI what to display
                  if(_DS[_ActiveSets[k]].DisplayType == DISPLAY_TEXT)
                      Serial.print((char)Cols);
              }
          }
      
      
        // read data packet 
        // set spi link to write mode
        digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(WRITE_CHAR);
        
        // request dataset
        SPI.transfer(ESC_CHAR);
        SPI.transfer(START_PCKT);
        SPI.transfer(SOD_CHAR); 
        SPI.transfer(_DS[_ActiveSets[k]].DSID);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(END_PCKT);
        
        // set spi link to read mode
        SPI.transfer(ESC_CHAR);
        SPI.transfer(READ_CHAR);
        // delay to allow the ArduEye time to prepare the dataset
        delayMicroseconds(1);
        Idx = 0;
        // read MAX_SPI_PCKT_SIZE bytes of data at a time (max size defined in ArduEye.h)
        while(Idx + MAX_SPI_PCKT_SIZE < InSize)
        {
            for(i = 0; i < MAX_SPI_PCKT_SIZE; i++)
                _ReceiveBuffer[i] = SPI.transfer(0x00);
            
            // send data via serial
            if (_SerialTx)
            { 
                for (i = 0; i < MAX_SPI_PCKT_SIZE; i++)
                {
                    Serial.print(_ReceiveBuffer[i]);
                    //duplicate the data character if it is equal to the ESC_CHAR
                    if(_ReceiveBuffer[i] == ESC_CHAR)
                        Serial.print(_ReceiveBuffer[i]);
                }
                // print a spacer for legibility in serial monitor mode
                if(_SerialMonitorMode)
                    Serial.println(" ");
                
                // check that serial buffer is clear every two packets
                if(Idx % (MAX_SPI_PCKT_SIZE * 2) == 0)
                {     
                    if(!checkBufferFull())
                        break;
                }
            }
            Idx+= MAX_SPI_PCKT_SIZE;
          }
          remaining = InSize - Idx;
        
          // get remaining data
          if(remaining)
          {
              for(i = 0; i < remaining; i++)
                  _ReceiveBuffer[i] = SPI.transfer(0x00);
              
              // send data via serial
              if (_SerialTx)
              {   	
                  for (i = 0; i < remaining; i++)
                  {
                      Serial.print(_ReceiveBuffer[i]);
                      //duplicate the data character if it is equal to the ESC_CHAR
                      if(_ReceiveBuffer[i] == ESC_CHAR)
                          Serial.print(_ReceiveBuffer[i]);
                  }
                  if(_DS[_ActiveSets[k]].DisplayType == DISPLAY_TEXT)
                      Serial.print(_DS[_ActiveSets[k]].name);
                  // print a spacer for legibility in serial monitor mode
                  if(_SerialMonitorMode)
                      Serial.println(" ");
                  
              } 
          }
          digitalWrite(_chipSelectPin, HIGH);
          
        // send end of Packet bye
          if(_SerialTx)
          {
              if(_SerialMonitorMode) //send to serial monitor
              {
                  Serial.print((char)ESC_CHAR);
                  Serial.println((char)END_PCKT);
              }
              else     //send to UI
              {
                  Serial.print((char)ESC_CHAR);
                  Serial.print((char)END_PCKT);
              }
                  
          }        
  } 
  // when all datasets are received, call end of frame  
  endFrame();
}
/*---------------------------------------------------
 getDisplayType : read display type from DSRecord struct
 Input:   Dataset: Any of the values defined as "Dataset IDs"
           in the Sensor header file (ie ArmSensor.h)
 ---------------------------------------------------*/
int ArduEye::getDataIndex(char DataSet)
{
    for(int i = 0; i < MAX_DATASETS; i++)
    {
        if(_DS[i].DSID == DataSet)
           return i;
    }
           
    return 0;
}
/*---------------------------------------------------
 getDataSet:  Acquire one dataset from ArduEye and send data
 via serial if serial link is active.  Data will be stored in the 
 Buf array.  The max size for Buf is 512 bytes. If the dataset is 
 larger that this, Buf will contain a partial dataset.  This function
 can be called each loop.  dataRdy() must be checked each loop before
 getDataSet (getDataSet can be called for as many datasets as desired)and
 endFrame() must be called each loop after all datasets are acquired
 Input:   Dataset: Any of the values defined as "Dataset IDs"
            in the Sensor header file (ie ArmSensor.h) 
          Buf: Array to store Dataset
 ---------------------------------------------------*/
void ArduEye::getDataSet(char DataSet, char *Buf)
{
    int i, k, InSize, Rows, Cols, c, Idx, remaining;
	unsigned char Header[FULL_HEAD_SIZE];
    //Header: 1 byte DataId, 2 byte rows, 2 byte cols
    
    // find index of DataSet Settings
    int DataIdx = getDataIndex(DataSet);
    int DisplayType = _DS[DataIdx].DisplayType;
    
	// read data packet header
    digitalWrite(_chipSelectPin, LOW);
    // set SPI link to Write mode
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    // request dataset header
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(SOH_CHAR); 
    SPI.transfer(DataSet);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);
    
    // set spi link to read mode
    SPI.transfer(ESC_CHAR);
    SPI.transfer(READ_CHAR);
    //delay to allow ArduEye time to prepare header
    delayMicroseconds(1);
    // read header data
    for (i = 0; i < FULL_HEAD_SIZE; i++)
        Header[i] = SPI.transfer(0x00);
    digitalWrite(_chipSelectPin, HIGH);
    
    // assign row and colum data for serial display      
    Rows = (Header[1] << 8) + Header[2];
    Cols = (Header[3] << 8) + Header[4];
    // assign incoming dataset size
    InSize = Rows * Cols;
    
    // write header data to serial monitor or UI if active
    if(_SerialTx)
    {
        if(_SerialMonitorMode)
        { 
            // serial monitor mode is used primarily for debugging
            // print header info in a legible way
            if(checkBufferFull())
            {
                Serial.print(DataSet); Serial.print(" ");
                Serial.print(Rows, DEC); Serial.print(" ");
                Serial.print(Cols, DEC);Serial.print(" ");
                Serial.print(DisplayType); Serial.print(" ");
                Serial.println(" ");
            }
        }
        else
        {
            // Send header data packet to UI if buffer is clear
            if(checkBufferFull())
            {
                Serial.print((char)ESC_CHAR);
                Serial.print((char)START_PCKT);  // send start of packet byte
                for(i = 0; i < FULL_HEAD_SIZE; i++) // send header packet data
                {
                    Serial.print(Header[i]);
                    if(Header[i] == ESC_CHAR)
                        Serial.print(Header[i]);
                }
                Serial.print((char)DisplayType); // append display type
                Serial.print((char)ESC_CHAR);
                Serial.print((char)END_PCKT);  //send end of packet byte   
            }
        }
    }
    
    // abort read if size data is incorrect
    if(InSize <= 0)
        return;
    
    // send start of packet bytes via serial if _SerialTx is active
    if(_SerialTx)
    {
        if(_SerialMonitorMode) // print to serial monitor mode
        {
            Serial.print(ESC_CHAR);
            Serial.print(START_PCKT);
            Serial.println(DataSet,DEC);
        }
        else  // print to UI mode
        {
            Serial.print((char)ESC_CHAR); 
            Serial.print((char)START_PCKT); // send start of packet byte
            Serial.print((char)DataSet); // send dataset ID
            // text display has a different format to tell UI what to display
            if(DisplayType == DISPLAY_TEXT)
                Serial.print((char)Cols);
        }
    }
    
    
    // read data packet 
    // set spi link to write mode
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    // request dataset
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(SOD_CHAR); 
    SPI.transfer(DataSet);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);
    
    // set spi link to read mode
    SPI.transfer(ESC_CHAR);
    SPI.transfer(READ_CHAR);
    // delay to allow the ArduEye time to prepare the dataset
    delayMicroseconds(1);
    Idx = 0;
    // read MAX_SPI_PCKT_SIZE bytes of data at a time (max size defined in ArduEye.h)
    while(Idx + MAX_SPI_PCKT_SIZE < InSize)
    {
        for(i = 0; i < MAX_SPI_PCKT_SIZE; i++)
            _ReceiveBuffer[i] = SPI.transfer(0x00);
        
        // send data via serial
        if (_SerialTx)
        { 
            for (i = 0; i < MAX_SPI_PCKT_SIZE; i++)
            {
                Serial.print(_ReceiveBuffer[i]);
                //duplicate the data character if it is equal to the ESC_CHAR
                if(_ReceiveBuffer[i] == ESC_CHAR)
                    Serial.print(_ReceiveBuffer[i]);
            }
            // print a spacer for legibility in serial monitor mode
            if(_SerialMonitorMode)
                Serial.println(" ");
            
            // check that serial buffer is clear every two packets
            if(Idx % (MAX_SPI_PCKT_SIZE * 2) == 0)
            {     
                if(!checkBufferFull())
                    break;
            }
        }
        Idx+= MAX_SPI_PCKT_SIZE;
    }
    remaining = InSize - Idx;
    
    // get remaining data
    if(remaining)
    {
        for(i = 0; i < remaining; i++)
            Buf[i] = SPI.transfer(0x00);
        
        // send data via serial
        if (_SerialTx)
        {   	
            for (i = 0; i < remaining; i++)
            {
                Serial.print(Buf[i]);
                //duplicate the data character if it is equal to the ESC_CHAR
                if(Buf[i] == ESC_CHAR)
                    Serial.print(Buf[i]);
            }
            if(_DS[_ActiveSets[k]].DisplayType == DISPLAY_TEXT)
                Serial.print(_DS[DataIdx].name);
            // print a spacer for legibility in serial monitor mode
            if(_SerialMonitorMode)
                Serial.println(" ");
            
        } 
    }
    digitalWrite(_chipSelectPin, HIGH);
    
    // send end of Packet bye
    if(_SerialTx)
    {
        if(_SerialMonitorMode) //send to serial monitor
        {
            Serial.print((char)ESC_CHAR);
            Serial.println((char)END_PCKT);
        }
        else     //send to UI
        {
            Serial.print((char)ESC_CHAR);
            Serial.print((char)END_PCKT);
        }
        
    }        
}

/*---------------------------------------------------
 endFrame: send end of fram flags to ArduEye and UI
 This function must be called in embedded read mode each loop
 after all datasets have been read.  It is called automatically
 if using getData()
 ---------------------------------------------------*/
void ArduEye::endFrame()
{
    //send End of Data Indicator to ArduEye
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(END_FRAME);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);   
    digitalWrite(_chipSelectPin, HIGH);
    
    // send End of Frame Indicator to UI
    if((_NumActiveSets > 0) && _SerialTx && !_SerialMonitorMode)
    {
        Serial.print((char)ESC_CHAR);
        Serial.print((char)START_PCKT);
        Serial.print((char)END_FRAME);
        Serial.print((char)ESC_CHAR);
        Serial.print((char)END_PCKT);
    }
}

/*---------------------------------------------------
dataRdy: check data ready pin to see if ArduEye Data is ready
dataRdy() must be called each loop before getData()
 ---------------------------------------------------*/
boolean ArduEye::dataRdy()
{
	return (digitalRead(_dataReadyPin) == HIGH);
}

/*---------------------------------------------------
sensorRdy: check data ready pin to see if ArduEye is booted
sensorRdy() must be called before sending any intialization cmds
---------------------------------------------------*/
boolean ArduEye::sensorRdy()
{
	return (digitalRead(_dataReadyPin) == HIGH);
}

/*---------------------------------------------------
calibrate: send calibrate command to ArduEye
 calibrate generates a new Fixed Pattern noise mask for the vison chip
 ---------------------------------------------------*/
void ArduEye::calibrate()
{
	sendCommand(CMD_CALIBRATE, 0, 0);
}

/*---------------------------------------------------
setResolution: change raw image resolution
Input:  rows:  number of pixel rows
        cols:  number of pixel columns
---------------------------------------------------*/
void ArduEye::setResolution(int rows, int cols)
{
	char Cmd[2] = {rows,cols};
	sendCommand(CMD_RESOLUTION, Cmd, 2);
}
 
/*---------------------------------------------------
 setOFResolution: change optic flow array resolution 
  this command sets the number of optic flow 
  computation regions in the image 
 Input:   rows : number of bins in vertical direction
          cols: number of bins in horizontal direction
 ---------------------------------------------------*/
void ArduEye::setOFResolution(int rows, int cols)
{
	char Cmd[2] = {rows,cols};
	sendCommand(CMD_OF_RESOLUTION, Cmd, 2);
}

/*---------------------------------------------------
 sendCommand: send a user defined command to the ArduEye via SPI
 Input:   Cmd: Cmd value
          Value: Array of command parameters
          Size: number of bytes to read in Value array
 ---------------------------------------------------*/
void ArduEye::sendCommand(char Cmd, char * Value, int Size)
{
	int i;
    // lower chip select
    digitalWrite(_chipSelectPin, LOW);
    
    // set write mode
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    // send command start bytes
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT); // Size = Cmd byte + size of value array
    SPI.transfer(Cmd);

    // send command values
	for (i = 0; i < Size; i++)
	  	SPI.transfer(Value[i]);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);

    // raise chip select
	digitalWrite(_chipSelectPin, HIGH);
    
    // print command to Serial Monitor if active.
    if(_SerialMonitorMode && _SerialTx)  
    { 
        Serial.println(Cmd);
        for(i = 0; i < Size; i++)
        {
            Serial.print(Value[i]);
            Serial.print(" ");
        }
        Serial.println("cmd sent");
    }
	
}


/*---------------------------------------------------
 checkUIDATA: check communicatin from UI and process 

 ---------------------------------------------------*/
bool ArduEye::checkUIData(void)
{
	int i;
	int BytesReceived; 
    bool AckReceived = false;
    int FrameIdx = _InBufIdx;
    
    // check for available data on the serial port
    BytesReceived = Serial.available();
    
    // if there is not enough space in the buffer to record the new data
    // if not set the _InBufIdx to zero and continue recording there
    if(_InBufIdx + BytesReceived > MAX_IN_SERIAL)
    {
        _BufEnd = _InBufIdx;
        _InBufIdx = 0;
    }
    //  if data is available, copy it into the _InBuffer
	if(BytesReceived)
	{
		for (i = 0; i < BytesReceived; i++)
            _InBuffer[_InBufIdx++] = Serial.read();
        
        Toggle2 = !Toggle2;
        if(Toggle2)
            digitalWrite(7, HIGH);
        else
            digitalWrite(7, LOW);
        delay(1);
    }
    
    // for each byte receieced, check for special character
    for (i = 0; i < BytesReceived; i++)
    {
        if(_ESCReceived)
        {   
            
            switch(_InBuffer[FrameIdx + i])
            {
                case START_PCKT:
                    _InSIdx = FrameIdx + i;
                    Toggle = !Toggle;
                    if(Toggle)
                        digitalWrite(6, HIGH);
                    else
                        digitalWrite(6, LOW);
                    
                    break;
                case END_PCKT:
                    parseCmd(_InSIdx, FrameIdx + i);
                    break;
                // Ack char is sent by the UI in response to a ping from the Arduino requesting permission to continue.  If the
                // serial buffer is overflowing ACK_CHAR will not sent
                case ACK_CHAR:
                    AckReceived = true;
                    break;
                case ESC_CHAR:
                    //ignore
                default:
                    break;
            }
            _ESCReceived = false;
        }
        // check for escape characters.  The ESC_CHAR is sent at the beginning of all communcation.  
        else if(_InBuffer[FrameIdx + i] == ESC_CHAR)
            _ESCReceived = true;
        
    }
    return AckReceived;
}

/*---------------------------------------------------
 parseCmd: Process UI command and send to ArduEye if 
 appropriate. 
 Input:  StartIdx: Index of START_PCKT flag
         EndIdx: Index of END_PCKT flag
 ---------------------------------------------------*/
void ArduEye::parseCmd(int StartIdx, int EndIdx)
{
	int i, Idx = 0;
    //command buffer
    char cmd[MAX_CMD_SIZE];
    boolean on;
    
    // if serial input is active, send Command Acknowledge
    // UI checks for Command Acknowledge and will re-send command if needed
    if(_SerialTx)
    {
        Serial.print((char)ESC_CHAR);
        Serial.print((char)CMD_ACK);
    }
    
    // Read data between start and end indices to a buffer
    // case where packet wraps around the circular input buffer
    if(StartIdx > EndIdx)
    {
        for(i = StartIdx+1; i < _BufEnd; i++)
            cmd[Idx++] = _InBuffer[i];
        for(i = 0; i < EndIdx-1; i++)
            cmd[Idx++] = _InBuffer[i];
    }
    // case where data is in one non-wrapping sequence
    else
        for(i = StartIdx+1; i < EndIdx-1; i++)
            cmd[Idx++] = _InBuffer[i];
	
    // parse command
    switch(cmd[0])
    {
      // start new dataset acquire (This command updates the dataset status on the Arduino and the ArduEye)
      case DISPLAY_CMD:
        startDataStream(cmd[1]);
        break;
      // stop dataset acquire (This command updates the dataset status on the Arduino and the ArduEye)
      case STOP_CMD:
        stopDataStream(cmd[1]);
        break;
      // write a command to the ArduEye
      case WRITE_CMD:
          sendCommand(cmd[0], cmd + 1, Idx-1);
      
          // print command to serial monitor if _SerialMonitorMode is active
          if(_SerialMonitorMode && _SerialTx)  
          {          
            for(int i = 0; i < Idx; i++)
              Serial.print(cmd[i]);
            Serial.println("  cmd sent");
          }
          break;
        // start or stop serial txmission
        // when serial txmission is on, all acquired datasets will be send out via serial.
        case SERIAL_START:
            on = (cmd[1] > 0) ? true : false;
            enableSerialTx(on);
            break;
       default:
          break; 
    }
}
/*---------------------------------------------------
 enableSerialTx: Enable Serial Communication.  Enabled by default
 When enabled, all data sets will be sent out over serial to either the 
 ArduEye UI or the Serial Monitor
 Input:  Enable: true to enable or false to disable
 ---------------------------------------------------*/
void ArduEye::enableSerialTx(boolean Enable)
{
	_SerialTx = Enable;
}
/*---------------------------------------------------
setSerialMonitorMode: SerialMonitorMode is disabled by default.  When enabled
serial data will be formated for display on the serial monitor
SerialTx must be enabled for SerialMonitorMode to display data 
Input:  Enable: true to enable or false to disable
---------------------------------------------------*/
void ArduEye::setSerialMonitorMode(boolean Enable)
{
	_SerialMonitorMode = Enable;
}

/*---------------------------------------------------
 setDisplayType: define display type for a given dataset
 Input:  DSID: Any of the values defined as "Dataset IDs"
                in the Sensor header file (ie ArmSensor.h)
         DisplayType: Any of the values defined as "Display Types"
                in ArdyEye.h
 ---------------------------------------------------*/
void ArduEye::setDisplayType(int DSID, int DisplayType)
{
    for (int i = 0; i < MAX_DATASETS; i++)
    {  
        // if record matching DSID is found, set its DisplayType
        if(_DS[i].DSID == DSID)
        {
            _DS[i].DisplayType = DisplayType;
            break;
        }
    }
}
	

