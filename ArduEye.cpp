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
	_SerialTx = false;
	_SerialMonitorMode = false;
    _NumActiveSets = 0;
    _InSIdx = _InBufIdx = 0;
    _ESCReceived = false;
    _BufEnd = 0;
    Toggle = Toggle2 = false;
}

// Initialize Ardue Eye Interface
void ArduEye::begin(int RdyPin, int CSPin)
{
	_dataReadyPin = RdyPin;
	_chipSelectPin = CSPin;
    
    pinMode(6, OUTPUT);
    digitalWrite(6,LOW);
    
    pinMode(7, OUTPUT);
    digitalWrite(7,LOW);
	
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
      _DS[0].DisplayType = DISPLAY_GRAYSCALE_IMAGE;
    
	  _DS[1].DSID = ARDUEYE_ID_OFX;
      _DS[1].DisplayType = DISPLAY_CHARTX;
	  
	  _DS[2].DSID = ARDUEYE_ID_OFY;
      _DS[2].DisplayType = DISPLAY_CHARTY;
	  
	  _DS[3].DSID = ARDUEYE_ID_FPS;
      _DS[3].DisplayType = DISPLAY_TEXT;
    
      _DS[4].DSID = ARDUEYE_ID_CMD;
      _DS[4].DisplayType = DISPLAY_DUMP;
        
      _DS[5].DSID = ARDUEYE_ID_MAXES;
      _DS[5].DisplayType = DISPLAY_POINTS;
	 	
}

// Start streaming a new dataset from the ArduEye
// multiple datasets can be active at the same time
void ArduEye::startDataStream(char DataSet)
{
  int i;  
  sendCommand((char)DISPLAY_CMD, &DataSet, 1);
  
  // update DSRecord
  for (i = 0; i < MAX_DATASETS; i++)
  {   
    if(_DS[i].DSID == DataSet)
    {
        if(_DS[i].Active == false)
        {
            _ActiveSets[_NumActiveSets] = i;
            _NumActiveSets++;
        }
        _DS[i].Active = true;
      break;
    }
  }
    if(_SerialTx && _SerialMonitorMode)
    {
        for(i = 0; i < _NumActiveSets; i++)
            Serial.print(_ActiveSets[i], DEC);
        Serial.println(" ");
    }
}

// Stop streaming a dataset from the ArduEye
// Any other active datasets will continue to stream
void ArduEye::stopDataStream(char DataSet)
{  
    int i, m;  
    sendCommand((char)STOP_CMD, &DataSet, 1);
    
      // update DSRecord
      for (i = 0; i < MAX_DATASETS; i++)
      {   
        if(_DS[i].DSID == DataSet)
        {
          _DS[i].Active = false;
          break;
        }
      }
  
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
    if(_SerialTx && _SerialMonitorMode)
    {
        for(i = 0; i < _NumActiveSets; i++)
            Serial.print(_ActiveSets[i],DEC);
        Serial.println(" ");
    }
    
}

// check that serial buffer is not full, so it is OK to send data
boolean ArduEye::checkBufferFull()
{
    int BytesReceived = 0;
    int Count = 0, CycleCount = 0;
    char inByte;
    
    while(Count < 50)
    {
        Serial.print((char)ESC_CHAR);
        Serial.print((char)GO_CHAR);
        
        while(BytesReceived == 0)
            BytesReceived = Serial.available();
    
        if(checkUIData())
            return true; 
        
        BytesReceived = 0;
        Count++;
    }
    if(Count == 50)
    {
        Count = 0;
        CycleCount++;
        delay(2000);
        if(CycleCount > 10)
        {
            _SerialTx = false;
            digitalWrite(6,HIGH);
            return false;
        }
    }
        
}

// get data from active datasets
// this command can be run each loop to update
// dataRdy() must be checked before running getData()
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
        SPI.transfer(ESC_CHAR);
        SPI.transfer(WRITE_CHAR);
        
        SPI.transfer(ESC_CHAR);
        SPI.transfer(START_PCKT);
        SPI.transfer(SOH_CHAR); 
        SPI.transfer(_DS[_ActiveSets[k]].DSID);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(END_PCKT);
        
        SPI.transfer(ESC_CHAR);
        SPI.transfer(READ_CHAR);
        delayMicroseconds(1);
        for (i = 0; i < FULL_HEAD_SIZE; i++)
          Header[i] = SPI.transfer(0x00);
        digitalWrite(_chipSelectPin, HIGH);
        
        // assign row and colum data for serial display      
        Rows = (Header[1] << 8) + Header[2];
        Cols = (Header[3] << 8) + Header[4];
        InSize = Rows * Cols;
       

		// write header data to serial monitor / UI if active
		if(_SerialTx)
        {
           if(_SerialMonitorMode)
           { 
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
          
          // send start of packet bytes
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
              }
          }
      
      
        // read data packet 
         digitalWrite(_chipSelectPin, LOW);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(WRITE_CHAR);
        
        SPI.transfer(ESC_CHAR);
        SPI.transfer(START_PCKT);
        SPI.transfer(SOD_CHAR); 
        SPI.transfer(_DS[_ActiveSets[k]].DSID);
        SPI.transfer(ESC_CHAR);
        SPI.transfer(END_PCKT);
        
        SPI.transfer(ESC_CHAR);
        SPI.transfer(READ_CHAR);
        delayMicroseconds(1);
        Idx = 0;
        while(Idx + MAX_SPI_PCKT_SIZE < InSize)
        {
            for(i = 0; i < MAX_SPI_PCKT_SIZE; i++)
                _ReceiveBuffer[i] = SPI.transfer(0x00);
            
            if (_SerialTx)
            {   	
                for (i = 0; i < MAX_SPI_PCKT_SIZE; i++)
                {
                    Serial.print(_ReceiveBuffer[i]);
                    if(_ReceiveBuffer[i] == ESC_CHAR)
                        Serial.print(_ReceiveBuffer[i]);
                }
                if(_SerialMonitorMode)
                    Serial.println(" ");
                
                if(Idx % (MAX_SPI_PCKT_SIZE * 2) == 0)
                {     
                    if(!checkBufferFull())
                        break;
                }
            }
            Idx+= MAX_SPI_PCKT_SIZE;
          }
          remaining = InSize - Idx;
        
          if(remaining)
          {
              for(i = 0; i < remaining; i++)
              {
                  delayMicroseconds(1);
                  _ReceiveBuffer[i] = SPI.transfer(0x00);
              }
              
              if (_SerialTx)
              {   	
                  for (i = 0; i < remaining; i++)
                  {
                      Serial.print(_ReceiveBuffer[i]);
                      if(_ReceiveBuffer[i] == ESC_CHAR)
                          Serial.print(_ReceiveBuffer[i]);
                  }
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
  endFrame();
}
int ArduEye::getDisplayType(char DataSet)
{
    for(int i = 0; i < MAX_DATASETS; i++)
    {
        if(_DS[i].DSID == DataSet)
           return _DS[i].DisplayType;
    }
           
    return 0;
}
void ArduEye::getDataSet(char DataSet, char *Buf)
{
    int i, k, InSize, Rows, Cols, c, Idx, remaining;
	unsigned char Header[FULL_HEAD_SIZE];
    //Header: 1 byte DataId, 2 byte rows, 2 byte cols
    
    int DisplayType = getDisplayType(DataSet);
    

    // read data packet header
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(SOH_CHAR); 
    SPI.transfer(DataSet);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(READ_CHAR);
    delayMicroseconds(1);
    for (i = 0; i < FULL_HEAD_SIZE; i++)
        Header[i] = SPI.transfer(0x00);
    digitalWrite(_chipSelectPin, HIGH);
    
    // assign row and colum data for serial display      
    Rows = (Header[1] << 8) + Header[2];
    Cols = (Header[3] << 8) + Header[4];
    InSize = Rows * Cols;
    
    
    // write header data to serial monitor / UI if active
    if(_SerialTx)
    {
        if(_SerialMonitorMode)
        { 
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
    
    // send start of packet bytes
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
        }
    }
    
    // read data packet 
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(SOD_CHAR); 
    SPI.transfer(DataSet);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(READ_CHAR);
    delayMicroseconds(1);
    Idx = 0;
    while(Idx + MAX_SPI_PCKT_SIZE < InSize)
    {
        for(i = 0; i < MAX_SPI_PCKT_SIZE; i++)
            _ReceiveBuffer[i] = SPI.transfer(0x00);
        
        if (_SerialTx)
        {   	
            for (i = 0; i < MAX_SPI_PCKT_SIZE; i++)
            {
                Serial.print(_ReceiveBuffer[i]);
                if(_ReceiveBuffer[i] == ESC_CHAR)
                    Serial.print(_ReceiveBuffer[i]);
            }
            if(_SerialMonitorMode)
                Serial.println(" ");
            
            if(Idx % (MAX_SPI_PCKT_SIZE * 2) == 0)
            {     
                if(!checkBufferFull())
                    break;
            }
        }
        Idx+= MAX_SPI_PCKT_SIZE;
    }
    remaining = InSize - Idx;
    
    if(remaining > 0)
    {
        for(i = 0; i < remaining; i++)
            Buf[i] = SPI.transfer(0x00);
        
        if (_SerialTx)
        {   	
            for (i = 0; i < remaining; i++)
            {
                Serial.print(Buf[i]);
                if(Buf[i] == ESC_CHAR)
                    Serial.print(Buf[i]);
            }
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

void ArduEye::endFrame()
{
    //send End of Data Indicator to ArduEye
    digitalWrite(_chipSelectPin, LOW);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT);
    SPI.transfer(EOD_CHAR);
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

// check data ready pin to see if ArduEye Data is ready
// dataRdy() must be called before getData()
boolean ArduEye::dataRdy()
{
	return (digitalRead(_dataReadyPin) == HIGH);
}

// check data ready pin to see if ArduEye is booted
// sensorRdy() must be called before sending any intialization cmds
boolean ArduEye::sensorRdy()
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

// send a user defined command to the ArduEye
//
void ArduEye::sendCommand(char Cmd, char * Value, int Size)
{
	int i;
    digitalWrite(_chipSelectPin, LOW);
    
    // set write mode
    SPI.transfer(ESC_CHAR);
    SPI.transfer(WRITE_CHAR);
    // send command
    SPI.transfer(ESC_CHAR);
    SPI.transfer(START_PCKT); // Size = Cmd byte + size of value array
    SPI.transfer(Cmd);

	for (i = 0; i < Size; i++)
	  	SPI.transfer(Value[i]);
    SPI.transfer(ESC_CHAR);
    SPI.transfer(END_PCKT);
	digitalWrite(_chipSelectPin, HIGH);
    
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


// check for communication from UI and pass commands on to ArduEye
bool ArduEye::checkUIData(void)
{
	int i;
	int BytesReceived; 
    bool AckReceived = false;
    int FrameIdx = _InBufIdx;
    
 //   for (i= 0; i < Count; i++)
 //       _InBuffer[_InBufIdx++] = bytesIn[i];
    
    BytesReceived = Serial.available();
    
    
 
    if(_InBufIdx + BytesReceived > MAX_IN_SERIAL)
    {
        _BufEnd = _InBufIdx;
        _InBufIdx = 0;
    }
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
        else if(_InBuffer[FrameIdx + i] == ESC_CHAR)
            _ESCReceived = true;
        
    }
    return AckReceived;
}

void ArduEye::parseCmd(int StartIdx, int EndIdx)
{
	int i, Idx = 0;
    char cmd[MAX_CMD_SIZE];
    
    Serial.print((char)ESC_CHAR);
    Serial.print((char)CMD_ACK);
    
    if(StartIdx > EndIdx)
    {
        for(i = StartIdx+1; i < _BufEnd; i++)
            cmd[Idx++] = _InBuffer[i];
        for(i = 0; i < EndIdx-1; i++)
            cmd[Idx++] = _InBuffer[i];
    }
    else
        for(i = StartIdx+1; i < EndIdx-1; i++)
            cmd[Idx++] = _InBuffer[i];
	       
    switch(cmd[0])
    {
      case DISPLAY_CMD:
        startDataStream(cmd[1]);
        break;
      case STOP_CMD:
        stopDataStream(cmd[1]);
        break;
      case WRITE_CMD:
          sendCommand(cmd[0], cmd + 1, Idx-1);
      
          if(_SerialMonitorMode && _SerialTx)  
          {          
            for(int i = 0; i < Idx; i++)
              Serial.print(cmd[i]);
            Serial.println("  cmd sent");
          }
          break;
       default:
          break; 
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

void ArduEye::setDisplayType(int DSID, int DisplayType)
{
    for (int i = 0; i < MAX_DATASETS; i++)
    {   
        if(_DS[i].DSID == DSID)
        {
            _DS[i].DisplayType = DisplayType;
            break;
        }
    }
}
	

