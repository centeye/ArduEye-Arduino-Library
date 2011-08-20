/*
ArduEye Interface Example using the ArduEye library.

The ArduEye library implements an interface to the ArduEye Sensor using 
a 5 wire interface (SPI + a DataReady wire)
The SPI pins are defined in the SPI library ( MOSI: 11, MISO: 12, SCK: 13)
The user must define pins for Chip Select and DataReady.


The ArduEye library can be used as a pure embedded sensor interface, using
the optical flow or image data to control other embedded behaviors.

It can also output data to the Serial Monitor for diagnostics. This mode is off
by default but can be turned on by calling ArduEye.setSerialMonitorMode(true);

Data can also be read by the ArduEye UI.  The UI provides additional diagonostic
capbilites and can display a graphical representation of the raw image and 
image processing data.  The UI can be activated at any time while the sensor
is running to check and see what's happening

To Activate Datasets using startDataStream(), the following IDs can be used:
ARDUEYE_ID_RAW
ARDUEYE_ID_OFX
ARDUEYE_ID_OFY
ARDUEYE_ID_FPS

*/

#include "WProgram.h"
#include <SPI.h>
#include <ArduEye.h>

ArduEye arduEye;

int dataReadyPin = A0;
int chipSelectPin = 10;


void setup()
{
  char Val[3] = {71, 112, 112};
  arduEye.begin(dataReadyPin, chipSelectPin);
//  arduEye.enableSerialTx(false);
  
//  delay(5000);
  //arduEye.sendCommand(WRITE_CMD, Val, 3);
//  arduEye.startDataStream(ARDUEYE_ID_RAW);
//  arduEye.startDataStream(ARDUEYE_ID_OFX);
//  arduEye.startDataStream(ARDUEYE_ID_OFY);
//  arduEye.startDataStream(ARDUEYE_ID_FPS);
 // arduEye.setSerialMonitorMode(true);
}

void loop()
{
  if(arduEye.dataRdy())
      arduEye.getData();
      
   arduEye.checkUIData();
      
 //  delay(5);   
}


