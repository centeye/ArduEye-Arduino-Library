/*
ArduEye Interface Example using the ArduEye library.

The ArduEye library implements an interface to the ArduEye Sensor using 
a 5 wire interface (SPI + a DataReady wire)
The SPI pins are defined in the SPI library ( MOSI: 11, MISO: 12, SCK: 13)
The Chip Select and Dataeady are defined by the ArduEye layout. (CS: 10, DataRdy: 9).  These
pins can be changed but the ArduEye firmware must be updated as well.

The ArduEye library can be used either as a pure embedded sensor interface (using
the optical flow or image data to control other embedded behaviors) or can pass data via serial to 
the ArduEye UI.  The UI provides additional diagonostic  capbilites and can display a graphical 
representation of the raw image and image processing data.  The UI can be activated at any time while the sensor
is running to check and see what's happening, even while running in embedded mode.

This example shows how one might interface with the ArduEye in embedded mode.  Setup commands and dataset calls are
issued by the Arduino sketch as the UI is not connected. 

*/

#include "WProgram.h"
#include <SPI.h>
#include <ArduEye.h>

ArduEye arduEye;

/// select pins for SPI chip select and DataReady.  These pins are defined on both the ARM and Arduino Pro Mini.  
/// different pins can be used if desired but the ARM firmware must also be adjusted accordingly
int dataReadyPin = 9;
int chipSelectPin = 10;

char OpticBuf[50];
char fps[2];


void setup()
{
 // begin must be called to initilialize the ArduEye class
  arduEye.begin(dataReadyPin, chipSelectPin);
  
  // serial TX enabled to check display output - would not need to enable in a pure embedded application
  // serial TX can also be enabled from the UI
  arduEye.enableSerialTx(true);
  
  // display types for each dataset are set to default values but can
  // also be defined here (not necessary for embedded mode, but good practice to
  // define in case one wants to plug in the UI)
  arduEye.setDisplayType(ARDUEYE_ID_RAW, DISPLAY_GRAYSCALE_IMAGE);
  
  // wait to see that ArduEye has booted before sending commands
  while(!arduEye.sensorRdy());
  
  // set datasets to active, locally and on ArduEye
  arduEye.startDataStream(ARDUEYE_ID_OF);
  arduEye.startDataStream(ARDUEYE_ID_FPS);
}

void loop()
{
  // check if ArduEye is ready to send data
  if(arduEye.dataRdy())
  {
    // retrieve data from ArduEye and store in local arrays
    arduEye.getDataSet(ARDUEYE_ID_OF, OpticBuf);
    arduEye.getDataSet(ARDUEYE_ID_FPS, fps);
    
    // end Frame must be sent at the end of each frame to flag the ArduEye that data tx is over image capture can start
    arduEye.endFrame();
  }
   
   // check if any commands have come in from the UI and process
   // in emebedded mode, setup commands can be issued via the UI but datasets cannot be started or stopped via the UI   
   arduEye.checkUIData();
}


