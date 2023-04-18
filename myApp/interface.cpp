#include "myApp.h"
#include "interface.h"

// sensor libraries for data interperting
#include "meas_acc/resources.h"
#include "meas_gyro/resources.h"
#include "meas_magn/resources.h"
#include "meas_imu/resources.h"

uint8_t count[] = {48};
//Commands formated as byte array with [Command, data?...] (data optional)
enum Commands 
{
    HELLO = 0,
    BLINK = 9,
    BEGIN_SUB=1,
    END_SUB=2,
};
//Responses formated as byte array with [Response, tag, data?...] (data optional)
enum Responses 
{
    COMMAND_RESULT = 1,
    DATA = 2,
    ERROR = 3,
};
    
const char IMUPath[]="/Meas/IMU6/104";
const uint8_t DEFAULT_REFERENCE=99; //appears as 63 in hex

void myApp::handleCommand(uint8_t cmd, const uint8_t values[], size_t len){
    switch (cmd)
    {
        case Commands::HELLO:
        {
            // Hello response, for use as a sanity check <3
            uint8_t helloMsg[] = {'H','e','l','l','o','!'};
            //tags aren't explicitly necessary, but they're a good way of grouping responses.
            //The included console uses them to filter and format responses.
            uint8_t tag=1;
            sendPacket(helloMsg, sizeof(helloMsg), tag, Responses::COMMAND_RESULT);
	    break;
        }
	case Commands::BLINK:
        {
                ledSetPattern(1000, 2000, 3);
		break;
        }
	case Commands::BEGIN_SUB:
        {
            //unsubscribes to prevent duplicate subscriptions
            unsubscribe(DEFAULT_REFERENCE);
            //subscribes to the path given above, in this case the IMU at 104hz
            subscribe(IMUPath, sizeof(IMUPath), DEFAULT_REFERENCE);
	    break;
        }
        case Commands::END_SUB:
        {
            //unsubscribes only from default service
            unsubscribe(DEFAULT_REFERENCE);
	    count[0] = {48};
        }
        break;
        
    }
}

void myApp::processData(wb::ResourceId resourceId, const wb::Value &value){


  if (findDataSub(resourceId) ->clientReference != DEFAULT_REFERENCE)
  	  return;
  const WB_RES::IMU6Data &data = value.convertTo<WB_RES::IMU6Data&>();
  float magnitudes[16];
  const wb::Array<wb::FloatVector3D> &accData = data.arrayAcc;

  float avgMag = 0;
  size_t i;
  for(i=0; i<15 && i< accData.size(); i++) {
        wb::FloatVector3D a = accData[i];
	float magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
	magnitudes[i+1] = magnitude;
	avgMag += magnitude;
  }
  avgMag /= i;
  *((char *) magnitudes+3) = avgMag <3.0f? 1:0;
  uint8_t tag = 2;
  sendPacket((uint8_t *)magnitudes+3, 1+i*sizeof(float), tag, Responses::DATA);


  /*
  float averageMagnitude[1];
  float x_avg = 0;
  float y_avg = 0;
  float  z_avg = 0;

  size_t i;
  for(i = 0; i<accData.size(); i++) {
	  wb::FloatVector3D a = accData[i];
	  float magnitude = sqrt(a.x*a.x + a.y*a.y + a.z*a.z);
	  averageMagnitude[0] += magnitude;
	  x_avg += a.x;
	  y_avg += a.y;
	  z_avg += a.z;
  }
  averageMagnitude[0] = averageMagnitude[0]/i;
  x_avg /= i;
  y_avg /= i;
  z_avg /= i;

  uint8_t tag = 5;

  //if (y_avg > 11.0f and z_avg > 2.0f ) {
	count[0]++; 
	sendPacket((uint8_t *)(averageMagnitude), sizeof(count), tag, Responses::COMMAND_RESULT);

  	//sendPacket(count, sizeof(count), tag, Responses::COMMAND_RESULT);
  	ledSetPattern(1000,2000,1);
  //}
 */
}
