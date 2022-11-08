/* Sketch #4 - David Oppenheim 
 * DIGF 6037 Creation & Computation
 * 
 * Rotates a servo clockwise or counterclockwise based using the built-in IMU of the Arduino Nano33 (using roll values)
 
 * ATTRIBUTION: 
 * 
 * Pitch and roll code from Kate Hartman & Nick Puckett: 
 * Interface via the Sparkfun LSM6DS3 library: https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
 * Filter Code by Trent Cleghorm: https://github.com/tcleg/Six_Axis_Complementary_Filter
 * 
 */



#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "six_axis_comp_filter.h"

#include <Servo.h>



LSM6DS3 nano33IMU(I2C_MODE, 0x6A); //define the IMU object
CompSixAxis CompFilter(0.1, 2); //define the filter object

Servo cntServo1; 
int spin1 = 2;

float pitch;
float roll;

int rollValue; 


void setup() 
{
  Serial.begin(9600);
  
  //Call .begin() to configure the IMU (Inertial Measurement Unit)
  nano33IMU.begin();

  cntServo1.attach(spin1);
  cntServo1.write(88); // set servo to neutral to start 

}


void loop() 
{
calculatePitchAndRoll();

rollValue = roll; 

if (rollValue >=300 && rollValue <=345) {
  cntServo1.write(0);
}
  
else if (rollValue >= 0 && rollValue <= 50) {
cntServo1.write(180);
} 
  
  else {
    cntServo1.write(90); 
 }

}


void calculatePitchAndRoll()
{
  float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle;       

  //  Get all motion sensor (in this case LSM6DS3) parameters,
  //  If you're using a different sensor you'll have to replace the values
  accelX = nano33IMU.readFloatAccelX();
  accelY = nano33IMU.readFloatAccelY();
  accelZ = nano33IMU.readFloatAccelZ();

  gyroX = nano33IMU.readFloatGyroX();
  gyroY = nano33IMU.readFloatGyroY();
  gyroZ = nano33IMU.readFloatGyroZ();

  // Convert these values into angles using the Complementary Filter
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
  CompFilter.CompGyroUpdate(gyroX, gyroY, gyroZ); // takes arguments un rad/s 
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  // Get angle relative to X and Y axes and write them to the variables in the arguments
  //in radians
  CompFilter.CompAnglesGet(&xAngle, &yAngle);

  //convert from radians to angles
  pitch = xAngle*RAD_TO_DEG;
  roll = yAngle*RAD_TO_DEG;
  
  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(roll);

}
