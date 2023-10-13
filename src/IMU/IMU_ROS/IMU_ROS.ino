#include <Wire.h>
#include <SPI.h>
#include <SparkFunLSM9DS1.h>
#include <LSM9DS1_Types.h>
#include <LSM9DS1_Registers.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

//////////////////////////
// LSM9DS1 Library Init //
//////////////////////////
// Use the LSM9DS1 class to create an object. [imu] can be
// named anything, we'll refer to that throught the sketch.
LSM9DS1 imu;

///////////////////////
// Example I2C Setup //
///////////////////////
// SDO_XM and SDO_G are both pulled high, so our addresses are:
// #define LSM9DS1_M  0x1E // Would be 0x1C if SDO_M is LOW
// #define LSM9DS1_AG 0x6B // Would be 0x6A if SDO_AG is LOW

////////////////////////////
// Sketch Output Settings //
////////////////////////////
#define PRINT_SPEED 250 // 250 ms between prints
#define PI 3.1415926535897932384626433832795
#define GRAVITY 9.80665


static unsigned long lastPrint = 0; // Keep track of print time

// Earth's magnetic field varies by location. Add or subtract
// a declination to get a more accurate heading. Calculate
// your's here:
// http://www.ngdc.noaa.gov/geomag-web/#declination
#define DECLINATION -8.58 // Declination (degrees) in Boulder, CO.

//Function definitions
void printGyro();
void printAccel();
void printMag();
void printAttitude(float ax, float ay, float az, float mx, float my, float mz);

ros::NodeHandle n;

sensor_msgs::Imu imu_msg;

ros::Publisher imu_pub("imu", &imu_msg);
//ros::Rate loop_rate(10);


void setup()
{
  n.initNode();
  n.advertise(imu_pub);
}

void loop()
{
  // Update the sensor values whenever new data is available
  if ( imu.gyroAvailable() )
  {
    // To read from the gyroscope,  first call the
    // readGyro() function. When it exits, it'll update the
    // gx, gy, and gz variables with the most current data.
    imu.readGyro();
  }
  if ( imu.accelAvailable() )
  {
    // To read from the accelerometer, first call the
    // readAccel() function. When it exits, it'll update the
    // ax, ay, and az variables with the most current data.
    imu.readAccel();
  }
  if ( imu.magAvailable() )
  {
    // To read from the magnetometer, first call the
    // readMag() function. When it exits, it'll update the
    // mx, my, and mz variables with the most current data.
    imu.readMag();
  }

  if ((lastPrint + PRINT_SPEED) < millis())
  {
    imu_msg.angular_velocity.x = getGyro("x");  // Print "G: gx, gy, gz" //change all this to updating ROS node
    imu_msg.angular_velocity.y = getGyro("y");
    imu_msg.angular_velocity.z = getGyro("z");
    
    imu_msg.linear_acceleration.x = getAccel("x"); // Print "A: ax, ay, az"
    imu_msg.linear_acceleration.y = getAccel("y");
    imu_msg.linear_acceleration.z = getAccel("z");
    
    // The LSM9DS1's mag x and y
    // axes are opposite to the accelerometer, so my, mx are
    // substituted for each other
    imu_msg.orientation.x = getAttitude(imu.ax, imu.ay, imu.az,
                          -imu.my, -imu.mx, imu.mz, "x");
    imu_msg.orientation.y = getAttitude(imu.ax, imu.ay, imu.az,
                          -imu.my, -imu.mx, imu.mz, "y");
    imu_msg.orientation.z = getAttitude(imu.ax, imu.ay, imu.az,
                          -imu.my, -imu.mx, imu.mz, "z");
   
    // Not sure if magnetic_field is a field for imu_msg
    // imu_msg.magnetic_field.x = getMag("x");   // Print "M: mx, my, mz"
    // imu_msg.magnetic_field.y = getMag("y"); 
    // imu_msg.magnetic_field.z = getMag("z"); .
 

    lastPrint = millis(); // Update lastPrint time
  }

  imu_pub.publish(&imu_msg);
  n.spinOnce();

}

float getGyro(String axis)
{//val is in deg/s by default
  if (axis=="x")
  {
    float val = imu.calcGyro(imu.gx);
    return val*PI;
  }
  else if (axis=="y")
  {
      float val = imu.calcGyro(imu.gy);
      return val*PI;
  }
  else
  {
      float val = imu.calcGyro(imu.gz);
      return val*PI;
  }

}

float getAccel(String axis)
{// default value of val is g
  if (axis=="x")
  {
    float val = imu.calcAccel(imu.ax);
    return val*GRAVITY;
  }
  else if (axis=="y")
  {
    float val = imu.calcAccel(imu.ay);
    return val*GRAVITY;
  }
  else
  {
    float val = imu.calcAccel(imu.az);
    return val*GRAVITY;
  }
  
  // default unit of accel_info is g
  //accel_info = [imu.calcAccel(imu.ax), imu.calcAccel(imu.ay), imu.calcAccel(imu.az)];
  //for (int i=0; i<length(accel_info); i++){
  //  accel_info[i] = accel_info[i]*GRAVITY;
  //}
  //return accel_info; //accel_info is now m/s^2
}

float getMag(String axis)
{
  if (axis=="x")
  {
    return imu.calcMag(imu.mx);
    
  }
  else if (axis=="y")
  {
    return imu.calcMag(imu.my);
    
  }
  else
  {
    return imu.calcMag(imu.mz);
  }
    
  //return [imu.calcMag(imu.mx), imu.calcMag(imu.my), imu.calcMag(imu.mz)]; //Mag units are Gs
  
}

// Calculate pitch, roll, and heading.
// Pitch/roll calculations take from this app note:
// https://web.archive.org/web/20190824101042/http://cache.freescale.com/files/sensors/doc/app_note/AN3461.pdf
// Heading calculations taken from this app note:
// https://web.archive.org/web/20150513214706/http://www51.honeywell.com/aero/common/documents/myaerospacecatalog-documents/Defense_Brochures-documents/Magnetic__Literature_Application_notes-documents/AN203_Compass_Heading_Using_Magnetometers.pdf
float getAttitude(float ax, float ay, float az, float mx, float my, float mz, String axis)
{
  float roll = atan2(ay, az);
  float pitch = atan2(-ax, sqrt(ay * ay + az * az));

  float heading;
  if (my == 0)
  {
    heading = (mx < 0) ? PI : 0;
  }
  else
  {
    heading = atan2(mx, my);
  }

  heading -= DECLINATION * PI / 180;

  if (heading > PI) heading -= (2 * PI);
  else if (heading < -PI) heading += (2 * PI);

  if (axis=="x")
  {
    return roll;
  }
  else if (axis=="y")
  {
    return pitch;
  }

  else{
    return heading;
  }

  // Convert everything from radians to degrees:
  // heading *= 180.0 / PI;
  // pitch *= 180.0 / PI;
  // roll  *= 180.0 / PI;
  
  //return [roll, pitch, heading]; //
}
