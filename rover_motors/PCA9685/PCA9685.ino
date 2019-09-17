

/* -----------------------------------------------------------------------------
 * Example .ino file for arduino communication with ROS for the
 * PCA9685 motor controller board
 *----------------------------------------------------------------------------*/
#include <ros.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

ros::NodeHandle  nh;

int reverse = 0;

float throttle = 0; 
float angle = 0; 

float x_range = .2 - (-.2);
float y_range = 1.57 - (-1.57);

float a_range = 400 - 275;
float t_range = 360 - 300;

void cmdAckCB( const ackermann_msgs::AckermannDriveStamped& ack)
{
 
  throttle = ((ack.drive.speed - (-.2)) / (x_range / t_range)) + 300;
  angle = ((ack.drive.steering_angle - (-1)) / (y_range / a_range)) + 275;

  // Handle reverse
  if (ack.drive.speed <= 0 && reverse == 0)
  {
     pwm.setPWM(0, 0, 300);
     delay(200);
     pwm.setPWM(0, 0, 350);
     delay(200);
     pwm.setPWM(0, 0, throttle);
     //delay(2000);
     reverse = 1;
  }
  else if (ack.drive.speed <= 0 && reverse > 0)
  {
    pwm.setPWM(0, 0, throttle);
  }
  else if (ack.drive.speed > 0){
    pwm.setPWM(0, 0, throttle);
    reverse = 0;
  }

  pwm.setPWM(1, 0, angle );

  Serial.println(throttle);

}

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  //float throttle = 0; //twist.linear.x;
  //float angle = 0; //twist.angular.z;

  //float x_range = .2 - (-.2);
  //float y_range = 1.57 - (-1.57);

  //float a_range = 400 - 275;
  //float t_range = 400 - 300;  

  throttle = ((twist.linear.x - (-.2)) / (x_range / t_range)) + 300; 
  angle = ((twist.angular.z - (-1)) / (y_range / a_range)) + 275; 

  //Reverse on RC cars is a little tricky because the ESC must receive a reverse pulse, zero pulse, reverse pulse to start to go backwards. 
  if (twist.linear.x <= 0 && reverse == 0)
  {
     pwm.setPWM(0, 0, 300);
     delay(200);
     pwm.setPWM(0, 0, 350);
     delay(200);
     pwm.setPWM(0, 0, throttle);
     //delay(2000);
     reverse = 1;
  }
  else if (twist.linear.x <= 0 && reverse > 0)
  {
    pwm.setPWM(0, 0, throttle);
  }
  else if (twist.linear.x > 0){
    pwm.setPWM(0, 0, throttle);
    reverse = 0;
  }  

  pwm.setPWM(1, 0, angle );

  Serial.println(throttle);
  
}


ros::Subscriber<geometry_msgs::Twist> subCmdVel("/rover_velocity_controller/cmd_vel", cmdVelCB);

ros::Subscriber<ackermann_msgs::AckermannDriveStamped> subAck("/racecar/ackermann_cmd_mux/output", cmdAckCB);


void setup() {
  Serial.begin(9600);

  pwm.begin();

  pwm.setPWMFreq(60);  // 1600 is the maximum PWM frequency

  // if you want to really speed stuff up, you can go into 'fast 400khz I2C' mode
  // some i2c devices dont like this so much so if you're sharing the bus, watch
  // out for this!
  //Wire.setClock(400000);

  nh.initNode();
  nh.subscribe(subCmdVel);
  nh.subscribe(subAck);

  // send zero pulse to calibrate the ESC
  pwm.setPWM(0, 0, 350 );
  reverse = 0;

}


void loop() {

  nh.spinOnce();
}
