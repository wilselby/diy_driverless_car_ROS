
/* -----------------------------------------------------------------------------
 * Example .ino file for arduino communication with ROS for the
 * L298N motor controller board
 * Command signals expected to be between -255 and +255 with motors stopping at 0
 *----------------------------------------------------------------------------*/

#include <ros.h>
#include <geometry_msgs/Twist.h>

#define ENA 5
#define ENB 11
#define IN1 6
#define IN2 7
#define IN3 8
#define IN4 9

float L = 0.1; //distance between wheenls

ros::NodeHandle  nh;

void cmdVelCB( const geometry_msgs::Twist& twist)
{
  int gain = 700;
  float left_wheel_data = gain*(twist.linear.x + twist.angular.z*L);
  float right_wheel_data = gain*(twist.linear.x - twist.angular.z*L);
  
  if (left_wheel_data >= 0)
  {
    analogWrite(ENA, abs(left_wheel_data));
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    analogWrite(ENA, abs(left_wheel_data));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (right_wheel_data >= 0)
  {
    analogWrite(ENB, abs(right_wheel_data));
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else
  {
    analogWrite(ENB, abs(right_wheel_data));
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
}

ros::Subscriber<geometry_msgs::Twist> subCmdVel("/rover_velocity_controller/cmd_vel", cmdVelCB);
  
void setup()
{
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);  
    
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
    
  nh.initNode();
  nh.subscribe(subCmdVel);
    
 }
  
 void loop()
 {
  nh.spinOnce();
 }
