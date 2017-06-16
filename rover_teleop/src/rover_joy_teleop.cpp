/*
 * Copyright 2017 Wil Selby www.wilselby.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include<ros/ros.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/TwistStamped.h>
#include<sensor_msgs/Joy.h>
#include<iostream>

using namespace std;
float max_linear_vel = 0.2;
float max_angular_vel = 1.5707;

class TeleopJoy{
  public:
    TeleopJoy();

  private:
    void callBack(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher pubStamped;
    ros::Subscriber sub;
    int i_velLinear;
    int i_velAngular;
};

TeleopJoy::TeleopJoy()
{

  i_velLinear = 1;
  i_velAngular = 0;
  n.param("axis_linear", i_velLinear, i_velLinear);
  n.param("axis_angular", i_velAngular, i_velAngular);
  sub = n.subscribe<sensor_msgs::Joy>("/joy_teleop/joy", 10, &TeleopJoy::callBack, this);
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  pubStamped = n.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);
  
  while(true)
  {

  }

}


void TeleopJoy::callBack(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  vel.angular.z = max_angular_vel*joy->axes[0];
  vel.linear.x = max_linear_vel*joy->axes[1];
  pub.publish(vel);

  geometry_msgs::TwistStamped velStamped;
  ros::Time current_time = ros::Time::now();

  velStamped.header.stamp.sec = current_time.sec;
  velStamped.header.stamp.nsec = current_time.nsec;

  velStamped.twist = vel;
  pubStamped.publish(velStamped);

}


int main(int argc, char** argv) {

  ros::init(argc, argv, "rover_joy_teleop");	//Specify node name
  TeleopJoy teleopjoy;

  ros::spin();

  return 0;
}
