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

ros::Publisher pub;
ros::Publisher pubStamped;
ros::Subscriber sub;
int i_velLinear;
int i_velAngular;
double max_linear_vel;
double max_angular_vel;

void callBack(const sensor_msgs::JoyConstPtr& joy)
{
  geometry_msgs::Twist vel;
  geometry_msgs::TwistStamped velStamped;

  // Only publish if RT depressed. This prevents joy from over riding other topics
	if (joy->axes[5] == -1){ 
    
    vel.angular.z = max_angular_vel*joy->axes[0];
    vel.linear.x = max_linear_vel*joy->axes[1];
    pub.publish(vel);

    
    ros::Time current_time = ros::Time::now();

    velStamped.header.stamp.sec = current_time.sec;
    velStamped.header.stamp.nsec = current_time.nsec;

    velStamped.twist = vel;
    pubStamped.publish(velStamped);
  }

}

int main(int argc, char** argv) {

  ros::init(argc, argv, "rover_joy_teleop");	//Specify node name
  
  ros::NodeHandle n;

  n.param("/joy_teleop/rover_joy_teleop/axis_linear", i_velLinear, 1);
  n.param("/joy_teleop/rover_joy_teleop/axis_angular", i_velAngular, 0);
  n.param("/joy_teleop/rover_joy_teleop/max_linear_vel", max_linear_vel, 0.2);
  //n.param("max_angular_vel", max_angular_vel, 1.5707);

  if (n.param("/joy_teleop/rover_joy_teleop/max_angular_vel", max_angular_vel, 1.5707))
    {
      ROS_INFO("Got param: %f", max_angular_vel);
    }
  else
    {
      ROS_ERROR("Failed to get param 'max_angular_vel'");
    }

  sub = n.subscribe("joy", 10, callBack);
  pub = n.advertise<geometry_msgs::Twist>("cmd_vel",1);
  pubStamped = n.advertise<geometry_msgs::TwistStamped>("cmd_vel_stamped",1);

  ros::spin();

  return 0;
}





