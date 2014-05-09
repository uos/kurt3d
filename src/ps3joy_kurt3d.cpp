/*
 * Copyright (c) 2014, Osnabrueck University
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * * Neither the name of the Osnabrueck University nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include "kurt3d/Scan.h"
#include <sensor_msgs/JointState.h>

#define RAD(GRAD) ((GRAD * (float)M_PI) / (float)180)

double max_vel_x, max_rotational_vel;
ros::Publisher vel_pub;
double speed_multiplier;
static ros::ServiceClient client;
ros::Publisher servo_pub;

static double min_pos[5] =
{
    RAD(-50.0),//fertig
    RAD(-65.0),//fertig
    RAD(-65.0),//fertig
    RAD(-50.0),//fertig
    RAD(-65.0) //fertig
};
static double max_pos[5] =
{
    RAD(60.0),
    RAD(45.0),
    RAD(45.0),
    RAD(60.0),
    RAD(45.0)
};

void ps3joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  geometry_msgs::Twist vel;
  if (joy->buttons[8] == 1)   // check for full-speed button
  {
    speed_multiplier = 1.0;
  }
  else if (true)  // check if right analog stick was used to scale speed
  {
    speed_multiplier = 0.5 + (0.5 * joy->axes[3]);  // stick full front -> speed_multiplier = 1.0 , full back -> 0.0
  }
  else  // else use half max speed
  {
    speed_multiplier = 0.5;
  }

  // check if cross is used for steering
  if (joy->buttons[4] || joy->buttons[5] || 
      joy->buttons[6] || joy->buttons[7])
  {
    // note that every button of the cross (axes 4-7) generates 
    // an output in [-1.0, 0.0]
    vel.linear.x = max_vel_x * (joy->axes[4] * -1.0 + joy->axes[6]) * speed_multiplier;
    vel.angular.z = max_rotational_vel * (joy->axes[7] * -1.0 + joy->axes[5]) * speed_multiplier;
  }
  // check if triangle is used for laser scanning
  if (joy->buttons[12])
  {
     kurt3d::Scan srv;

     if (client.call(srv))
     {
       ROS_INFO("Scan finished");
     }
     else
     {
       ROS_ERROR("Failed to call laserscanner_node service");
     }

  }
  if(joy->buttons[10])
  {
    sensor_msgs::JointState msg;
    msg.name.resize(2);
    msg.position.resize(2);
    msg.velocity.resize(2);
    msg.effort.resize(2);
    msg.name[0] = "servo_1_b_to_wing_1_b";
    msg.name[1] = "servo_1_a_to_wing_1_a";

    msg.position[0] = (((joy->axes[2]*-1.0+1.0) / 2.0) * (max_pos[2]-min_pos[2])) + min_pos[2];
    msg.position[1] = (((joy->axes[3]*-1.0+1.0) / 2.0) * (max_pos[1]-min_pos[1])) + min_pos[1];

    servo_pub.publish(msg);
  }
  if(joy->buttons[11])
  {
    sensor_msgs::JointState msg;
    msg.name.resize(2);
    msg.position.resize(2);
    msg.velocity.resize(2);
    msg.effort.resize(2);
    msg.name[0] = "servo_2_b_to_wing_2_b";
    msg.name[1] = "servo_2_a_to_wing_2_a";

    msg.position[0] = (((joy->axes[2]*-1+1.0) / 2.0) * (max_pos[4]-min_pos[4])) + min_pos[4];
    msg.position[1] = (((joy->axes[3]+1.0) / 2.0) * (max_pos[3]-min_pos[3])) + min_pos[3];

    servo_pub.publish(msg);
  }
  else  // use left analog stick
  {
    vel.linear.x = max_vel_x * joy->axes[1] * speed_multiplier;
    vel.angular.z = max_rotational_vel * joy->axes[0] * speed_multiplier;
  }
  vel_pub.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ps3joy_kurt3d");

  ros::NodeHandle nh;
  ros::NodeHandle nh_ns("~");

  nh_ns.param("max_vel_x", max_vel_x, 1.5);
  nh_ns.param("max_rotational_vel", max_rotational_vel, 3.0);

  vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber ps3joy_sub = nh.subscribe("joy", 10, ps3joyCallback);

  client = nh.serviceClient<kurt3d::Scan>("laserscanner_node");

  servo_pub = nh.advertise<sensor_msgs::JointState>("servo_control", 1);

  ros::spin();
}

