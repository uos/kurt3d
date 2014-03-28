#include "ros/ros.h"
#include "pololu_driver/Servo_Command.h"
#include "uos_3dscanner/Scan.h"
#include "std_msgs/String.h"
#include <cstdlib>


#define MIN_POS -50.0
#define STANDBY_POS 5.0
#define MAX_POS 60.0

static ros::ServiceClient client;

void servoCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Scan: [%s]", msg->data.c_str());
}

bool scan(uos_3dscanner::Scan::Request  &req,
         uos_3dscanner::Scan::Response &res)
{
    std::cout << "Min Pos" << std::endl;
    pololu_driver::Servo_Command srv;

    srv.request.channel = 0;
    srv.request.angle = 0;
    srv.request.speed = 10;

    if (client.call(srv))
    {
      ROS_INFO("Movement finished");
    }
    else
    {
      ROS_ERROR("Failed to call service servo nodding");
      return 1;
    }


    for(int i = 0; i < 1000; i++)
    {
        pololu_driver::Servo_Command srv;
        float angle;

        angle = (((MAX_POS-MIN_POS)/1000.0)*(float)i);
        std::cout << "Angle: " << angle << std::endl;

        srv.request.channel = 0;
        srv.request.angle = (angle);
        srv.request.speed = 0;

        if (client.call(srv))
        {
          ROS_INFO("Movement finished");
        }
        else
        {
          ROS_ERROR("Failed to call service servo nodding");
          return 1;
        }

        ros::spinOnce();
    };



    srv.request.channel = 0;
    srv.request.angle = STANDBY_POS;
    srv.request.speed = 10;

    if (client.call(srv))
    {
      ROS_INFO("Movement finished");
    }
    else
    {
      ROS_ERROR("Failed to call service servo nodding");
      return 1;
    }




    res.finished = true;
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "laserscanner_server");


  ros::NodeHandle n;

  client = n.serviceClient<pololu_driver::Servo_Command>("servo_node");

  ros::Subscriber sub = n.subscribe("servo_state", 1, servoCallback);

  // inits the service
  ros::ServiceServer service = n.advertiseService("laserscanner_node", scan);


  ROS_INFO("laser scanner server ready to laser scan");
  ros::spin();
  return 0;
}
