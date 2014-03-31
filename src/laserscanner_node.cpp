#include "ros/ros.h"
#include "pololu_driver/Servo_Command.h"
#include "uos_3dscanner/Scan.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_assembler/AssembleScans2.h"
#include <cstdlib>
#define _USE_MATH_DEFINES


#define MIN_POS ((-50.0 * (float)M_PI) / 180.0)
#define STANDBY_POS ((-20.0 * (float)M_PI) / 180.0)
#define MAX_POS ((60.0 * (float)M_PI) / 180.0)

static ros::ServiceClient client;
static ros::ServiceClient pointCloudClient;
static ros::Publisher state_pub;

bool scan(uos_3dscanner::Scan::Request  &req,
         uos_3dscanner::Scan::Response &res)
{
    std::cout << "Min Pos" << std::endl;
    pololu_driver::Servo_Command srv;

    srv.request.channel = 0;
    srv.request.angle = MIN_POS;
    srv.request.speed = 15;

    if (client.call(srv))
    {
      ROS_INFO("Movement finished");
    }
    else
    {
      ROS_ERROR("Failed to call service servo_command");
      return 1;
    }

    laser_assembler::AssembleScans2 assemble_srv;
    assemble_srv.request.begin = ros::Time::now();

    for(int i = 0; i < 1000; i++)
    {
        pololu_driver::Servo_Command srv;
        float angle;

        angle = MIN_POS+ (float)i/1000.0 * (MAX_POS-MIN_POS);

        ROS_INFO("angle: [%f]",angle);

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



    assemble_srv.request.end = ros::Time::now();

    if (pointCloudClient.call(assemble_srv))
    {
        ROS_INFO("Scan assembled");

        state_pub.publish(assemble_srv.response.cloud);
    }
    else
    {
      ROS_ERROR("Failed to call service assemble_scans");
      return 1;
    }


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
  pointCloudClient = n.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");

  // inits the publisher
  state_pub = n.advertise<sensor_msgs::PointCloud2>("uos_3dscans", 1);


  // inits the service
  ros::ServiceServer service = n.advertiseService("laserscanner_node", scan);


  ROS_INFO("laser scanner server ready to laser scan");
  ros::spin();
  return 0;
}
