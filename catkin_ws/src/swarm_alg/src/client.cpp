#include "ros/ros.h"
#include "rospy_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "client");
    if (argc != 3)
    {
      ROS_INFO("usage: client X Y");
      return 1;
    }
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("takeoff_client_node");
    rospy_tutorials::AddTwoInts srv;
    srv.request.a = atoll(argv[1]);
    srv.request.b = atoll(argv[2]);
    if (client.call(srv))
    {
      ROS_INFO("sub: %ld", (long int)srv.response.sum);
    }
    else
    {
      ROS_ERROR("Failed to call service takeoff_client_node");
      return 1;
    }

    return 0;
 }
