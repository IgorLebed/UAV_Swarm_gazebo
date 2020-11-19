// %Tag(FULLTEXT)%
#include <ros/ros.h>
//#include "std_msgs/String.h"
#include <sensor_msgs/NavSatFix.h>

sensor_msgs::NavSatFix gpos_data;

// %Tag(CALLBACK)%
void position_callback(const sensor_msgs::NavSatFixConstPtr &gpos)
{
  std::cout << "\nlatitude \taltitude";
  std::cout << "\n" << gpos->latitude << "\t\t" << gpos->altitude;
  //ROS_INFO("I heard: [%d]", gpos->latitude);
}
// %EndTag(CALLBACK)%

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
// %Tag(SUBSCRIBER)%
  ros::Subscriber sub = n.subscribe("/flytos/mavros/global_position/global", 1000, position_callback);
// %EndTag(SUBSCRIBER)%
  ros::spin();
  return 0;
}
// %EndTag(FULLTEXT)%
