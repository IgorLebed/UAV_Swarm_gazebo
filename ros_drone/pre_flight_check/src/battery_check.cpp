#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>

//sensor_msgs::BatteryState battery_date;

void battery_callback(const sensor_msgs::BatteryStateConstPtr &battery)
{
  std::cout << "\nVoltage \tcurrent";
  std::cout << "\n" << battery->voltage << "\t\t" << battery->current;
  //float x = battery->voltage;
  //if (x < 20){
  //  ROS_INFO("FAIL BATTERY");
  //}
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_callback");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/flytos/mavros/battery", 1000, battery_callback);
  ros::spin();
  return 0;
}
