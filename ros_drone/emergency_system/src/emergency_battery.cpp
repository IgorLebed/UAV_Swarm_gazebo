#include <ros/ros.h>
#include <core_api/ParamGetGlobalNamespace.h>
#include <sensor_msgs/BatteryState.h>
#include <core_api/Land.h>

std::string global_namespace;

core_api::Land land_srv;
core_api::ParamGetGlobalNamespace namespace_srv;
sensor_msgs::BatteryState battery_date;

ros::ServiceClient land_client;
ros::Subscriber battery_sub;

void battery_callback(const sensor_msgs::BatteryStateConstPtr &battery)
{

  ros::NodeHandle n;

  ros::ServiceClient namespace_client = n.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
  namespace_client.call(namespace_srv);
  global_namespace = namespace_srv.response.param_info.param_value;

  land_client   = n.serviceClient<core_api::Land>("/"+global_namespace+"/navigation/land");

  //std::cout << "\nVoltage \tcurrent";
  //std::cout << "\n" << battery->voltage << "\t\t" << battery->current;
  std::cout << "\nVoltage";
  std::cout << "\n" << battery->voltage << "\n";

  float x = battery->voltage;
  if (x < 21.5){
    ROS_INFO("FAILED BATTERY VOLTAGE");
    ROS_INFO("Landing");
    land_srv.request.async=false;
    land_client.call(land_srv);
    if(!land_srv.response.success)
      {
      ROS_INFO("Failed to land!=(");
      }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "battery_callback");
  ros::NodeHandle n;

  //ros::ServiceClient namespace_client = n.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
  //namespace_client.call(namespace_srv);
  //global_namespace = namespace_srv.response.param_info.param_value;

  ros::Subscriber sub = n.subscribe("/flytos/mavros/battery", 1, battery_callback);
  ros::spin();

  //land_client   = n.serviceClient<core_api::Land>("/"+global_namespace+"/navigation/land");
  //battery_sub   = n.subscribe("/flytsim/mavros/battery");

  //ROS_INFO("Landing");
  //land_srv.request.async=false;
  //land_client.call(land_srv);
  //if(!land_srv.response.success)
  //{
  //  ROS_INFO("Failed to land!=(");
  //  return 1;
  //}
  //ros::spin();
  return 0;
}
