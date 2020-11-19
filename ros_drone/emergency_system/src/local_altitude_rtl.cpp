#include <ros/ros.h>
#include <core_api/ParamGetGlobalNamespace.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/TwistStamped.h>
#include <core_api/Land.h>
#include <core_api/RTL.h>

std::string global_namespace;

core_api::Land land_srv;
core_api::RTL rtl_srv;
core_api::ParamGetGlobalNamespace namespace_srv;
geometry_msgs::TwistStamped altitude_date;

ros::ServiceClient land_client,rtl_client;
ros::Subscriber battery_sub, altitude_sub;


void altitude_callback(const geometry_msgs::TwistStampedConstPtr &altitude)
{

    ros::NodeHandle n;

    ros::ServiceClient namespace_client = n.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
    namespace_client.call(namespace_srv);
    global_namespace = namespace_srv.response.param_info.param_value;

    land_client   = n.serviceClient<core_api::Land>("/"+global_namespace+"/navigation/land");
    rtl_client    = n.serviceClient<core_api::RTL>("/"+global_namespace+"/navigation/rtl");

    std::cout << "\nAltitude";
    //std::cout << "\n" << altitude->twist.angular << "\n";
    std::cout << "\n" << altitude->twist.linear.z;

    float x = altitude->twist.linear.z;
    if (x < -20){
        ROS_INFO("HIGH ALTITUDE VALUE");
        ROS_INFO("RTL mode");
        //rtl_srv.request.async=false;
        rtl_client.call(rtl_srv);
        if(!rtl_srv.response.success)
        {
            ROS_INFO("Failed return to launch!");
            //TODO make land mode
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "altitude_callback");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/flytos/mavros/local_position/local", 1, altitude_callback);
    ros::spin();

    return 0;
}
