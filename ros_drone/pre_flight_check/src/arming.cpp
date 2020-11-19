//
// Created by igor on 22.10.2019.
//
#include <ros/ros.h>
#include <iostream>
#include <core_api/ParamGetGlobalNamespace.h>
#include <core_api/TakeOff.h>
#include <core_api/Land.h>
#include <core_api/Arm.h>

std::string global_namespace;

core_api::ParamGetGlobalNamespace namespace_srv;
//core_api::TakeOff takeoff_srv;
//core_api::Land land_srv;
core_api::Arm arm_srv;

ros::ServiceClient land_client,takeoff_client, arm_client;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_disarm");
    ros::NodeHandle nh;

    ros::ServiceClient namespace_client = nh.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
    namespace_client.call(namespace_srv);
    global_namespace = namespace_srv.response.param_info.param_value;

    //takeoff_client = nh.serviceClient<core_api::TakeOff>("/"+global_namespace+"/navigation/take_off");
    //land_client    = nh.serviceClient<core_api::Land>("/"+global_namespace+"/navigation/land");
    arm_client     = nh.serviceClient<core_api::Arm>("/"+global_namespace+"/navigation/arm");

    ROS_INFO("Arming");
    //land_srv.request.async =false;
    arm_client.call(arm_srv);
    if(!arm_srv.response.success)
    {
        ROS_ERROR("Failed to Arm!");
        return 1;
    }
    return 0;
}
