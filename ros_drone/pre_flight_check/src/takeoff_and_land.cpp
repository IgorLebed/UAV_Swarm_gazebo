//
// Created by igor on 22.10.2019.
//
#include <ros/ros.h>
#include <iostream>
#include <core_api/ParamGetGlobalNamespace.h>
#include <core_api/TakeOff.h>
#include <core_api/Land.h>

std::string global_namespace;

core_api::ParamGetGlobalNamespace namespace_srv;
core_api::TakeOff takeoff_srv;
core_api::Land land_srv;

ros::ServiceClient land_client,takeoff_client;

int main(int argc, char **argv)
{
    if(argc < 2)
    {
        std::cout<<"\nThis app expects arguments\n";
        exit(0);
    }

    float height = std::stof(argv[1]);             //Convert Argument from string to float
    ros::init(argc, argv, "takeoff_and_land");
    ros::NodeHandle nh;

    ros::ServiceClient namespace_client = nh.serviceClient<core_api::ParamGetGlobalNamespace>("/get_global_namespace");
    namespace_client.call(namespace_srv);
    global_namespace = namespace_srv.response.param_info.param_value;

    takeoff_client = nh.serviceClient<core_api::TakeOff>("/"+global_namespace+"/navigation/take_off");
    land_client    = nh.serviceClient<core_api::Land>("/"+global_namespace+"/navigation/land");

    ROS_INFO("Taking Off");
    takeoff_srv.request.takeoff_alt = height;
    takeoff_client.call(takeoff_srv);
    if(!takeoff_srv.response.success)
    {
        ROS_ERROR("Failed to takeoff");
        return 1;
    }

    ROS_INFO("Landing");
    land_srv.request.async =false;
    land_client.call(land_srv);
    if(!land_srv.response.success)
    {
        ROS_ERROR("Failed to Land!");
        return 1;
    }
    return 0;
}
