#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    int radius = 2;
    const double PI = 3.141592653589793;

    geometry_msgs::PoseStamped start_pose;
    geometry_msgs::PoseStamped pose[32];

    start_pose.pose.position.x = 0;
    start_pose.pose.position.y = 0;
    start_pose.pose.position.z = 2;
    for (int i = 0; i < 32; i++)
    {
        double cosinus = cos(PI / 16 * i);
        double sinus = sin(PI / 16 * i);
        pose[i].pose.position.x = radius * cosinus - radius;
        pose[i].pose.position.y = radius * sinus;
        pose[i].pose.position.z = 2;
    }

    for (int i = 0; ros::ok() && i < 32; i++)
    {
        local_pos_pub.publish(pose[i]);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
                break;
            }
        }

        local_pos_pub.publish(start_pose);

        ros::spinOnce();
        rate.sleep();
    }
    for (double i = 0; ros::ok(); i += 0.2)
    {
        if (int(i) == 32)
        {
            i = 0;
            ROS_INFO("CIRCLE");
        }
        local_pos_pub.publish(pose[int(i)]);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}