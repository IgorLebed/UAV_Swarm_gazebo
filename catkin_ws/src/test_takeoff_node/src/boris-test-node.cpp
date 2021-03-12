#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <thread>
#include <mutex>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
std::mutex mu;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void changePose(int x, int y, int z)
{
    mu.lock();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    mu.unlock();
}

void posePublisher(ros::Rate rate)
{
    ros::NodeHandle nh;
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    while (ros::ok())
    {
        mu.lock();
        local_pos_pub.publish(pose);
        mu.unlock();
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node_boris");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    changePose(0, 0, 0);
    std::thread posePub(posePublisher, rate);

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    int xy = 2;
    ros::Time changeXY = ros::Time::now();
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
            }
        }

        // СМЕНА ПОЗИЦИИ
        if (ros::Time::now() - changeXY > ros::Duration(5.0))
        {
            changePose(xy, xy, 2);
            xy *= -1;
            changeXY = ros::Time::now();
        }

        ros::spinOnce();
        rate.sleep();
    }
    if (posePub.joinable())
    {
        posePub.join();
    }
    return 0;
}