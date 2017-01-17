#include <ros/ros.h>
#include "communication_node/comm_msg.h"

void receiveStrategyMessage(const communication_node::comm_msg& toggle_msg){
    int size = sizeof(toggle_msg.MotorA);
    ROS_ERROR("MotorA = %d,", toggle_msg.MotorA);
    ROS_ERROR("MotorB = %d,", toggle_msg.MotorB);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simulator_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    ros::Subscriber strategy_subcriber = n.subscribe("dummy_topic", 1, receiveStrategyMessage);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }    
    return 0;
}