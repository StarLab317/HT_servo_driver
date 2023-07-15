#include <thread>
#include <iostream>
#include "socketcan.h"
#include "can_bus.hpp"
#include "ht_servo.h"
#include "pid_controller.hpp"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Vector3.h"

void empty(void)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HT_servo_driver");

    ros::NodeHandle ros_node;

    CAN::SocketCAN can_interface;

    auto can_bus = std::make_shared<CAN::CanBus>("can0", 50);

    HT_Servo test1(1, 10, can_bus);

    PID_Controller servo_1_pid(0.1, 0.1, 0, -5000, 5000);
    ButterworthFilter servo_1_velocity_filter(50.0, 2.0);

    // can_interface.open("can0", std::bind(&empty), 1);

    ros::Publisher robot_state_pub = ros_node.advertise<trajectory_msgs::JointTrajectory>("robot_state", 1);
    ros::Publisher rqt_data_pub = ros_node.advertise<geometry_msgs::Vector3>("for_rqt_plot", 1);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory state;
        state.points.push_back(trajectory_msgs::JointTrajectoryPoint());
        state.points[0].positions.push_back(test1.get_angle());
        double velocity_filter = servo_1_velocity_filter.step(test1.get_velocity());
        state.points[0].velocities.push_back(velocity_filter);
        robot_state_pub.publish(state);

        geometry_msgs::Vector3 for_rqt_data;
        for_rqt_data.x = velocity_filter;
        rqt_data_pub.publish(for_rqt_data);

        std::cout << velocity_filter << std::endl;

        

        test1.request(HT_Command::POSITION);
        loop_rate.sleep();
    }

    ros::shutdown();
}

