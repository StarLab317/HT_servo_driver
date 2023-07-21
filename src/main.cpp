#include <thread>
#include <iostream>
#include <chrono>
#include "socketcan.h"
#include "can_bus.hpp"
#include "ht_servo.h"
#include "pid_controller.hpp"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Vector3.h"

using namespace std;

void empty(void)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HT_servo_driver");

    ros::NodeHandle ros_node;

    CAN::SocketCAN can_interface;

    std::shared_ptr<CAN::CanBus> can_bus = std::make_shared<CAN::CanBus>("can0", 50);

    std::shared_ptr<HT_Servo> servo_3 = std::make_shared<HT_Servo>(4, 10, 100, can_bus);

    PID_Controller servo_3_pid(3, 1.5, 0.0, -5000, 5000);
    ButterworthFilter servo_3_velocity_filter(50.0, 10.0);

    ros::Publisher robot_state_pub = ros_node.advertise<trajectory_msgs::JointTrajectory>("robot_state", 1);
    ros::Publisher rqt_data_pub = ros_node.advertise<geometry_msgs::Vector3>("for_rqt_plot", 1);

    ros::Rate loop_rate(50);

    

    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory state;
        
        double servo_3_velocity = servo_3_velocity_filter.step(servo_3->get_differential_velocity());

        // std::cout << velocity_filter << std::endl;

        double control_target = servo_3_pid.step(5 - servo_3_velocity);

        geometry_msgs::Vector3 for_rqt_data;
        for_rqt_data.x = servo_3->get_velocity();
        for_rqt_data.y = servo_3_velocity;
        rqt_data_pub.publish(for_rqt_data);

        auto start = std::chrono::steady_clock::now();
        servo_3->request(HT_Command::SET_DISABLE, 500);
        auto end = std::chrono::steady_clock::now();

        cout << "Elapsed time in microseconds: "
            << chrono::duration_cast<chrono::microseconds>(end - start).count()
            << " µs" << endl;

        loop_rate.sleep();
    }

    ros::shutdown();
}

