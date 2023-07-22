#include <thread>
#include <iostream>
#include <chrono>
#include "socketcan.h"
#include "can_bus.hpp"
#include "ht_servo.h"
#include "pid_controller.hpp"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

void empty(void)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HT_servo_driver");

    ros::NodeHandle ros_node;

    std::shared_ptr<CAN::CanBus> can_bus = std::make_shared<CAN::CanBus>("can0", 50);

    std::shared_ptr<HT_Servo> servo_3 = std::make_shared<HT_Servo>(3, 10, 240.12, can_bus);

    PID_Controller servo_3_pid(3, 1.5, 0.0, -5000, 5000);
    ButterworthFilter servo_3_velocity_filter(50.0, 10.0);
    ButterworthFilter position_filter(50.0, 2.0);

    ros::Publisher robot_state_pub = ros_node.advertise<trajectory_msgs::JointTrajectory>("robot_state", 1);
    std::shared_ptr<ros::Publisher> rqt_data_pub = std::make_shared<ros::Publisher>(ros_node.advertise<geometry_msgs::Quaternion>("for_rqt_plot", 1));

    std::shared_ptr<ros::Rate> loop_rate = std::make_shared<ros::Rate>(100);

    // servo_3->position_calibration(loop_rate, rqt_data_pub);

    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory state;
        
        double servo_3_velocity = servo_3_velocity_filter.step(servo_3->get_differential_velocity());

        // std::cout << velocity_filter << std::endl;

        double control_target = servo_3_pid.step(5 - servo_3_velocity);

        geometry_msgs::Quaternion for_rqt_data;
        for_rqt_data.x = servo_3->get_velocity();
        for_rqt_data.y = servo_3_velocity;
        rqt_data_pub->publish(for_rqt_data);

        auto start = std::chrono::steady_clock::now();
        servo_3->request(HT_Command::POSITION, 500);
        auto end = std::chrono::steady_clock::now();

        // cout << "Elapsed time in microseconds: "
        //     << chrono::duration_cast<chrono::microseconds>(end - start).count()
        //     << " µs" << endl;

        ROS_INFO("%lf\n", position_filter.step(servo_3->get_position()));

        loop_rate->sleep();
        loop_rate->sleep();
    }

    servo_3->request(HT_Command::SET_DISABLE, 500);
    ros::shutdown();
}

