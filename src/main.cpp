#include <thread>
#include <iostream>
#include <chrono>
#include "servo_driver/socketcan.h"
#include "servo_driver/can_bus.hpp"
#include "servo_driver/ht_servo.h"
#include "ros/ros.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "geometry_msgs/Quaternion.h"

using namespace std;

void empty(void)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "servo_test");

    ros::NodeHandle ros_node;

    std::shared_ptr<CAN::CanBus> can_bus = std::make_shared<CAN::CanBus>("can0", 50);

    std::shared_ptr<HT_Servo> servo_1 = std::make_shared<HT_Servo>(1, 10, 170.68, can_bus, true);
    servo_1->pid.set_parameter(500, 3000, 0);
    servo_1->pid.set_limit(-8000, 8000);
    std::shared_ptr<HT_Servo> servo_2 = std::make_shared<HT_Servo>(2, 10, 334.59, can_bus, true);
    std::shared_ptr<HT_Servo> servo_3 = std::make_shared<HT_Servo>(3, 10, 240.12, can_bus, true);
    std::shared_ptr<HT_Servo> servo_4 = std::make_shared<HT_Servo>(4, 10, 334.73, can_bus, true);

    // PID_Controller servo_3_pid(3, 1.5, 0.0, -5000, 5000);
    ButterworthFilter servo_3_velocity_filter(50.0, 10.0);
    ButterworthFilter position_filter(50.0, 2.0);

    ros::Publisher robot_state_pub = ros_node.advertise<trajectory_msgs::JointTrajectory>("robot_state", 1);
    std::shared_ptr<ros::Publisher> rqt_data_pub = std::make_shared<ros::Publisher>(ros_node.advertise<geometry_msgs::Quaternion>("for_rqt_plot", 1));

    std::shared_ptr<ros::Rate> loop_rate = std::make_shared<ros::Rate>(50);

    // servo_4->position_calibration(loop_rate, rqt_data_pub, 1);
    // servo_3->position_calibration(loop_rate, rqt_data_pub, 1, 45, 2500);
    // while (!servo_3->is_reached_target() && ros::ok())
    // {
    //     servo_3->request(HT_Command::POSITION, 500);
    //     loop_rate->sleep();
    // }
    // servo_2->position_calibration(loop_rate, rqt_data_pub, 1);
    // servo_1->position_calibration(loop_rate, rqt_data_pub, 1, 45, 8000);

    // servo_1->set_position(0, 500);

    // servo_1->request(HT_Command::SET_DISABLE, 500);

    while (ros::ok())
    {
        trajectory_msgs::JointTrajectory state;
        
        double servo_3_velocity = servo_3_velocity_filter.step(servo_1->get_differential_velocity());

        // std::cout << velocity_filter << std::endl;

        // double control_target = servo_3_pid.step(5 - servo_3_velocity);

        geometry_msgs::Quaternion for_rqt_data;
        for_rqt_data.x = servo_1->get_velocity();
        for_rqt_data.y = servo_3_velocity;
        for_rqt_data.w = servo_1->get_current();
        rqt_data_pub->publish(for_rqt_data);

        auto start = std::chrono::steady_clock::now();
        servo_1->request(HT_Command::POSITION, 500);
        // servo_1->request(HT_Command::STATE, 50);
        auto end = std::chrono::steady_clock::now();

        cout << "Elapsed time in microseconds: "
            << chrono::duration_cast<chrono::microseconds>(end - start).count()
            << " µs" << endl;

        // ROS_INFO("%lf\n", position_filter.step(servo_1->get_position()));

        loop_rate->sleep();
    }

    servo_1->request(HT_Command::SET_DISABLE, 50);
    servo_2->request(HT_Command::SET_DISABLE, 50);
    servo_3->request(HT_Command::SET_DISABLE, 50);
    servo_4->request(HT_Command::SET_DISABLE, 50);
    ros::shutdown();
}

