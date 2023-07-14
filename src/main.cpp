#include <thread>
#include <iostream>
#include "socketcan.h"
#include "can_bus.hpp"
#include "ht_servo.h"
#include "ros/ros.h"

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

    // can_interface.open("can0", std::bind(&empty), 1);

    ros::Rate loop_rate(30);

    while (ros::ok())
    {
        test1.request(HT_Command::POSITION);
        loop_rate.sleep();
        std::cout << test1.get_angle() << std::endl;
        loop_rate.sleep();
    }

    ros::shutdown();
}

