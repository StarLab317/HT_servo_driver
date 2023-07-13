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

    auto can_bus = std::make_shared<CAN::CanBus>("can0");

    HT_Servo test1(1, can_bus);

    // can_interface.open("can0", std::bind(&empty), 1);

    while (ros::ok())
    {

    }
}

