#include "socketcan.h"
#include "ros/ros.h"

void empty(void)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "HT_servo_driver");

    ros::NodeHandle ros_node;

    SocketCAN can_interface;

    can_interface.open("can0", std::bind(&empty), 1);

    while (ros::ok())
    {

    }
}
