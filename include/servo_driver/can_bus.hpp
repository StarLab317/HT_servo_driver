#ifndef CAN_BUS_HPP
#define CAN_BUS_HPP

#include <vector>
#include <memory>
#include <mutex>
#include <chrono>
#include "servo_driver/socketcan.h"
#include "ros/ros.h"

namespace CAN
{

#define DEVICE_ADDRESS_MASK 0X0F
#define DEVICE_ADDRESS_MAX 0XFFFF

struct FrameStamp
{
    public:
        can_frame frame;
        std::chrono::steady_clock::time_point stamp;
};

class Receiver
{
    public:
        virtual void reception_callback(const FrameStamp& frame_stamp) = 0;
};

class CanBus
{
    public:

        CanBus(const std::string& bus_name, int thread_priority)
        {
            // Initialize device at can_device, false for no loop back.
            while (!socket_can.open(bus_name, std::bind(&CanBus::frame_callback, this, std::placeholders::_1), thread_priority) && ros::ok())
                ros::Duration(.5).sleep();
            ROS_INFO("Successfully connected to %s.", bus_name.c_str());
        }
        ~CanBus()
        {
            device_list.clear();
        }

        void add_device(int device_id, Receiver* device)
        {
            int list_size = device_list.size();
            int map_size = device_list_map.size();
            if (device_id - map_size == 1)
            {
                device_list_map.push_back(list_size);
            }
            else if (device_id - map_size > 1)
            {
                device_list_map.resize(device_id);
                device_list_map[device_id - 1] = list_size;
            }
            else
            {
                device_list_map[device_id - 1] = list_size;
            }
            device_list.push_back(device);
        }

        void send(uint32_t can_id, uint8_t can_dlc, uint8_t* data)
        {
            can_frame frame;

            if (can_dlc > CAN_MAX_DLEN)
                can_dlc = CAN_MAX_DLEN;
            frame.can_id = can_id;
            frame.can_dlc = can_dlc;
            frame.__pad = 0;
            frame.__res0 = 0;
            frame.__res1 = 0;
            memcpy(frame.data, data, can_dlc*sizeof(uint8_t));
            socket_can.write(&frame);
        }

    private:

        /** \brief This function will be called when CAN bus receive message. It push frame which received into a vector: read_buffer_.
         *
         * @param frame The frame which socketcan receive.
         */
        void frame_callback(const can_frame& frame)
        {
            std::lock_guard<std::mutex> guard(mutex_);
            uint8_t device_id = frame.can_id & DEVICE_ADDRESS_MASK;
            FrameStamp can_frame_stamp{ .frame = frame, .stamp = std::chrono::steady_clock::now() };
            device_list[device_list_map[device_id - 1]]->reception_callback(can_frame_stamp);  // 调用对应地址的设备的回调函数
        }

        SocketCAN socket_can;
        mutable std::mutex mutex_;
        std::vector<uint16_t> device_list_map;
        std::vector<Receiver*> device_list;
};

}  // namespace CAN

#endif




