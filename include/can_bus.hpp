#ifndef CAN_BUS_HPP
#define CAN_BUS_HPP

#include <vector>
#include <memory>
#include <mutex>
#include "socketcan.h"
#include "ros/ros.h"

namespace CAN
{

#define ADDRESS_MASK 0X0F

struct CanFrameStamp
{
    can_frame frame;
    ros::Time stamp;
};

class Receiver
{
    public:
        virtual void reception_callback(const CAN::CanFrameStamp& frame_stamp) = 0;
};

class CanBus
{
    public:

        CanBus(const std::string& bus_name)
        {
            
        }
        ~CanBus()
        {
            device_list.clear();
        }

        void add_device(Receiver* device)
        {
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
            uint8_t device_id = frame.can_id & ADDRESS_MASK;
            if (device_id < device_list.size())
            {
                CanFrameStamp can_frame_stamp{ .frame = frame, .stamp = ros::Time::now() };
                device_list[device_id - 1]->reception_callback(can_frame_stamp);  // 调用对应地址的设备的回调函数
            }
        }

        SocketCAN socket_can;
        mutable std::mutex mutex_;
        std::vector<Receiver*> device_list;
};

}  // namespace CAN

#endif




