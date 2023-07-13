#include "ht_servo.h"

HT_Servo::HT_Servo(int _id, std::shared_ptr<CAN::CanBus> _can_bus):
    id(_id), can_bus(_can_bus)
{
    can_bus->add_device(this);
}

void HT_Servo::inquire_state(void)
{
    uint32_t can_id = static_cast<uint32_t>(HT_Command::STATE)<<4 | id;
    uint8_t data[0];
    can_bus->send(can_id, 0, data);
}

void HT_Servo::reception_callback(const CAN::CanFrameStamp& frame_stamp)
{
    if ((frame_stamp.frame.can_id >> 4) == static_cast<uint32_t>(HT_Command::POSITION))
    {
        angle = (frame_stamp.frame.data[5] << 24 |
                 frame_stamp.frame.data[4] << 16 |
                 frame_stamp.frame.data[3] << 8 |
                 frame_stamp.frame.data[2]) * 360 / 0X4000;
        angular_velocity = (frame_stamp.frame.data[7] << 8 | frame_stamp.frame.data[6]) * 0.1;
    }
}


