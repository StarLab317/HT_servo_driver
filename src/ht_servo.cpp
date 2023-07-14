#include "ht_servo.h"

HT_Servo::HT_Servo(int _id, int _gear_ratio, std::shared_ptr<CAN::CanBus> _can_bus):
    id(_id), gear_ratio(_gear_ratio), can_bus(_can_bus)
{
    can_bus->add_device(id, this);
}

void HT_Servo::request(HT_Command command)
{
    uint32_t can_id = static_cast<uint32_t>(command)<<4 | id;
    uint8_t data[0];
    can_bus->send(can_id, 0, data);
}

void HT_Servo::set_power(int16_t power)
{
    uint32_t can_id = static_cast<uint32_t>(HT_Command::SET_POWER)<<4 | id;
    uint8_t data[2];
    data[0] = power & 0XFF;
    data[1] = (power >> 8) & 0XFF;
    can_bus->send(can_id, 2, data);
}

void HT_Servo::set_velocity(int16_t rpm)
{
    uint32_t can_id = static_cast<uint32_t>(HT_Command::SET_VELOCITY)<<4 | id;
    uint8_t data[2];
    data[0] = rpm & 0XFF;
    data[1] = (rpm >> 8) & 0XFF;
    can_bus->send(can_id, 2, data);
}

void HT_Servo::reception_callback(const CAN::FrameStamp& frame_stamp)
{
    if ((frame_stamp.frame.can_id >> 4) == static_cast<uint32_t>(HT_Command::POSITION))
    {
        angle = (frame_stamp.frame.data[5] << 24 |
                 frame_stamp.frame.data[4] << 16 |
                 frame_stamp.frame.data[3] << 8 |
                 frame_stamp.frame.data[2]) * 360.0 / 16384.0 / gear_ratio;
        angular_velocity = (frame_stamp.frame.data[7] << 8 | frame_stamp.frame.data[6]) * 0.1;
    }
}


