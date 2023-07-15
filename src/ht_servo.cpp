#include "ht_servo.h"
#include <chrono>

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

void HT_Servo::set_power(int16_t power, uint16_t wait_response_ms)
{
    uint32_t can_id = static_cast<uint32_t>(HT_Command::SET_POWER)<<4 | id;
    uint8_t data[2];
    data[0] = power & 0XFF;
    data[1] = (power >> 8) & 0XFF;
    can_bus->send(can_id, 2, data);

    if (wait_response_ms != 0)
        wait_response_block(wait_response_ms);
}

void HT_Servo::set_velocity(int16_t rpm, uint16_t wait_response_ms)
{
    uint32_t can_id = static_cast<uint32_t>(HT_Command::SET_VELOCITY)<<4 | id;
    uint8_t data[2];
    data[0] = rpm & 0XFF;
    data[1] = (rpm >> 8) & 0XFF;
    can_bus->send(can_id, 2, data);

    if (wait_response_ms != 0)
        wait_response_block(wait_response_ms);
}

bool HT_Servo::wait_response_block(uint16_t wait_response_ms)
{
    is_responded = false;
    auto start = std::chrono::steady_clock::now();
    auto end = std::chrono::steady_clock::now();
    while (!is_responded && 
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() < wait_response_ms)
    {
        end = std::chrono::steady_clock::now();
    }

    return is_responded;
}

void HT_Servo::reception_callback(const CAN::FrameStamp& frame_stamp)
{
    HT_Command received_command = static_cast<HT_Command>(frame_stamp.frame.can_id >> 4);

    if (HT_Command::POSITION == received_command ||
        HT_Command::SET_POWER == received_command ||
        HT_Command::SET_VELOCITY == received_command)
    {
        angle = (frame_stamp.frame.data[5] << 24 |
                 frame_stamp.frame.data[4] << 16 |
                 frame_stamp.frame.data[3] << 8 |
                 frame_stamp.frame.data[2]) * 360.0 / 16384.0 / gear_ratio;
        // 单位 rad/s
        angular_velocity = static_cast<int16_t>(frame_stamp.frame.data[7] << 8 | frame_stamp.frame.data[6]) * 0.1 * 0.10472;

        is_responded = true;
    }
}


