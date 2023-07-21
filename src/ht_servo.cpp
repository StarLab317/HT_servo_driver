#include "ht_servo.h"
#include <chrono>

HT_Servo::HT_Servo(int _id, double _gear_ratio, double _angle_range, std::shared_ptr<CAN::CanBus> _can_bus):
    id(_id), gear_ratio(_gear_ratio), angle_range(_angle_range), can_bus(_can_bus)
{
    can_bus->add_device(id, this);
}

void HT_Servo::position_calibration(std::shared_ptr<ros::Rate> loop_rate)
{
    request(HT_Command::POSITION, 500);
    loop_rate->sleep();

    double control_val = 0;
    while (ros::ok())
    {
        control_val = pid.step(1 - diff_angular_velocity);
        if (pid.is_saturated())
        {
            request(HT_Command::POSITION, 500);
            angle_zero_bias = angle;
        }
        else
        {
            set_power(control_val, 500);
        }
        loop_rate->sleep();
    }
}

void HT_Servo::request(HT_Command command, uint16_t wait_response_ms)
{
    uint32_t can_id = static_cast<uint32_t>(command)<<4 | id;
    uint8_t data[0];
    can_bus->send(can_id, 0, data);

    if (wait_response_ms != 0)
        wait_response_block(wait_response_ms);
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

void HT_Servo::set_velocity(double rads, uint16_t wait_response_ms)
{
    int16_t rpm = static_cast<int16_t>((rads / RPM2RADS) * 10.0 * gear_ratio);
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
        angular_velocity = static_cast<int16_t>(frame_stamp.frame.data[7] << 8 | frame_stamp.frame.data[6]) * 0.1 * RPM2RADS / gear_ratio;

        // 后向差分计算速度
        current_position.value = angle;
        current_position.stamp = frame_stamp.stamp;
        double duration_s = std::chrono::duration_cast<std::chrono::nanoseconds>(current_position.stamp - last_position.stamp).count() / 1e9;
        diff_angular_velocity = (current_position.value - last_position.value) / duration_s * DEG2RAD;
        last_position = current_position;

        diff_angular_velocity = velocity_filter.step(diff_angular_velocity);

        is_responded = true;
    }
}


