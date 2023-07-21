#ifndef HT_SERVO_H
#define HT_SERVO_H

#include <memory>
#include <chrono>
#include "can_bus.hpp"
#include "pid_controller.hpp"

constexpr double DEG2RAD = 0.017453293;
constexpr double RPM2RADS = 0.104719755;

enum class HT_Command
{
    POSITION = 0X2F,
    STATE = 0X40,
    SET_DISABLE = 0X50,
    SET_POWER = 0X53,
    SET_VELOCITY = 0X54,
};

struct PositionStamp
{
    public:
        double value = 0;
        std::chrono::steady_clock::time_point stamp;
};

class HT_Servo: protected CAN::Receiver
{
    public:

        HT_Servo(int _id, double _gear_ratio, double _angle_range, std::shared_ptr<CAN::CanBus> _can_bus);
        void position_calibration(std::shared_ptr<ros::Rate> loop_rate);
        void request(HT_Command command, uint16_t wait_response_ms = 0);
        void set_power(int16_t power, uint16_t wait_response_ms = 0);
        void set_velocity(double rads, uint16_t wait_response_ms = 0);

        double get_angle(void)
        {
            return angle;
        }
        double get_velocity(void)
        {
            return angular_velocity;
        }
        double get_differential_velocity(void)
        {
            return diff_angular_velocity;
        }

        PID_Controller pid = PID_Controller(3, 1.5, 0.0, -5000, 5000);
        ButterworthFilter velocity_filter = ButterworthFilter(50.0, 10.0);

    private:

        const int id;
        const double gear_ratio;
        const double angle_range;
        const std::shared_ptr<CAN::CanBus> can_bus;

        double angle_constraint_lower = 0;
        double angle_constraint_upper = 0;
        double angle_zero_bias = 0;

        bool is_responded = false;

        double angle = 0;
        double angular_velocity = 0;
        double diff_angular_velocity = 0;
        double voltage = 0;
        double current = 0;
        double temperature = 0;
        uint8_t fault_code = 0;
        uint8_t operational_state = 0;

        PositionStamp current_position;
        PositionStamp last_position;

        bool wait_response_block(uint16_t wait_response_ms);
        virtual void reception_callback(const CAN::FrameStamp& frame_stamp);
};

#endif

