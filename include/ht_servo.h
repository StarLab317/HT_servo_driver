#ifndef HT_SERVO_H
#define HT_SERVO_H

#include <memory>
#include "can_bus.hpp"

enum class HT_Command
{
    POSITION = 0X2F,
    STATE = 0X40,
    SET_POWER = 0X53,
    SET_VELOCITY = 0X54,
};

class HT_Servo: protected CAN::Receiver
{
    public:

        HT_Servo(int _id, int _gear_ratio, std::shared_ptr<CAN::CanBus> _can_bus);
        void request(HT_Command command);
        void set_power(int16_t power, uint16_t wait_response_ms = 0);
        void set_velocity(int16_t rpm, uint16_t wait_response_ms = 0);

        double get_angle(void)
        {
            return angle;
        }
        double get_velocity(void)
        {
            return angular_velocity;
        }

    private:

        const int id;
        const int gear_ratio;
        const std::shared_ptr<CAN::CanBus> can_bus;

        bool is_responded = false;

        double angle = 0;
        double angular_velocity = 0;
        double voltage = 0;
        double current = 0;
        double temperature = 0;
        uint8_t fault_code = 0;
        uint8_t operational_state = 0;

        bool wait_response_block(uint16_t wait_response_ms);
        virtual void reception_callback(const CAN::FrameStamp& frame_stamp);
};

#endif

