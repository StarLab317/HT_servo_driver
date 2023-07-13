#ifndef HT_SERVO_H
#define HT_SERVO_H

#include <memory>
#include "can_bus.hpp"

enum class HT_Command
{
    POSITION = 0X2F,
    STATE = 0X40,
};

class HT_Servo: protected CAN::Receiver
{
    public:

        HT_Servo(int _id, std::shared_ptr<CAN::CanBus> _can_bus);
        void request(HT_Command command);

        double get_angle(void)
        {
            return angle;
        }

    private:

        const int id;
        const std::shared_ptr<CAN::CanBus> can_bus;

        double angle = 0;
        double angular_velocity = 0;
        double voltage = 0;
        double current = 0;
        double temperature = 0;
        uint8_t fault_code = 0;
        uint8_t operational_state = 0;

        virtual void reception_callback(const CAN::FrameStamp& frame_stamp);
};

#endif

