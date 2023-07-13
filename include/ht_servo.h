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
        void inquire_state(void);
        void inquire_position(void);

    private:

        const int id;
        const std::shared_ptr<CAN::CanBus> can_bus;

        double angle;
        double angular_velocity;
        double voltage;
        double current;
        double temperature;
        uint8_t fault_code;
        uint8_t operational_state;

        virtual void reception_callback(const CAN::CanFrameStamp& frame_stamp);
};

#endif

