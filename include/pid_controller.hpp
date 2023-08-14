#ifndef PPC_PID_CONTROLLER_H
#define PPC_PID_CONTROLLER_H

#include <chrono>
#include "filter.hpp"

class PID_Controller
{
    public:
        PID_Controller(double _Kp, double _Ki, double _Kd, double _output_min, double _output_max,
            double _frequency, double _differential_filter_alpha = 1.0):
                Kp(_Kp), Ki(_Ki), Kd(_Kd), output_min(_output_min), output_max(_output_max), 
                sample_time(1.0 / _frequency), differential_filter(_differential_filter_alpha) {}
        
        double step(const double input_bias, const std::chrono::steady_clock::time_point& time_stamp)
        {
            double differential_val = 0;
            double output = 0;

            double duration = abs(std::chrono::duration_cast<std::chrono::nanoseconds>(time_stamp - last_stamp).count() / 1e9);
            last_stamp = time_stamp;
            if (duration > sample_time * 2.0)
            {
                duration = sample_time * 2.0;
                std::cout << "[WARNING] Call duration does not match the sampling time" << std::endl;
            }

            // ANTI-Windup
            if (integral_val <= output_min)
            {
                is_saturated_flag = true;
                integral_val = output_min;  // 积分限幅
                if (input_bias > 0)
                    integral_val += input_bias * Ki * duration;
            }
            else if (integral_val >= output_max)
            {
                is_saturated_flag = true;
                integral_val = output_max;  // 积分限幅
                if (input_bias < 0)
                    integral_val += input_bias * Ki * duration;
            }
            else
            {
                is_saturated_flag = false;
                integral_val += input_bias * Ki * duration;
            }
            
            differential_val = differential_filter.step(input_bias - last_bias);
            last_bias = input_bias;

            output = Kp * input_bias + integral_val + Kd * differential_val;

            if (output < output_min)
                output = output_min;
            else if (output > output_max)
                output = output_max;

            return output;
        }

        bool is_saturated(void)
        {
            return is_saturated_flag;
        }

        void set_parameter(double _Kp, double _Ki, double _Kd, double _differential_filter_alpha = 1.0)
        {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
            differential_filter.set_filter_alpha(_differential_filter_alpha);
        }

        void set_limit(double _output_min, double _output_max)
        {
            output_min = _output_min;
            output_max = _output_max;
        }

        void set_frequency(double _frequency)
        {
            sample_time = 1.0 / _frequency;
        }

        void reset(void)
        {
            integral_val = 0;
            last_bias = 0;
            is_saturated_flag = false;
            differential_filter.reset();
        }

    private:
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;
        double output_min = 0;
        double output_max = 0;
        double sample_time = 0;

        double integral_val = 0;
        double last_bias = 0;

        std::chrono::steady_clock::time_point last_stamp = std::chrono::steady_clock::now();

        bool is_saturated_flag = false;

        FirstOrderFilter differential_filter;
};

#endif
