#ifndef PPC_PID_CONTROLLER_H
#define PPC_PID_CONTROLLER_H

#include "filter.hpp"

class PID_Controller
{
    public:
        PID_Controller(double _Kp, double _Ki, double _Kd, double _output_min, double _output_max,
            double _differential_filter_alpha = 1.0):
                Kp(_Kp), Ki(_Ki), Kd(_Kd), output_min(_output_min), output_max(_output_max), 
                differential_filter(_differential_filter_alpha) {}
        
        double step(double input_bias)
        {
            double differential_val = 0;
            double output = 0;

            // ANTI-Windup
            if (integral_val < output_min)
            {
                is_saturated_flag = true;
                if (input_bias > 0)
                    integral_val += input_bias * Ki;
            }
            else if (integral_val > output_max)
            {
                is_saturated_flag = true;
                if (input_bias < 0)
                    integral_val += input_bias * Ki;
            }
            else
            {
                is_saturated_flag = false;
                integral_val += input_bias * Ki;
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

        void set_parameter(double _Kp, double _Ki, double _Kd, double _output_min, double _output_max,
            double _differential_filter_alpha = 1.0)
        {
            Kp = _Kp;
            Ki = _Ki;
            Kd = _Kd;
            output_min = _output_min;
            output_max = _output_max;
            differential_filter.set_filter_alpha(_differential_filter_alpha);
        }

        void set_parameter(double _output_min, double _output_max)
        {
            output_min = _output_min;
            output_max = _output_max;
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

        double integral_val = 0;
        double last_bias = 0;

        bool is_saturated_flag = false;

        FirstOrderFilter differential_filter;
};

#endif
