#ifndef PPC_PID_CONTROLLER_H
#define PPC_PID_CONTROLLER_H

#include <chrono>
#include "servo_driver/filter.hpp"
#include "servo_driver/time_manager.hpp"

class _P_ModuleBase
{
    public:
        _P_ModuleBase(const double Kp): _Kp(Kp) {}
        double step(const double input_bias) { return _Kp * input_bias; }
        void set_Kp(const double Kp) { _Kp = Kp; }

    private:
        double _Kp;
};

class _I_ModuleBase
{
    public:
        double _output_min;
        double _output_max;

        _I_ModuleBase(const double Ki, const double output_min, const double output_max):
            _output_min(output_min), _output_max(output_max), _Ki(Ki) {}

        double step(const double input_bias, const double sample_time)
        {
            // ANTI-Windup
            if (_integral_val <= _output_min)
            {
                _is_saturated_flag = true;
                _integral_val = _output_min;  // 积分限幅
                if (input_bias > 0)
                    _integral_val += input_bias * _Ki * sample_time;
            }
            else if (_integral_val >= _output_max)
            {
                _is_saturated_flag = true;
                _integral_val = _output_max;  // 积分限幅
                if (input_bias < 0)
                    _integral_val += input_bias * _Ki * sample_time;
            }
            else
            {
                _is_saturated_flag = false;
                _integral_val += input_bias * _Ki * sample_time;
            }
            return _integral_val;
        }

        bool is_saturated(void) { return _is_saturated_flag; }
        void set_Ki(const double Ki) { _Ki = Ki; }
        void reset(void) { _integral_val = 0; }

    private:
        double _Ki;
        double _integral_val = 0;
        bool _is_saturated_flag = false;
};

class _D_ModuleBase
{
    public:
        _D_ModuleBase(const double Kd, double differential_filter_alpha):
            _Kd(Kd), _differential_filter(differential_filter_alpha) {}

        double step(const double input_bias, const double sample_time)
        {
            const double differential_val = _differential_filter.step((input_bias - _last_input) / sample_time);
            _last_input = input_bias;
            return differential_val;
        }

        void set_Kd(const double Kd) { _Kd = Kd; }
        void set_differential_filter_alpha(const double alpha) { _differential_filter.set_filter_alpha(alpha); }
        void reset(void) { _last_input = 0; _differential_filter.reset(); }

    private:
        double _Kd;
        double _last_input;
        FirstOrderFilter _differential_filter;
};

template <bool DIFFERENTIAL_FORWARD>
class _PID_ModuleBase
{
    public:

        _PID_ModuleBase(const double Kp, const double Ki, const double Kd,
            const double output_min, const double output_max, const double differential_filter_alpha):
                P_module(Kp), I_module(Ki, output_min, output_max), D_module(Kd, differential_filter_alpha) {}

        bool is_saturated(void) { return I_module.is_saturated(); }
        
        double get_current_bias(void) { return _bias; }

        void set_parameter(const double Kp, const double Ki, const double Kd, const double differential_filter_alpha = 1.0)
        {
            P_module.set_Kp(Kp);
            I_module.set_Ki(Ki);
            D_module.set_Kd(Kd);
            D_module.set_differential_filter_alpha(differential_filter_alpha);
        }

        void set_limit(const double output_min, const double output_max)
        {
            I_module._output_min = output_min;
            I_module._output_max = output_max;
        }

        void reset(void)
        {
            I_module.reset();
            D_module.reset();
        }
    
    protected:

        double pid_step(const double target, const double feedback, const double sample_time)
        {
            _bias = target - feedback;
            double output = 0;
            if (DIFFERENTIAL_FORWARD)
                output = P_module.step(_bias) + I_module.step(_bias, sample_time) + D_module.step(feedback, sample_time);
            else
                output = P_module.step(_bias) + I_module.step(_bias, sample_time) + D_module.step(_bias, sample_time);

            if (output < I_module._output_min)
                output = I_module._output_min;
            else if (output > I_module._output_max)
                output = I_module._output_max;

            return output;
        }

    private:

        double _bias;
        _P_ModuleBase P_module;
        _I_ModuleBase I_module;
        _D_ModuleBase D_module;
};

template <bool USE_TIME_STAMP = false, bool DIFFERENTIAL_FORWARD = false>
class PID_Controller: public _PID_ModuleBase <DIFFERENTIAL_FORWARD>
{
    public:
        PID_Controller(double Kp, double Ki, double Kd, double output_min, double output_max,
            double sample_time, double differential_filter_alpha = 1.0):
                _PID_ModuleBase<DIFFERENTIAL_FORWARD>(Kp, Ki, Kd, output_min, output_max, differential_filter_alpha),
                _sample_time(sample_time)
        { }

        double step(const double target, const double feedback)
        {
            return _PID_ModuleBase<DIFFERENTIAL_FORWARD>::pid_step(target, feedback, _sample_time);
        }

        void set_sample_time(const double sample_time)
        {
            _sample_time = sample_time;
        }

    private:
        double _sample_time;
};

template <bool DIFFERENTIAL_FORWARD>
class PID_Controller <true, DIFFERENTIAL_FORWARD>: public _PID_ModuleBase <DIFFERENTIAL_FORWARD>
{
    public:
        PID_Controller(double Kp, double Ki, double Kd, double output_min, double output_max,
            double sample_time, double differential_filter_alpha = 1.0):
                _PID_ModuleBase<DIFFERENTIAL_FORWARD>(Kp, Ki, Kd, output_min, output_max, differential_filter_alpha),
                _sample_manager(sample_time)
        { }

        double step(const double target, const double feedback, const std::chrono::steady_clock::time_point& time_stamp)
        {
            _sample_manager.sample_update(time_stamp);
            double sample_time = _sample_manager.get_actual_sample_period();
            return _PID_ModuleBase<DIFFERENTIAL_FORWARD>::pid_step(target, feedback, sample_time);
        }

        void set_sample_time(const double sample_time)
        {
            _sample_manager.set_sample_period(sample_time);
        }

    private:
        SamplePeriodManager _sample_manager;

};

#endif
