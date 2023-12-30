#ifndef PPC_FILTER_H
#define PPC_FILTER_H

#include <algorithm>
#include "eigen-3.3.9/Eigen/Core"
#include "servo_driver/time_manager.hpp"

template <int SIZE>
struct TypeSelect
{ 
    using Type = Eigen::Matrix<double, SIZE, 1>; 
    static Type fill_value(const double val) 
    {
        Type ret;
        for (int i = 0; i < SIZE; i++) ret[i] = val;
        return ret;
    }
    static Type tan_t(Type& input) { for (int i = 0; i < SIZE; i++) input[i] = std::tan(input[i]); }
};

template <>
struct TypeSelect<1>
{ 
    using Type = double; 
    static Type fill_value(const double val) { return val; } 
    static Type tan_t(Type input) { return std::tan(input); }
};

// template <int SIZE, typename DataType = typename TypeSelect<SIZE>::Type>
// class FirstOrderFilter
// {
//     public:
//         FirstOrderFilter() { }
//         FirstOrderFilter(DataType filter_alpha):
//             _filter_alpha(filter_alpha)
//         { }

//         DataType step(DataType input_val)
//         {
//             _integral_val = _filter_alpha * input_val + (_value_one-_filter_alpha) * _integral_val;
//             return _integral_val;
//         }

//         void set_filter_alpha(DataType filter_alpha)
//         {
//             _filter_alpha = filter_alpha;
//         }

//         void reset(void)
//         {
//             _integral_val = TypeSelect<SIZE>::fill_value(0);
//         }

//     private:

//         const DataType _value_one = TypeSelect<SIZE>::fill_value(1.0);
//         DataType _integral_val = TypeSelect<SIZE>::fill_value(0);
//         DataType _filter_alpha = TypeSelect<SIZE>::fill_value(0);
// };

class FirstOrderFilter
{
    public:
        FirstOrderFilter() {}
        FirstOrderFilter(double _filter_alpha):
            filter_alpha(_filter_alpha) {}

        double step(double input_val)
        {
            integral_val = filter_alpha * input_val + (1.0-filter_alpha) * integral_val;
            return integral_val;
        }

        void set_filter_alpha(double _filter_alpha)
        {
            filter_alpha = _filter_alpha;
        }

        void reset(void)
        {
            integral_val = 0;
        }

    private:
        double integral_val = 0;
        double filter_alpha = 0;
};

class ButterworthFilter
{
    public:

        ButterworthFilter(double sample_frequency, double cutoff_frequency)
        {
            set_cutoff_frequency(sample_frequency, cutoff_frequency);
        }

        void set_cutoff_frequency(double sample_frequency, double cutoff_frequency)
        {
            double fr = sample_frequency / cutoff_frequency;
            double ohm = tan(M_PI / fr);
            double c = 1.0 + 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm;
            if (cutoff_frequency <= 0.0) {
                // no filtering
                return;
            }
            param_b[0] = ohm * ohm / c;
            param_b[1] = 2.0 * param_b[0];
            param_b[2] = param_b[0];
            param_a[0] = 1.0;
            param_a[1] = 2.0 * (ohm * ohm - 1.0) / c;
            param_a[2] = (1.0 - 2.0 * cos(M_PI / 4.0) * ohm + ohm * ohm) / c;
        }

        double step(double input)
        {
            if(output_buffer[0] == 0 &&
                output_buffer[1] == 0 &&
                output_buffer[2] == 0 &&
                input_buffer[0] == 0 &&
                input_buffer[1] == 0 &&
                input_buffer[2] == 0)
            {
                output_buffer[0] = input;
                output_buffer[1] = input;
                output_buffer[2] = input;
                input_buffer[0] = input;
                input_buffer[1] = input;
                input_buffer[2] = input;
                return input;
            }
            /* 获取最新x(n) */
            input_buffer[2] = input;
            /* Butterworth滤波 */
            output_buffer[2] = param_b[0] * input_buffer[2]
                                + param_b[1] * input_buffer[1]
                                + param_b[2] * input_buffer[0]
                                - param_a[1] * output_buffer[1]
                                - param_a[2] * output_buffer[0];
            /* x(n) 序列保存 */
            input_buffer[0] = input_buffer[1];
            input_buffer[1] = input_buffer[2];
            /* y(n) 序列保存 */
            output_buffer[0] = output_buffer[1];
            output_buffer[1] = output_buffer[2];
                
            for(uint16_t i = 0; i < 3; i++)
            {
                if(isnan(output_buffer[i]) == 1
                    || isnan(input_buffer[i]) == 1)		
                {
                    output_buffer[0] = input;
                    output_buffer[1] = input;
                    output_buffer[2] = input;
                    input_buffer[0] = input;
                    input_buffer[1] = input;
                    input_buffer[2] = input;
                    return input;
                }
            }
            return output_buffer[2];
        }

    private:

        double param_a[3] = {0};
        double param_b[3] = {0};

        double input_buffer[3] = {0};
        double output_buffer[3] = {0};
};

template <int SIZE, bool USE_TIME_STAMP = false>
class DifferentiatorThreePoint
{
    public:

        DifferentiatorThreePoint(const double sample_time, const double sample_limit_ratio = 4.0): 
            _sample_time_twice(sample_time * 2.0),
            _last_sample_time_measure(sample_time),
            _sample_manager(SamplePeriodManager(sample_time, sample_limit_ratio))
            {}

        Eigen::Matrix<double, SIZE, 1> step(const Eigen::Matrix<double, SIZE, 1> new_value)
        {
            static_assert(!USE_TIME_STAMP, "DifferentiatorThreePoint: Incorrect function called");

            return _calculate(new_value, _sample_time_twice);
        }

        Eigen::Matrix<double, SIZE, 1> step(const Eigen::Matrix<double, SIZE, 1> new_value,
            std::chrono::steady_clock::time_point now_time_stamp)
        {
            static_assert(USE_TIME_STAMP, "DifferentiatorThreePoint: Incorrect function called");

            Eigen::Matrix<double, SIZE, 1> result;
            _sample_manager.sample_update(now_time_stamp);
            double now_sample_time = _sample_manager.get_actual_sample_period();
            result = _calculate(new_value, _last_sample_time_measure + now_sample_time);
            _last_sample_time_measure = now_sample_time;

            return result;
        }

        void set_sample_time(const double sample_time)
        {
            _sample_manager.set_sample_period(sample_time);
            _sample_time_twice = sample_time * 2.0;
        }

        bool check_is_initialized(void) { return _is_initialized; }

    private:

        Eigen::Matrix<double, SIZE, 1> _calculate(const Eigen::Matrix<double, SIZE, 1> new_value, const double time_sum)
        {
            if (!_is_initialized)
            {
                _step_count++;
                if (_step_count >= 3)
                    _is_initialized = true;
            }

            Eigen::Matrix<double, SIZE, 1> diff_value = 
                (_last_2_val - 4.0*_last_1_val + 3.0*new_value) / time_sum;
                _last_2_val = _last_1_val;
                _last_1_val = new_value;

            return diff_value;
        }

        double _sample_time_twice;
        double _last_sample_time_measure;
        bool _is_initialized = false;
        int _step_count = 0;
        
        Eigen::Matrix<double, SIZE, 1> _last_1_val = Eigen::Matrix<double, SIZE, 1>::Zero();
        Eigen::Matrix<double, SIZE, 1> _last_2_val = Eigen::Matrix<double, SIZE, 1>::Zero();

        SamplePeriodManager _sample_manager;
};

#endif
