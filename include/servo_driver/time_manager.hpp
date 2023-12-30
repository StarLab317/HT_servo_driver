#ifndef TIME_MANAGER_HPP
#define TIME_MANAGER_HPP

#include <algorithm>
#include <iostream>
#include <chrono>

class SamplePeriodManager
{
    public:

        SamplePeriodManager(double sample_period, double limit_ratio = 4.0)
            :_actual_sample_period(sample_period)
        {
            set_sample_period(sample_period, limit_ratio);
        }

        void set_sample_period(double sample_period, double limit_ratio = 4.0)
        {
            if (limit_ratio < 1.0) limit_ratio = 1.0;

            _sample_period_set = sample_period;
            _sample_period_lower = sample_period / limit_ratio;
            _sample_period_upper = sample_period * limit_ratio;
        }

        double sample_update(const std::chrono::steady_clock::time_point& now_stamp)
        {
            double period_tmp = abs(std::chrono::duration_cast<std::chrono::nanoseconds>(now_stamp - _last_stamp).count() / 1e9);
            if (period_tmp > _sample_period_upper)
            {
                period_tmp = _sample_period_upper;
                std::cout << "[WARNING] Call period does not match the sampling time" << std::endl;
            }
            else if ( period_tmp < _sample_period_lower)
            {
                period_tmp = _sample_period_lower;
                std::cout << "[WARNING] Call period does not match the sampling time" << std::endl;
            }
            _last_stamp = now_stamp;
            _actual_sample_period = period_tmp;
            
            return _actual_sample_period;
        }

        double get_actual_sample_period(void) { return _actual_sample_period; }

        double get_setting_sample_period(void) { return _sample_period_set; }

    private:

        double _actual_sample_period;
        double _sample_period_set;
        double _sample_period_lower;
        double _sample_period_upper;

        std::chrono::steady_clock::time_point _last_stamp = std::chrono::steady_clock::now();


};

#endif



