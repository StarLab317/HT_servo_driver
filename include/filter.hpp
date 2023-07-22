#ifndef PPC_FILTER_H
#define PPC_FILTER_H

#include <algorithm>

class FirstOrderFilter
{
    public:
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
        double integral_val;
        double filter_alpha;
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

#endif
