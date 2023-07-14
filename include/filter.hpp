#ifndef PPC_FILTER_H
#define PPC_FILTER_H

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

    private:
        double integral_val;
        double filter_alpha;
};

#endif
