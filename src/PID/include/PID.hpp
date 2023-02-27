#ifndef PID_H
#define PID_H

#include<cmath>

class PID{

    private:
        double kp, ki, kd, alpha;
        double max_out;
        double integral;
        double old_ef;
        double t_prev;

    public:
        PID(double _kp, double _ki, double _kd, double _fc);
        static double calcAlphaEMA(double fn);
        double update(double err, double t);
};

#endif
