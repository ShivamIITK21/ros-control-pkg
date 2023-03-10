#include "PID.hpp"

PID::PID(double _kp, double _ki, double _kd, double _fc){
    kp = _kp;
    ki = _ki;
    kd = _kd;
    alpha = calcAlphaEMA(_fc);
    integral = 0;
    old_ef = 0;
    t_prev = -1;
}


double PID::calcAlphaEMA(double fn){
    if (fn <= 0)
        return 1;
    const double c = std::cos(2 * float(M_PI) * fn);
    return c - 1 + std::sqrt(c * c - 4 * c + 3);
}

double PID::update(double error, double t){
    double dt = t - t_prev;
    double ef = alpha*error + (1 - alpha)*old_ef;
    double derivative = (ef - old_ef)/dt;
    //double derivative = 0; this is just for testing with random messages
    double new_integral = integral + error*dt;

    double updated = kp*error + ki*integral + kd*derivative;

    integral = new_integral;
    old_ef = ef;
    t_prev = t;

    return updated;
}
