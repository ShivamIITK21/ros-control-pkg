#ifndef FUZZYPID_H
#define FUZZYPID_H

#include "fl/Headers.h"
#include<utility>
using namespace fl;
typedef struct gainRange{
    std::pair<double, double> deledot;
    std::pair<double, double> dele;
    std::pair<double, double> delkp;
    std::pair<double, double> delki;
    std::pair<double, double> delkd;
}gainRange;


class FuzzyPID{
    public:
        double kp, ki, kd;
        double integral;
        double derivative;
        double old_ef;
        double t_prev;
        gainRange ranges;
        fl::Engine * engine;

        fl::InputVariable * e;
        fl::InputVariable * edot;
        fl::OutputVariable * dkp;
        fl::OutputVariable * dki;
        fl::OutputVariable * dkd;

        FuzzyPID(double _kp, double _ki, double _kd, gainRange _ranges);

        double update(double error, double t);
        void updateGains();
        void setMembershipFuncsInp(InputVariable * var, double start, double end);
        void setMembershipFuncsOut(OutputVariable * var, double start, double end);
};


#endif
