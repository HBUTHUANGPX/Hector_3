#ifndef _F_M_CALCULATE_H
#define _F_M_CALCULATE_H
#include <ros/ros.h>
#include "../common/cppTypes.h"
class F_M_calculate
{
private:
    double L1X, L1Z, L1, L2, L3, L4, L5;
    double M1, M2, M3;
    double F1, F2, F3;
    double m1, m2, m3;
    double sita, fai, alpha;
    double sin_fai, sin_alpha, sin_sita;
    double cos_fai, cos_alpha, cos_sita;
    double g;
    double f, F4, My;
    Vec6<double> F_M;

public:
    F_M_calculate(/* args */)
    {
        g = 9.81;
        L1X = 0.05;
        L1Z = 0.05;
        m1 = 0.6;
        L1 = sqrt(L1X * L1X + L1Z * L1Z);
        fai = asin(L1Z / L1);
        sin_fai = L1Z / L1;
        cos_fai = L1X / L1;
        F1 = m1 * g * sin_fai;
        M1 = -m1 * g * cos_fai * L1;
        L2 = 0.15;
        L3 = 0.16;
        L4 = 0.05;
        L5 = 0.04;
        m2 = 1.1;
        m3 = 1.1;
        F_M.setZero();
    }
    ~F_M_calculate()
    {
    }
    Vec6<double> calculate(double sita_)
    {
        sita = sita_;
        alpha = M_PI_2 - sita + fai;
        sin_alpha = sin(alpha);
        sin_sita = sin(sita);
        cos_alpha = cos(alpha);
        cos_sita = cos(sita);
        F2 = F1 * cos_alpha + m2 * g * cos_sita * L2 / 2;
        M2 = -M1 - m2 * g * sin_sita * L2 / 2 - F1 * sin_alpha * L2;
        F3 = F2*cos(2*sita)+m3*g*cos_sita;
        M3 = -M2-m3*g*sin_sita*L3/2-F2*sin(2*sita)*L3;
        F_M[0] = F3*sin_sita;
        F_M[2] = -F3*cos_sita;
        F_M[4] = M3;
        printf("%lf  %lf  %lf\n",M1,-M2,M3);
        return F_M;
    }
};
#endif