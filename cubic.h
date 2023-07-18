#ifndef CUBIC_H
#define CUBIC_H

#include <iostream>
#include "cppTypes.h"

class Cubic{
    public:
    Cubic(double t_start, double t_end, Vec3<double> pos, Vec3<double> vel, Vec3<double> desired_pos, Vec3<double> desired_vel)
        : t_start(t_start), t_end(t_end), pos(pos) {
        T = t_end - t_start;
        Delta = desired_pos - pos;
        rho_0 = {0, 0, 0};
        rho_1 = {(T * vel[0]) / Delta[0], (T * vel[1]) / Delta[1], (T * vel[2]) / Delta[2]};
        rho_2 = {(-2 * vel[0] - desired_vel[0]) * T / Delta[0] + 3, (-2 * vel[1] - desired_vel[1]) * T / Delta[1] + 3, (-2 * vel[2] - desired_vel[2]) * T / Delta[2] + 3};
        rho_3 = {(vel[0] + desired_vel[0]) * T / Delta[0] - 2, (vel[1] + desired_vel[1]) * T / Delta[1] - 2, (vel[2] + desired_vel[2]) * T / Delta[2] - 2};
    }

    Vec3<double> answer(double t) {
        double Tau = (t - t_start) / T;
        if (Tau > 1)
            throw std::runtime_error("tau > 1 in cubic");
        Vec3<double> ans = {pos[0] + Delta[0] * (rho_0[0] + rho_1[0] * Tau + rho_2[0] * (Tau * Tau) + rho_3[0] * (Tau * Tau * Tau)),
                            pos[1] + Delta[1] * (rho_0[1] + rho_1[1] * Tau + rho_2[1] * (Tau * Tau) + rho_3[1] * (Tau * Tau * Tau)),
                            pos[2] + Delta[2] * (rho_0[2] + rho_1[2] * Tau + rho_2[2] * (Tau * Tau) + rho_3[2] * (Tau * Tau * Tau))};
        //std::cout << ans << std::endl;
        return ans;
    }
    
    private:
    double t_start, t_end;
    Vec3<double> pos;
    double T;
    Vec3<double> Delta;
    Vec3<double> rho_0, rho_1, rho_2, rho_3;
};


#endif