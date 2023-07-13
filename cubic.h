#ifndef CUBIC_H
#define CUBIC_H

#include <iostream>

class Cubic{
    public:
    Cubic(double t_start, double t_end, double pos, double vel, double desired_pos, double desired_vel)
        : t_start(t_start), t_end(t_end), pos(pos) {
        T = t_end - t_start;
        Delta = desired_pos - pos;
        rho[0] = 0;
        rho[1] = (T * vel) / Delta;
        rho[2] = (-2 * vel - desired_vel) * T / Delta + 3;
        rho[3] = (vel + desired_vel) * T / Delta - 2;
    }

    double answer(double t) {
        double Tau = (t - t_start) / T;
        if (Tau > 1)
            throw std::runtime_error("tau > 1 in cubic");
        double ans = pos + Delta * (rho[0] + rho[1] * Tau + rho[2] * (Tau * Tau) + rho[3] * (Tau * Tau * Tau));
        std::cout << ans << std::endl;
        return ans;
    }
    
    private:
    double t_start, t_end;
    double pos;
    double T;
    double Delta;
    Vec3<double> rho;
};


#endif