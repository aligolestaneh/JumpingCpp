#ifndef UTILITIES_H
#define UTILITIES_H

#include "cppTypes.h"
#include "GPIO.h"
#include "actuator.h"

#define CONTACT_PIN 13
#define m1 0x06
#define m2 0x04
#define m3 0x05

Vec3<double> fixq3(Vec3<double> q){
    q[2] = q[2] * (18./28);
    return q;
}

Vec3<double> fixq3inv(Vec3<double> q){
    q[2] = q[2] * (28./18);
    return q;
}

Vec3<double> robot2rbdl(double p1, double p2, double p3, float* q_home){
    Vec3<double> q = {-(p1 - q_home[0]), -(p2 - q_home[1]), -(p3 - q_home[2])};
    return fixq3(q);
}

Vec3<double> rbdl2robot(double p1, double p2, double p3, float* q_home){
    Vec3<double> p = fixq3inv({p1, p2, p3});
    return {q_home[0] - p[0], q_home[1] - p[1], q_home[2] - p[2]};
}

Vec3<double> robot2rbdl_vel(double v1, double v2, double v3){
    Vec3<double> q_dot = {-(v1 - 0), -(v2 - 0), -(v3 - 0)};
    return fixq3(q_dot);
}

Vec3<double> rbdl2robot_vel(double v1, double v2, double v3){
    Vec3<double> rx_dot = {v1, v2, v3}, diff = {0., 0., 0.,};
    return fixq3(diff - rx_dot);
}

bool detect_contact(){
    return GPIO::input(CONTACT_PIN);
}

Vec3<double> cal_qdot(Vec3<double> q_pre, Vec3<double> q, double t_pre,double  t_now){
    return ((q - q_pre) / (t_now - t_pre));
}

// Vec3<double> cubic_to_slip(double t, double t_td, double t_des, Vec3<double> q_td, Vec3<double> qdot_td, float* q_home){
//     double T = t_des - t_td;
//     double Tau = (t - t_td) / T;
//     if (Tau > 1)
//         throw std::runtime_error("[ERROR]: Tau > 1 in SLIPS");
//     Vec3<double> qdot_des = {0., 0., 0.}, q_des = rbdl2robot(0.032, 1.2014, -1.819, q_home);    //This hard code is from compression in slip model
//     Vec3<double> delta_q = q_des - q_td;
//     Vec4<double> rho = {0.,
//                         (T * qdot_td) / delta_q,
//                         (-2 * qdot_td - qdot_des) * T / delta_q + 3,
//                         (qdot_td + qdot_des) * T / delta_q - 2};
//     return q_td + delta_q * (rho[0] + rho[1] * Tau + rho[2] * (Tau * Tau) + rho[3] * (Tau * Tau * Tau));
// }

/*
This function gets tip of the foot close to hip.
*/
double cubic_comp(double t, double t_lo, double t_ap, double hip2calf_len){
    double T = t_ap - t_lo;
    double Tau = (t - t_lo) / T;
    if (Tau > 1)
        throw std::runtime_error("[ERROR]: Tau > 1 in COMP");
    /*!
    * @param y_ap : The height we want to descend in flight mode 
    */
    double y_ap = 0.01, y_lo = 0, ydot_lo = 0, ydot_ap = 0, delta_y = y_ap - y_lo;
    Vec4<double> rho = {0.,
                        (T * ydot_lo) / delta_y,
                        (-2 * ydot_lo - ydot_ap) * T / delta_y + 3,
                        (ydot_ap + ydot_lo) * T / delta_y - 2};
    return hip2calf_len + y_lo + delta_y * (rho[0] + rho[1] * Tau + rho[2] * (Tau * Tau) + rho[3] * (Tau * Tau * Tau));
}

/*
This function gets tip of the foot far from hip.
*/
double cubic_decomp(double t, double t_td, double t_ap, double hip2calf_len){
    double T = t_td - t_ap;
    double Tau = (t - t_ap) / T;
    if (Tau > 1)
        throw std::runtime_error("[ERROR]: Tau > 1 in DECOMP");
    /*!
    * @param y_td : The height we want to descend in flight mode 
    */
    double y_ap = 0., y_td = -0.1, ydot_td = 0, ydot_ap = 0, delta_y = y_td - y_ap;
    Vec4<double> rho = {0.,
                        (T * ydot_ap) / delta_y,
                        (-2 * ydot_ap - ydot_td) * T / delta_y + 3,
                        (ydot_ap + ydot_td) * T / delta_y - 2};
    return hip2calf_len + y_ap + delta_y * (rho[0] + rho[1] * Tau + rho[2] * (Tau * Tau) + rho[3] * (Tau * Tau * Tau));
}

void safety_check(Vec3<double> pre_cond, Vec3<double> cur_cond, Actuator leg){
    Vec3<double> epsilon = {0.15, 0.15, 0.2};
    for (int i(0); i < 3; i++){
        if (std::abs(cur_cond[i] - pre_cond[i]) > epsilon[i]){
            std::cout << "Motor" << (i+1) << "diff: " << cur_cond[0] - pre_cond[0] << std::endl;
            leg.command(m1, 0, 0, 0, 0, 0);
            leg.command(m2, 0, 0, 0, 0, 0);
            leg.command(m3, 0, 0, 0, 0, 0);
            leg.disable(m1, false);
            leg.disable(m2, false);
            leg.disable(m3, false);
            std::string motor = "motor" + std::to_string(i+1);
            std::cout << "[WARNING]: Big step detected in " << motor << "!" << std::endl;
            throw std::runtime_error("[ERROR]:" + motor + "had a big step!");
        }
    }
}


#endif
