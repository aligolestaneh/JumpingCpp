#ifndef HARDWARE_H
#define HARDWARE_H

class Hardware{
    public:
    Hardware(double q1, double q2, double q3)
        : qh1(q1), qh2(q2), qh3(q3) {}

    Vec3<double> fixq3(Vec3<double> q){
    q[2] = q[2] * (18./28.);
    return q;
    }

    Vec3<double> fixq3inv(Vec3<double> q){
        q[2] = q[2] * (28./18.);
        return q;
    }

    Vec3<double> robot2rbdl(double p1, double p2, double p3){
        Vec3<double> q = {-(p1 - qh1), -(p2 - qh2), -(p3 - qh3)};
        return fixq3(q);
    }

    Vec3<double> rbdl2robot(double p1, double p2, double p3){
        Vec3<double> p = fixq3inv({p1, p2, p3});
        return {qh1 - p[0], qh2 - p[1], qh3 - p[2]};
    }

    Vec3<double> robot2rbdl_vel(double v1, double v2, double v3){
        Vec3<double> q_dot = {-(v1 - 0), -(v2 - 0), -(v3 - 0)};
        return fixq3(q_dot);
    }

    Vec3<double> rbdl2robot_vel(double v1, double v2, double v3){
        Vec3<double> rx_dot = {v1, v2, v3}, diff = {0., 0., 0.,};
        return fixq3inv(diff - rx_dot);
    }

        private:
        double qh1, qh2, qh3;
};

#endif
