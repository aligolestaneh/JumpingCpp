#ifndef HARDWARE_H
#define HARDWARE_H

class Hardware{
    public:
    Hardware(double q1, double q2, double q3)
        : qh1(q1), qh2(q2), qh3(q3) {}

    private:
    double qh1, qh2, qh3;
};



#endif