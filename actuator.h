#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <linux/can.h>
#include <string>

class Actuator {
public:
    std::string __can_conf_1;
    std::string __can_conf_2;
    std::string __can_conf_3;
    std::string __can_conf_4;
    int s;
    int __data[8] = {0,0,0,0,0,0,0,0};
    float __pose_shift;
    struct can_frame frame;
    struct can_filter rfilter[1];
    Actuator(std::string can_index);
    void enable(int id);
    void disable(int id, bool down_communication /* false*/);

    void zero(int id);
    float * command(int id,double p_d,double v_d,double kp,double kd,double ff);
    float * __unpack(int motor_id);
    float * __pack2(double p_d,double v_d,double kp,double kd,double ff);
    void decToBinary(int n, int* binaryNum);
    int binaryToDec(int* binaryNum);
};

#endif