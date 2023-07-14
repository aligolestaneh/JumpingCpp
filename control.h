#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <string>
#include <ctime>

#include "actuator.h"
#include "extract_data.h"
#include "robot_class.h"
#include "hardware.h"
#include "cubic.h"
#include "GPIO.h"

class Control {
    public:
    Control(Hardware hw, int m_1, int m_2, int m_3)
        : _m1(m_1), _m2(m_2), _m3(m_3), _hw(hw){
        hip_d = data.get_pose_data_f(0);
        thigh_d = data.get_pose_data_f(1);
        calf_d = data. get_pose_data_f(2);
        hip_vel_d = data.get_vel_data_f(0);
        thigh_vel_d = data.get_vel_data_f(1);
        calf_vel_d = data.get_vel_data_f(2);
    }
    void data_for_jump();

    private:
    int _m1, _m2, _m3;
    Hardware _hw;
    Data data;
    Vec78<double> hip_d;
    Vec78<double> thigh_d;
    Vec78<double> calf_d;
    Vec78<double> hip_vel_d;
    Vec78<double> thigh_vel_d;
    Vec78<double> calf_vel_d;
    Vec78<double> hip_d_robot;
    Vec78<double> thigh_d_robot;
    Vec78<double> calf_d_robot;
    Vec78<double> hip_vel_d_robot;
    Vec78<double> thigh_vel_d_robot;
    Vec78<double> calf_vel_d_robot;
};


#endif