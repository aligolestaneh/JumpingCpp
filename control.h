#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <vector>
#include <string>
#include <ctime>

#include "actuator.h"
#include "homing.h"
#include "extract_data.h"
#include "robot_class.h"
#include "hardware.h"
#include "cubic.h"
#include "GPIO.h"

class Control {
    public:
    Control(Hardware hardware, Actuator m_1, Actuator m_2, Actuator m_3)
        : _m1(m_1), _m2(m_2), _m3(m_3), hardware(hardware)
    {
        // hopping_data = data.get_pose_data_f();
        // hopping_velocity = data.get_vel_data_f();
        // hip_d = hopping_data[0];
        // thigh_d = hopping_data[1];
        // calf_d = hopping_data[2];
        // hip_vel_d = hopping_velocity[0];
        // thigh_vel_d = hopping_velocity[1];
        // calf_vel_d = hopping_velocity[2];
    }

    private:
    Actuator _m1, _m2, _m3;
    Hardware hardware;
    Data data;
    std::vector<std::vector<double>> hopping_data;
    std::vector<std::vector<double>> hopping_velocity;
    std::vector<double> hip_d;
    std::vector<double> thigh_d;
    std::vector<double> calf_d;
    std::vector<double> hip_vel_d;
    std::vector<double> thigh_vel_d;
    std::vector<double> calf_vel_d;
    std::vector<double> hip_d_robot;
    std::vector<double> thigh_d_robot;
    std::vector<double> calf_d_robot;
    std::vector<double> hip_vel_d_robot;
    std::vector<double> thigh_vel_d_robot;
    std::vector<double> calf_vel_d_robot;

};


#endif