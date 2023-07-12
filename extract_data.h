#ifndef EXTRACT_DATA_H
#define EXTRACT_DATA_H

#include "cppTypes.h"

class Data{
    public:
        Data();
        Vec316<double> get_pose_data(int link);
        Vec316<double> get_vel_data(int link);
        Vec78<double> get_pose_data_f(int link);
        Vec78<double> get_vel_data_f(int link);

        Vec316<double> hip, hip_vel, thigh, thigh_vel, calf, calf_vel;
        Vec78<double> hip_f, hip_f_vel, thigh_f, thigh_f_vel, calf_f, calf_f_vel, time_f;
};

#endif