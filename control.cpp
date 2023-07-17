#include "control.h"

/*
For jumping we use slip data that is for RBDL model so we should alter it to robot coordinates.
*/
void Control::data_for_jump(){
    for(int i(0); i < 78; i++){
        hip_d_robot[i] = _hw.rbdl2robot(hip_d[i], 0, 0)[0];
        thigh_d_robot[i] = _hw.rbdl2robot(0, thigh_d[i], 0)[1];
        calf_d_robot[i] = _hw.rbdl2robot(0, 0, calf_d[i])[2];

        hip_vel_d_robot[i] = _hw.rbdl2robot_vel(hip_vel_d[i], 0, 0)[0];
        thigh_vel_d_robot[i] = _hw.rbdl2robot_vel(0, thigh_vel_d[i], 0)[1];
        calf_vel_d_robot[i] = _hw.rbdl2robot_vel(0, 0, calf_vel_d[i])[2];
    }
}

