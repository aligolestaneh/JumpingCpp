/*
Ali Golestaneh
Created: Jul. 5, 2023

This code makes robot jump for many times as wanted.
*/

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <thread>

#include "homing.h"
#include "extract_data.h"
#include "utilities.h"
#include "robot_class.h"
#include "hardware.h"
#include "control.h"

int main() {
    Data JumpData;

    double robot_des_vel_hip = rbdl2robot_vel(JumpData.get_vel_data_f(0)[0] ,0, 0)[0];
    double robot_des_vel_thigh = rbdl2robot_vel(0, JumpData.get_vel_data_f(1)[0], 0)[0];
    double robot_des_vel_calf = rbdl2robot_vel(0, JumpData.get_vel_data_f(2)[0], 0)[0];

    /*
    hip_d = hopping_data[0]
    thigh_d = hopping_data[1]
    calf_d = hopping_data[2]

    hip_vel_d = hopping_velocity[0]
    thigh_vel_d = hopping_velocity[1]
    calf_vel_d = hopping_velocity[2]
    */

    Homing leg("can0");        //Real Robot
    leg.enable(m1);
    leg.enable(m2);
    leg.enable(m3);

    double kp = 60, kd = 0.9;
  //double kp = 40, kd = 0.8;
  //double kp = 20, kd = 0.5;

    float* q_home = leg.start(m1, m2, m3, 20, 0.5, true, false);

    Vec3<float*> rx;
    rx[0] = leg.command(m1, q_home[0], 0, kp, kd, 0);
    rx[1] = leg.command(m2, q_home[1], 0, kp, kd, 0);
    rx[2] = leg.command(m3, 0, 0, 0, 0, 0);    //TODO

    /*
    Find the safe range during slip commands from data
    */
    std::vector<double> hip_d_robot, thigh_d_robot, calf_d_robot;
    for(int i(0); i < 78; i++){
        hip_d_robot.push_back(rbdl2robot(JumpData.get_pose_data_f(0)[i], 0, 0, q_home)[0]);
        thigh_d_robot.push_back(rbdl2robot(0, JumpData.get_pose_data_f(1)[i], 0, q_home)[1]);
        calf_d_robot.push_back(rbdl2robot(0, 0, JumpData.get_pose_data_f(2)[i], q_home)[2]);
    }
    double min_hip = *(std::min_element(hip_d_robot.begin(), hip_d_robot.end()));
    double max_hip = *(std::max_element(hip_d_robot.begin(), hip_d_robot.end()));
    double min_thigh = *(std::min_element(thigh_d_robot.begin(), thigh_d_robot.end()));
    double max_thigh = *(std::max_element(thigh_d_robot.begin(), thigh_d_robot.end()));
    double min_calf = *(std::min_element(calf_d_robot.begin(), calf_d_robot.end()));
    double max_calf = *(std::max_element(calf_d_robot.begin(), calf_d_robot.end()));

    Vec3<double> q_rbdl = robot2rbdl(rx[0][1], rx[1][1], rx[2][1], q_home);
    Vec3<double> home_robot_q = {rx[0][1], rx[1][1], rx[2][1]};
    Vec3<double> q_d_rbdl = {0.032, 1.201, -1.819};

    /*
    Starting initial positioning of HIP
    */
    double dr;
    if (q_rbdl[0] > q_d_rbdl[0])
        dr = -0.02;
    else
        dr = 0.02;

    std::vector<double> pos;
    for (double p = q_rbdl[0]; p < q_d_rbdl[0]; p += dr)
        pos.push_back(p);

    double dt = 1.0 / pos.size();
    std::time_t tpre = std::time(nullptr);

    std::cout << "HIP:";
    for (const double& value : pos)
        std::cout << " " << value;
    std::cout << std::endl;

    std::this_thread::sleep_for(std::chrono::seconds(2));

    float q_pre = rx[0][1];

    for (const double& p : pos){
        Vec3<double> p_robot = rbdl2robot(p, 0, 0, q_home);
        double diff = std::abs(q_pre - p_robot[0]);
        if (diff > 0.1) {
            std::cout << "diff: " << diff << std::endl;

            // Perform necessary actions in case of unsafe command to motors detected
            leg.command(m1, 0, 0, 0, 0, 0);
            leg.command(m2, 0, 0, 0, 0, 0);
            leg.command(m3, 0, 0, 0, 0, 0);
            leg.disable(m1, false);
            leg.disable(m2, false);
            leg.disable(m3, false);
            std::cout << "Unsafe command to motors is detected!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            throw std::runtime_error("Unsafe command to motors is detected!");
        } else {
            float* _rx = leg.command(m1, p_robot[0], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::time(nullptr) - tpre < dt)
                int temp = 0;

            tpre = std::time(nullptr);

            // Your code inside the else block
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }       

    }

    /*
    Starting initial positioning of CALF
    */


    return 0;
}

