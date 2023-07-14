/*
Ali Golestaneh
Created: Jul. 5, 2023

This code makes robot jump for many times as wanted.
*/

#include <cmath>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <thread>
#include <sys/time.h>

#include "homing.h"
#include "utilities.h"
#include "robot_class.h"
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
    Vec78<double> hip_d_robot, thigh_d_robot, calf_d_robot;
    //std::vector<double> hip_d_robot, thigh_d_robot, calf_d_robot;
    for(int i(0); i < 78; i++){
        hip_d_robot[i] = rbdl2robot(JumpData.get_pose_data_f(0)[i], 0, 0, q_home)[0];
        thigh_d_robot[i] = rbdl2robot(0, JumpData.get_pose_data_f(1)[i], 0, q_home)[1];
        calf_d_robot[i] = rbdl2robot(0, 0, JumpData.get_pose_data_f(2)[i], q_home)[2];
    }
    double min_hip = hip_d_robot.minCoeff();
    double max_hip = hip_d_robot.maxCoeff();
    double min_thigh = thigh_d_robot.minCoeff();
    double max_thigh = thigh_d_robot.maxCoeff();
    double min_calf = calf_d_robot.minCoeff();
    double max_calf = calf_d_robot.maxCoeff();

    Vec3<double> q_rbdl = robot2rbdl(rx[0][1], rx[1][1], rx[2][1], q_home);
    Vec3<double> home_robot_q = {rx[0][1], rx[1][1], rx[2][1]};
    Vec3<double> q_d_rbdl = {0.032, 1.201, -1.819};

    /* ////////////////////////////////
    Starting initial positioning of HIP
    //////////////////////////////// */
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
    float* _rx;
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
            _rx = leg.command(m1, p_robot[0], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::time(nullptr) - tpre < dt)
                int temp = 0;
            tpre = std::time(nullptr);
        }
    }
    Vec3<double> final_pos_motor, final_pos_rbdl;
    final_pos_motor[0] = _rx[1];
    final_pos_rbdl[0] = robot2rbdl(_rx[1], 0, 0, q_home)[0];

    std::this_thread::sleep_for(std::chrono::seconds(1));

    /* /////////////////////////////////
    Starting initial positioning of CALF
    ///////////////////////////////// */
    if (q_rbdl[2] > q_d_rbdl[2])
        dr = -0.02;
    else
        dr = 0.02;

    pos.clear();
    for (double p = q_rbdl[2]; p < q_d_rbdl[2]; p += dr)
        pos.push_back(p);

    dt = 1.0 / pos.size();
    tpre = std::time(nullptr);

    std::cout << "CALF:";
    for (const double& value : pos)
        std::cout << " " << value;
    std::cout << std::endl;

    q_pre = rx[2][1];
    for (const double& p : pos){
        Vec3<double> p_robot = rbdl2robot(0, 0, p, q_home);
        double diff = std::abs(q_pre - p_robot[2]);
        if (diff > 0.12) {
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
            _rx = leg.command(m3, p_robot[2], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::time(nullptr) - tpre < dt)
                int temp = 0;
            tpre = std::time(nullptr);
        }
    }
    final_pos_motor[2] = _rx[1];
    final_pos_rbdl[2] = robot2rbdl(0, 0, _rx[1], q_home)[2];

    std::this_thread::sleep_for(std::chrono::seconds(1));

    /* //////////////////////////////////
    Starting initial positioning of THIGH
    ////////////////////////////////// */
    if (q_rbdl[1] > q_d_rbdl[1])
        dr = -0.02;
    else
        dr = 0.02;

    pos.clear();
    for (double p = q_rbdl[1]; p < q_d_rbdl[1]; p += dr)
        pos.push_back(p);

    dt = 1.0 / pos.size();
    tpre = std::time(nullptr);

    std::cout << "THIGH:";
    for (const double& value : pos)
        std::cout << " " << value;
    std::cout << std::endl;

    q_pre = rx[1][1];
    for (const double& p : pos){
        Vec3<double> p_robot = rbdl2robot(0, p, 0, q_home);
        double diff = std::abs(q_pre - p_robot[1]);
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
            _rx = leg.command(m2, p_robot[1], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::time(nullptr) - tpre < dt)
                int temp = 0;
            tpre = std::time(nullptr);
        }
    }
    final_pos_motor[1] = _rx[1];
    final_pos_rbdl[1] = robot2rbdl(0, _rx[1], 0, q_home)[2];

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "==================================================";
    std::cout << "Final position of motors:" << final_pos_motor << std::endl;
    std::cout << "Final position in RBDL:" << final_pos_rbdl << std::endl;


    Hardware hardware(q_home[0], q_home[1], q_home[2]);
    int counter = 1;

    double t_first_cycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double real_time_cycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first_cycle;
    Vec78<double> real_time_cycle_list;
    Vec78<Vec3<float*>> data_cycle;
    Vec78<bool> contact_list_cycle;

    // Part1 lists
    Vec78<double> q_hip_com_1, q_thigh_com_1, q_calf_com_1;
    Vec78<double> q_hip_motor_1, q_thigh_motor_1, q_calf_motor_1;
    Vec78<double> real_time_list_1;

    //Part3 lists
    Vec78<double> q_hip_com_3, q_thigh_com_3, q_calf_com_3;
    Vec78<double> q_hip_motor_3, q_thigh_motor_3, q_calf_motor_3;

    Control control(hardware, m1, m2, m3);
    /*
    std::string path = "leg_RBDL.urdf"
    Robot robot([0, 0, 0], [0, 0, 0], path)     //Model
    */
    int data_counter = 0;
    int num = 0;
    control.data_for_jump();

    std::this_thread::sleep_for(std::chrono::seconds(20));

    /* //////////////////////////////////
    /////////// START HOPPING ///////////
    ////////////////////////////////// */
    kp = 90;
    kd = 0.3;
    dt = 0.003;
    int first_check;

    double robot_hip, robot_thigh, robot_calf;
    double robot_hip_vel, robot_thigh_vel, robot_calf_vel;

    while (counter <= 1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        first_check = 0;
        double t_first = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

        for(int i(0); i < 78; i++){
            double t_now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
            real_time_list_1[i] = t_now - t_first;
            
            robot_hip = control.hip_d_robot[i];
            robot_thigh = control.thigh_d_robot[i];
            robot_calf = control.calf_d_robot[i];

            robot_hip_vel = control.hip_vel_d_robot[i];
            robot_thigh_vel = control.thigh_vel_d_robot[i];
            robot_calf_vel = control.calf_vel_d_robot[i];

            if (!((min_hip - 0.1 < robot_hip) && (robot_hip < max_hip + 0.1))){
                std::cout << "Hip: " << robot_hip << std::endl;
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else if (!((min_thigh - 0.1 < robot_thigh) && (robot_thigh < max_thigh + 0.1))){
                std::cout << "Thigh: " << robot_thigh << std::endl;
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else if (!((min_calf - 0.1 < robot_calf) && (robot_calf < max_calf + 0.1))){
                std::cout << "Calf " << robot_calf << std::endl;
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else {
                rx[0] = leg.command(m1, robot_hip, robot_hip_vel, kp, kd, 0);
                rx[1] = leg.command(m2, robot_thigh, robot_thigh_vel, kp, kd, 0);
                rx[2] = leg.command(m3, robot_calf, robot_calf_vel, kp, kd, 0);

                real_time_cycle_list[i] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first_cycle;
                data_cycle[i] = {rx[0], rx[1], rx[2]};
              //contact_list_cycle[i] = detect_contact();
                
                q_hip_com_1[i] = robot_hip;
                q_thigh_com_1[i] = robot_thigh;
                q_calf_com_1[i] = robot_calf;

              //qdot_hip_com_1[i] = robot_hip_vel
#             //qdot_thigh_com_1[i] = robot_thigh_vel
#             //qdot_calf_com_1[i] = robot_calf_vel

                q_hip_motor_1[i] = rx[0][1];
                q_thigh_motor_1[i] = rx[1][1];
                q_calf_motor_1[i] = rx[2][1];

                num++;

                if(((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first) - real_time_list_1[num - 1]) > dt)
                    std::cout << "Loop is taking more time than expected -> " << ((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first) - real_time_list_1[num - 1]);
                while(((std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first) - real_time_list_1[num - 1]) < dt)
                    int temp = 0;
            }
        }
        counter++;
        data_counter++;
    }
    



    return 0;
}

