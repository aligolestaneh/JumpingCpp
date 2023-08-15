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
#include <fstream>
#include <wiringPi.h>

#include "homing.h"
#include "utilities.h"
#include "robot_class.h"
#include "control.h"

int main() {
    Data JumpData;

    Vec78<double> robot_des_vel_hip;
    Vec78<double> robot_des_vel_thigh;
    Vec78<double> robot_des_vel_calf;
    for(int i(0); i < 78; i++){
        robot_des_vel_hip[i] = rbdl2robot_vel(JumpData.get_vel_data_f(0)[i] ,0, 0)[0];
        robot_des_vel_thigh[i] = rbdl2robot_vel(0, JumpData.get_vel_data_f(1)[i], 0)[1];
        robot_des_vel_calf[i] = rbdl2robot_vel(0, 0, JumpData.get_vel_data_f(2)[i])[2];
    }

    /*
    hip_d = hopping_data[0]
    thigh_d = hopping_data[1]
    calf_d = hopping_data[2]

    hip_vel_d = hopping_velocity[0]
    thigh_vel_d = hopping_velocity[1]
    calf_vel_d = hopping_velocity[2]
    */

    std::ofstream dataset;
    dataset.open("data_motor.txt");

//    GPIO sync_pin(GPIO_PIN);
//    int status;
//    status = sync_pin.setupPin(1);
//    status = sync_pin.setDirection(1);

    wiringPiSetupGpio();
    pinMode(PIN_IMU, OUTPUT);
    pinMode(PIN_CONTACT, INPUT);

    Homing leg("can0");        //Real Robot
    leg.enable(m1);
    leg.enable(m2);
    leg.enable(m3);

    double kp = 60, kd = 0.5;
  //double kp = 40, kd = 0.8;
  //double kp = 20, kd = 0.5;

    float* q_home = leg.start(m1, m2, m3, 60, 0.5, true, false);
    *(q_home + 2) -= 3.9366f;

    Vec3<float*> rx;
    rx[0] = leg.command(m1, q_home[0], 0, kp, kd, 0);
    rx[1] = leg.command(m2, q_home[1], 0, kp, kd, 0);
    rx[2] = leg.command(m3, 0, 0, 0, 0, 0);    //TODO

    /*
    Find the safe range during slip commands from data
    */
    Vec78<double> hip_d_robot, thigh_d_robot, calf_d_robot;
    hip_d_robot.Zero();
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
    std::chrono::steady_clock::time_point tpre = std::chrono::steady_clock::now();

    std::cout << "HIP:";
    for (const double& value : pos)
        std::cout << " " << value;
    std::cout << "\n";

    std::this_thread::sleep_for(std::chrono::seconds(2));

    float q_pre = rx[0][1];
    float* _rx;
    for (const double& p : pos){
        Vec3<double> p_robot = rbdl2robot(p, 0, 0, q_home);
        double diff = std::abs(q_pre - p_robot[0]);
        if (diff > 0.1) {
            std::cout << "diff: " << diff << '\n';

            // Perform necessary actions in case of unsafe command to motors detected
            leg.command(m1, 0, 0, 0, 0, 0);
            leg.command(m2, 0, 0, 0, 0, 0);
            leg.command(m3, 0, 0, 0, 0, 0);
            leg.disable(m1, false);
            leg.disable(m2, false);
            leg.disable(m3, false);
            std::cout << "Unsafe command to motors is detected!" << '\n';
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            throw std::runtime_error("Unsafe command to motors is detected!");
        } else {
            _rx = leg.command(m1, p_robot[0], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - tpre).count() < dt)
                int temp = 0;
            tpre = std::chrono::steady_clock::now();;
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

    std::vector<double> pos2;
    for (double p = q_rbdl[2]; q_d_rbdl[2] < p; p += dr)
        pos2.push_back(p);

    dt = 1.0 / pos2.size();
    tpre = std::chrono::steady_clock::now();

    std::cout << "CALF:";
    for (const double& value : pos2)
        std::cout << " " << value;
    std::cout << '\n';

    q_pre = rx[2][1];
    std::cout << "q_pre: " << q_pre << "\n";
    for (const double& p : pos2){
        Vec3<double> p_robot = rbdl2robot(0, 0, p, q_home);
        double diff = std::abs(q_pre - p_robot[2]);
        if (diff > 0.12) {
            std::cout << "diff: " << diff << '\n';

            // Perform necessary actions in case of unsafe command to motors detected
            leg.command(m1, 0, 0, 0, 0, 0);
            leg.command(m2, 0, 0, 0, 0, 0);
            leg.command(m3, 0, 0, 0, 0, 0);
            leg.disable(m1, false);
            leg.disable(m2, false);
            leg.disable(m3, false);
            std::cout << "Unsafe command to motors is detected!\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            throw std::runtime_error("Unsafe command to motors is detected!");
        } else {
            _rx = leg.command(m3, p_robot[2], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - tpre).count() < dt)
                int temp = 0;
            tpre = std::chrono::steady_clock::now();
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

    std::vector<double> pos3;
    for (double p = q_rbdl[1]; p < q_d_rbdl[1]; p += dr)
        pos3.push_back(p);

    dt = 1.0 / pos3.size();
    std::cout << "dt: " << dt << "\n";
    tpre = std::chrono::steady_clock::now();

    std::cout << "THIGH:";
    for (const double& value : pos3)
        std::cout << " " << value;
    std::cout << '\n';

    q_pre = rx[1][1];
    for (const double& p : pos3){
        Vec3<double> p_robot = rbdl2robot(0, p, 0, q_home);
        double diff = std::abs(q_pre - p_robot[1]);
        if (diff > 0.1) {
            std::cout << "diff: " << diff << '\n';

            // Perform necessary actions in case of unsafe command to motors detected
            leg.command(m1, 0, 0, 0, 0, 0);
            leg.command(m2, 0, 0, 0, 0, 0);
            leg.command(m3, 0, 0, 0, 0, 0);
            leg.disable(m1, false);
            leg.disable(m2, false);
            leg.disable(m3, false);
            std::cout << "Unsafe command to motors is detected!\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            throw std::runtime_error("Unsafe command to motors is detected!");
        } else {
            _rx = leg.command(m2, p_robot[1], 0, kp, kd, 0);
            q_pre = _rx[1];

            while (std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - tpre).count() < dt)
                int temp = 0;
            tpre = std::chrono::steady_clock::now();
        }
    }
    final_pos_motor[1] = _rx[1];
    final_pos_rbdl[1] = robot2rbdl(0, _rx[1], 0, q_home)[1];

    std::this_thread::sleep_for(std::chrono::seconds(1));

    std::cout << "==================================================\n";
    std::cout << "Final position of motors:" << final_pos_motor << '\n';
    std::cout << "Final position in RBDL:" << final_pos_rbdl << '\n';


    Hardware hardware(q_home[0], q_home[1], q_home[2]);
    int counter = 1;

    double t_first_cycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    double real_time_cycle = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first_cycle;
    Vec316<double> real_time_cycle_list;
    Vec316<bool> contact_list_cycle;

    // Part1 lists
    Vec316<double> q_hip_com_1, q_thigh_com_1, q_calf_com_1;
    Vec316<double> q_hip_motor_1, q_thigh_motor_1, q_calf_motor_1;
    Vec316<double> real_time_list_1;
    Vec316<double> real_time_list_3;
    //real_time_list_3.



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
    dt = 0.0025;
    int first_check, i_3;
    int n_counter = 5;
//    double robot_hip, robot_thigh, robot_calf;
//    double robot_hip_vel, robot_thigh_vel, robot_calf_vel;
    double real_time_1, real_time_2, t_des;
    Vec3<double> q_td, qdot_td, qdot_des, q_des;
    std::chrono::steady_clock::time_point t_origin;

    bool flag_first = true;

    while (counter <= n_counter)
    {
        //Part3 lists
        std::vector<double> q_hip_com_3, q_thigh_com_3, q_calf_com_3;
        std::vector<float> q_hip_motor_3, q_thigh_motor_3, q_calf_motor_3;

        Vec500<Vec3<float*>> data_cycle;
        Vec500<double> d_times;
        Vec500<int> contact_data;

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));
        first_check = 0;
        std::chrono::steady_clock::time_point t_first = std::chrono::steady_clock::now();

        int i;
        for(i = 0; i < 78; i++){
            real_time_list_1[i] = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count();

            double robot_hip = control.hip_d_robot[i];
            double robot_thigh = control.thigh_d_robot[i];
            double robot_calf = control.calf_d_robot[i];

            double robot_hip_vel = control.hip_vel_d_robot[i];
            double robot_thigh_vel = control.thigh_vel_d_robot[i];
            double robot_calf_vel = control.calf_vel_d_robot[i];

            if (!((min_hip - 0.1 < robot_hip) && (robot_hip < max_hip + 0.1))){
                std::cout << "Hip: " << robot_hip << '\n';
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else if (!((min_thigh - 0.1 < robot_thigh) && (robot_thigh < max_thigh + 0.1))){
                std::cout << "Thigh: " << robot_thigh << '\n';
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else if (!((min_calf - 0.1 < robot_calf) && (robot_calf < max_calf + 0.1))){
                std::cout << "Min Calf: " << min_calf << " Max calf: " << max_calf << '\n';
                std::cout << "Calf " << robot_calf << '\n';
                rx[0] = leg.command(m1, 0, 0, 0, 0, 0);
                rx[1] = leg.command(m2, 0, 0, 0, 0, 0);
                rx[2] = leg.command(m3, 0, 0, 0, 0, 0);
                leg.disable(m1, false);
                leg.disable(m2, false);
                leg.disable(m3, false);
                std::cout << "Unsafe command to motors is detected!!!!!!!!!!\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                throw std::runtime_error("Your command position is not in the safe range!!!!!!!!!");
            } else {
                    if (flag_first){
                        digitalWrite(PIN_IMU, 1);
                        t_origin = std::chrono::steady_clock::now();
                        flag_first = false;
                    }
                    rx[0] = leg.command(m1, robot_hip, robot_hip_vel, kp, kd, 0);
                    rx[1] = leg.command(m2, robot_thigh, robot_thigh_vel, kp, kd, 0);
                    rx[2] = leg.command(m3, robot_calf, robot_calf_vel, kp, kd, 0);

                    real_time_cycle_list[i] = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - t_first_cycle;
                    data_cycle[i] = {rx[0], rx[1], rx[2]};
                    d_times[i] = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_origin).count();
                    contact_data[i] = digitalRead(PIN_CONTACT);

                    q_hip_com_1[i] = robot_hip;
                    q_thigh_com_1[i] = robot_thigh;
                    q_calf_com_1[i] = robot_calf;

                  //qdot_hip_com_1[i] = robot_hip_vel
                  //qdot_thigh_com_1[i] = robot_thigh_vel
                  //qdot_calf_com_1[i] = robot_calf_vel

                    q_hip_motor_1[i] = rx[0][1];
                    q_thigh_motor_1[i] = rx[1][1];
                    q_calf_motor_1[i] = rx[2][1];

                    num++;
//                } else {
//                    std::cout << "pos: " << robot_hip << " " << robot_thigh << " " << robot_calf << '\n';
//                    std::cout << "vel: " << robot_hip_vel << " " << robot_thigh_vel << " " << robot_calf_vel << '\n';
//                }
                double _dt = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count() - real_time_list_1[i];
                double temp;
                if(_dt > dt)
                    std::cout << "Loop is taking more time than expected -> " << _dt <<"\n";
                while((std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count() - real_time_list_1[i]) < dt)
                    int temp = 0;
            }
        }

        /*
        TOUCHDOWN MOMENT
        */

        //std::this_thread::sleep_for(std::chrono::milliseconds(100));        //Time in flight mode
        //i++;
        //std::cout << i;
        std::chrono::steady_clock::time_point t_first_flight = std::chrono::steady_clock::now();
        std::chrono::steady_clock::time_point t_iter;
        while ((std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first_flight).count()) < 0.115){
            t_iter = std::chrono::steady_clock::now();
            rx[0] = leg.command(m1, rx[0][1], 0, 30, 0.5, 0);
            rx[1] = leg.command(m2, rx[1][1], 0, 30, 0.5, 0);
            rx[2] = leg.command(m3, rx[2][1], 0, 30, 0.5, 0);
            data_cycle[i] = {rx[0], rx[1], rx[2]};
            d_times[i] = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_origin).count();
            contact_data[i] = digitalRead(PIN_CONTACT);
            i++;
            //std::cout << i << '\n';
            while(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_iter).count() < dt)
                    int temp = 0;
        }


        t_first = std::chrono::steady_clock::now();
        real_time_1 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count();
        t_des = real_time_1 + 0.3;

        q_td = {rx[0][1], rx[1][1] , rx[2][1]};                             //q touch down given from last phase
        qdot_td = rbdl2robot_vel(0.032, 1.2014, -1.819);

        i_3 = 0;                                                            //Counter for phase 3 loop
        q_hip_motor_3.push_back(rx[0][1]);
        q_thigh_motor_3.push_back(rx[1][1]);
        q_calf_motor_3.push_back(rx[2][1]);
        dt = 0.0025;
        kp = 90;
        kd = 0.3;

        q_des = rbdl2robot(0.032, 1.2014, -1.819, q_home);
        qdot_des = {0, 0, 0};
        Cubic td_to_slip(real_time_1, t_des, q_td, qdot_td, q_des, qdot_des);
        while(real_time_1 <= t_des){
            //real_time_1 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count();
            //real_time_list_3[i_3] = real_time;
            q_des = td_to_slip.answer(real_time_1);
            //std::cout << real_time_1 << '\n';
            //std::cout << q_des[0] << " " << q_des[1] << " " << q_des[2] << '\n';
            t_iter = std::chrono::steady_clock::now();

            rx[0] = leg.command(m1, q_des[0], 0, kp, kd, 0);
            rx[1] = leg.command(m2, q_des[1], 0, kp, kd, 0);
            rx[2] = leg.command(m3, q_des[2], 0, kp, kd, 0);

            //real_time_cycle_list[i] = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - real_time_cycle).count();
            data_cycle[i] = {rx[0], rx[1], rx[2]};
            d_times[i] = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_origin).count();
            contact_data[i] = digitalRead(PIN_CONTACT);
            i++;
            data_counter++;

            q_hip_com_3.push_back(q_des[0]);
            q_thigh_com_3.push_back(q_des[1]);
            q_calf_com_3.push_back(q_des[2]);

            q_hip_motor_3.push_back(rx[0][1]);
            q_thigh_motor_3.push_back(rx[1][1]);
            q_calf_motor_3.push_back(rx[2][1]);

            safety_check({q_hip_motor_3[i_3], q_thigh_motor_3[i_3], q_calf_motor_3[i_3]}, {rx[0][1], rx[1][1], rx[2][1]}, leg);

            i_3++;

            real_time_2 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count();

            if((real_time_2 - real_time_1) > dt)
                std::cout << "Loop is taking more time than expected -> " << (real_time_2 - real_time_1) << '\n';
            while(std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_iter).count() < dt)
                    int temp = 0;
            real_time_1 = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now() - t_first).count();
        }

        if (counter == n_counter)
            digitalWrite(PIN_IMU, 0);

        leg.command(m1, q_des[0], 0, 90, 0.3, 0);
        leg.command(m2, q_des[1], 0, 90, 0.3, 0);
        leg.command(m3, q_des[2], 0, 90, 0.3, 0);

        std::this_thread::sleep_for(std::chrono::seconds(1));

        //std::cout << "We are OK so far!\n";

        for(int j(0); j < (i - 1); j++){
        dataset << data_cycle[j][0][0] << " " << data_cycle[j][0][1] << " " << data_cycle[j][0][2] << " " << data_cycle[j][0][3] << " "
                << data_cycle[j][1][0] << " " << data_cycle[j][1][1] << " " << data_cycle[j][1][2] << " " << data_cycle[j][1][3] << " "
                << data_cycle[j][2][0] << " " << data_cycle[j][2][1] << " " << data_cycle[j][2][2] << " " << data_cycle[j][2][3] << " "
                << contact_data[j] << " " << d_times[j] << std::endl;
        }

        counter++;
        data_counter++;

    }
    dataset.close();

    return 0;
}

