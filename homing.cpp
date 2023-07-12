#include "homing.h"
#include <iostream>
#include <math.h>
#include <thread>
#include <ctime>
#include <string>
#include <sys/time.h>
#include <unistd.h>
#include <unistd.h>
#include <chrono>

using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

Homing::Homing(string can_index) : Actuator(can_index){};

float Homing::min(float *numberList, int listSize)
{
    float minimum = numberList[0];

    for (int i = 0; i < listSize; i++)
    {
        if (numberList[i] < minimum)
        {
            minimum = numberList[i];
        }
    }
    return minimum;
}

float *Homing::arange(float start, float end, float step, int &size)
{
    float *result;
    size = (end - start) / step;
    result = new float[size];
    float last_item = start;
    for (int i = 0; i < size; i++)
    {
        result[i] = last_item;
        last_item += step;
    }

    return result;
}

float *Homing::start(int m1, int m2, int m3, float kp, float kd, bool enable_motors, bool disable_motors)
{

    if (enable_motors)
    {
        this->enable(m1);
        this->enable(m2);
        this->enable(m3);
    }

    float tau1_home = -.65;
    float tau3_home = .65 * 0;
    // m1_home1 = -1.92
    ///////////////////////////
    float m1_home1 = -0.843;
    // m1_home2 = -.88
    float m1_home2 = -1.843;
    ///////////////////////////
    float m2_home1 = -0.8870;
    float m2_home2 = 0.113;
    float m2_home3 = -1.8870;
    float m2_home;
    ///////////////////////////
    float m3_home_rel = -.7 * M_PI;

    float *rx1 = this->command(m1, 0, 0, 0, 0, tau1_home); // constant ff torque for hip
    float *rx3 = this->command(m3, 0, 0, 0, 0, tau3_home); // constant ff torque for calf
    float *rx2;
    this_thread::sleep_for(chrono::milliseconds(2000));

    rx1 = this->command(m1, 0, 0, 0, 0, tau1_home);
    float d1 = abs(rx1[1] - m1_home1);
    float d2 = abs(rx1[1] - m1_home2);

    float m1_home;

    if (d1 < d2) // check the nearest zero value
    {

        m1_home = m1_home1;
    }
    else
    {

        m1_home = m1_home2;
    }
    cout<<"m1_home "<<m1_home<<endl;
    cout<<"rx1[1] "<<rx1[1]<<endl;
    if (m1_home - rx1[1] > 0.15) // safty check for hip joint
    {
        rx1 = this->command(m1, 0, 0, 0, 0, 0);
        rx2 = this->command(m2, 0, 0, 0, 0, 0);
        rx3 = this->command(m3, 0, 0, 0, 0, 0);
        this->disable(m1, false);
        this->disable(m2, false);
        this->disable(m3, false);
        cout << "m1_home is: ";
        cout << rx1[1] << endl;
        cout << "diff from home :";
        cout << d1 << " " << d2 << endl;
        throw runtime_error("Motor 1 is not homed!");
    }
    else
    {

        rx1 = this->command(m1, m1_home, 0, kp, kd, 0);
        this_thread::sleep_for(chrono::milliseconds(200));
    }

    cout << "test" << endl;
    /////////////////////////// Motor 3 //////////////////////////

    rx3 = this->command(m3, 0, 0, 0, 0, tau3_home);
    rx3 = this->command(m3, 0, 0, 0, 0, tau3_home);
    float m3_home = rx3[1];

    if (m3_home > -0.4 || m3_home < -0.6) // safety check for calf actuator
    {
        rx1 = this->command(m1, 0, 0, 0, 0, 0);
        rx2 = this->command(m2, 0, 0, 0, 0, 0);
        rx3 = this->command(m3, 0, 0, 0, 0, 0);
        this->disable(m1, false);
        this->disable(m2, false);
        this->disable(m3, false);
        cout << "m3_home is:";
        cout << m3_home << endl;
        throw runtime_error("Motor 3 is not homed!");
    }
    else
    {
        float *rx = this->command(m3, 0, 0, 0, 0, 0);
        float xi = rx[1];
        float xf = xi + m3_home_rel;
        float dr = .02;
        if (xi > xf)
        {
            dr = -dr;
        }
        int size;
        float *pos = this->arange(xi, xf, dr, size);
        float dt = 1. / size;
        auto tpre = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        for (int i = 0; i < size; i++)
        {
            rx = this->command(m3, pos[i], 0, kp, kd, 0);
            while (duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count() - tpre < dt)
            {
                int temp = 0; // todo check this code
            }
            tpre = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
        }
        this_thread::sleep_for(chrono::milliseconds(200));
    }

    ////////////////////////// Motor 2 ////////////////////////////
    rx2 = this->command(m2, 0, 0, 0, 0, 0);

    cout << "rx2 = " << rx2[1] << endl;
    float dif1 = abs(rx2[1] - m2_home1);
    float dif2 = abs(rx2[1] - m2_home2);
    float dif3 = abs(rx2[1] - m2_home3);
    cout << "diff 1 = " << dif1 << endl;
    cout << "diff 2 = " << dif2 << endl;
    cout << "diff 3 = " << dif3 << endl;
    float lst[3] = {dif1, dif2, dif3};

    cout << "minimum = " << min(lst, 3) << endl;
    if (this->min(lst, 3) == dif1)
    {

        m2_home = m2_home1;
    }

    else if (this->min(lst, 3) == dif2)
    {
        m2_home = m2_home2;
    }
    else if (this->min(lst, 3) == dif3)
    {

        m2_home = m2_home3;
    }

    if (m2_home - rx2[1] > 0.15)
    {
        this->command(m1, 0, 0, 0, 0, 0);
        this->command(m2, 0, 0, 0, 0, 0);
        this->command(m3, 0, 0, 0, 0, 0);
        this->disable(m1, false);
        this->disable(m2, false);
        this->disable(m3, false);
        cout << "m2_home is: ";
        cout << m2_home << endl;
        cout << "dif: ";
        cout << m2_home - rx2[1] << endl;
        throw runtime_error("Motor 2 is not homed!");
    }
    else
    {

        rx2 = this->command(m2, m2_home, 0, kp, kd, 0);
        this_thread::sleep_for(chrono::milliseconds(200));

        if (abs(rx1[3]) > .4)
        {
            cout << "Homed motors" << m1 << ", " << m2 << ", " << m3 << ", successfully." << endl;
        }
        else
        {
            cout << "Homing failed." << endl;
        }
    }

    if (disable_motors)
    {
        rx1 = this->command(m1, 0, 0, 0, 0, 0);
        rx2 = this->command(m2, 0, 0, 0, 0, 0);
        rx3 = this->command(m3, 0, 0, 0, 0, 0);
        this->disable(m1, false);
        this->disable(m2, false);
        this->disable(m3, false);
    }
    static float result[3] = {m1_home, m2_home, m3_home};
    return result;
}