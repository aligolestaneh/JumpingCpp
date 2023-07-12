#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <chrono>
#include <thread>
#include <math.h>
#include <ctime>
#include <unistd.h>
#include "actuator.h"
using namespace std;
using std::chrono::duration_cast;
using std::chrono::milliseconds;
using std::chrono::system_clock;

Actuator::Actuator(string can_index)
{
    int ret;
    int nbytes;
    int bind_write_result;
    int bind_read_result;
    struct sockaddr_can addr;
    struct ifreq ifr;
    this->__can_conf_1 = "sudo ip link set can0 type can bitrate 1000000";
    this->__can_conf_2 = "sudo ifconfig can0 txqueuelen 65536";
    this->__can_conf_3 = "sudo ifconfig can0 up";
    this->__can_conf_4 = "sudo ifconfig can0 down";
    this->__pose_shift = 0.5;
    try
    {
        cout << this->__can_conf_2 << endl;
        cout << this->__can_conf_3 << endl;
        cout << this->__can_conf_4 << endl;

        system(this->__can_conf_4.c_str());
        system(this->__can_conf_1.c_str());
        system(this->__can_conf_2.c_str());
        system(this->__can_conf_3.c_str());

        this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW); // 1. create socket
        if (this->s < 0)
        {
            perror("socket PF_CAN failed");
            return;
        }

        //        2.Specify can_index device
        strcpy(ifr.ifr_name, "can0");
        ret = ioctl(this->s, SIOCGIFINDEX, &ifr);

        if (ret < 0)
        {
            perror("ioctl failed");
            return;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        ret = bind(this->s, (struct sockaddr *)&addr, sizeof(addr));

        if (ret < 0)
        {
            perror("bind write failed");
            return;
        }

        addr.can_family = PF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        ret = bind(this->s, (struct sockaddr *)&addr, sizeof(addr));
        if (ret < 0)
        {
            perror("bind failed");
            return;
        }
        //        struct can_filter rfilter[1];
        //        rfilter[0].can_id   = 0x00;
        //        rfilter[0].can_mask = CAN_SFF_MASK;
        //
        //        setsockopt(this->s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

        cout << "CAN communication established" << endl;
        return;
    }
    catch (int exp)
    {
        cout << "failed to establish CAN communication" << exp << endl;
    }
};

void Actuator::enable(int id)
{
    struct can_frame writeFrame, readFrame;

    writeFrame.can_id = id;
    writeFrame.can_dlc = 8;
    writeFrame.data[0] = 255;
    writeFrame.data[1] = 255;
    writeFrame.data[2] = 255;
    writeFrame.data[3] = 255;
    writeFrame.data[4] = 255;
    writeFrame.data[5] = 255;
    writeFrame.data[6] = 255;
    writeFrame.data[7] = 252;

    int nbytes = write(this->s, &writeFrame, sizeof(struct can_frame));

    if (nbytes != sizeof(writeFrame))
    {
        printf("Send Error frame[0]!\r\n");
    }

    nbytes = read(this->s, &readFrame, sizeof(struct can_frame));
    cout << "read data " << nbytes << endl;
}
void Actuator::disable(int id, bool down_communication = false)
{

    struct can_frame frame;

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = 255;
    frame.data[1] = 255;
    frame.data[2] = 255;
    frame.data[3] = 255;
    frame.data[4] = 255;
    frame.data[5] = 255;
    frame.data[6] = 255;
    frame.data[7] = 253;

    int nbytes = write(this->s, &frame, sizeof(struct can_frame));

    if (nbytes != sizeof(frame))
    {
        printf("Send Error frame[0]!\r\n");
    }

    if (down_communication)
    {
        system(this->__can_conf_4.c_str());
    }
}

void Actuator::zero(int id)
{
    struct can_frame frame;

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = 255;
    frame.data[1] = 255;
    frame.data[2] = 255;
    frame.data[3] = 255;
    frame.data[4] = 255;
    frame.data[5] = 255;
    frame.data[6] = 255;
    frame.data[7] = 254;

    int nbytes = write(this->s, &frame, sizeof(struct can_frame));

    if (nbytes != sizeof(frame))
    {
        printf("Send Error frame[0]!\r\n");
    }
}

float *Actuator::command(int id, double p_d, double v_d, double kp, double kd, double ff)
{
    struct can_frame frame;

    float *data = this->__pack2(p_d + this->__pose_shift, v_d, kp, kd, ff);

    frame.can_id = id;
    frame.can_dlc = 8;
    frame.data[0] = data[0];
    frame.data[1] = data[1];
    frame.data[2] = data[2];
    frame.data[3] = data[3];
    frame.data[4] = data[4];
    frame.data[5] = data[5];
    frame.data[6] = data[6];
    frame.data[7] = data[7];

    int nbytes = write(this->s, &frame, sizeof(struct can_frame));
    this_thread::sleep_for(chrono::milliseconds(10));
    if (nbytes != sizeof(frame))
    {
        printf("Send Error frame[0]!\r\n");
    }
    this_thread::sleep_for(chrono::milliseconds(10));
    delete[] data;
    float *result = this->__unpack(id);
    // delete allocated memory
    return result;
}

float *Actuator::__pack2(double p_d, double v_d, double kp, double kd, double ff)
{

    float *result = new float[8]; // delete allocated memory
    int p_data = static_cast<int>(((p_d + 95.5) * 65535 / 191));
    int v_data = static_cast<int>(((v_d + 45) * 4095 / 90));
    int kp_data = static_cast<int>((kp * 4095 / 500));
    int kd_data = static_cast<int>((kd * 4095 / 5));
    int ff_data = static_cast<int>(((ff + 18) * 4095 / 36));
    result[0] = p_data >> 8;
    result[1] = p_data & 0xFF;
    result[2] = v_data >> 4;
    result[3] = ((v_data & 0xF) << 4) | (kp_data >> 8);
    result[4] = kp_data & 0xFF;
    result[5] = kd_data >> 4;
    result[6] = ((kd_data & 0xF) << 4) | (ff_data >> 8);
    result[7] = ff_data & 0xFF;
    return result;
}

float *Actuator::__unpack(int motor_id)
{

    // 5.Receive data and exit
    struct can_frame frame;

    while (1)
    {
        int nbytes = read(this->s, &frame, sizeof(struct can_frame));
        if (nbytes > 0)
        {

//            printf("can_id = %d     \r\ncan_dlc = %d \r\n", frame.can_id, frame.can_dlc);
            int i = 0;
//            for (i = 0; i < 8; i++)
//            {
//                printf("data[%d] = %d\r\n", i, frame.data[i]);
//            }

            break;
        }
        else
        {
            cout << "reading data filed" << endl;
        }
    }

    float *result = new float[4];
    result[0] = frame.data[0];
    int p_data = frame.data[1] * 16 * 16 + frame.data[2];
    float p = p_data * (190. / 65535.) - 95.5;
    result[1] = p;

    int b4[8];
    this->decToBinary(frame.data[4], b4);

    int highBin[4] = {b4[0], b4[1], b4[2], b4[3]};
    int lowBin[4] = {b4[4], b4[5], b4[6], b4[7]};
    int d4_high = this->binaryToDec(highBin);

    int d4_low = this->binaryToDec(lowBin);

    int v_data = frame.data[3] * 16 + d4_high;

    float v = (v_data * 90) / 4095. - 45;

    result[2] = v;

    int i_data = d4_low * 16 * 16 + frame.data[5];
    float i = (i_data * 36) / 4095. - 18;
    result[3] = i;

    return result;
}

void Actuator::decToBinary(int n, int *binaryNum)
{
    for (int i = 7; i >= 0; i--)
    {
        binaryNum[i] = n % 2;
        n /= 2;
    }
}

int Actuator::binaryToDec(int *binaryNum)
{

    int decimal = 0;

    for (int i = 0; i < 4; i++)
        decimal = decimal * 2 + binaryNum[i];

    return decimal;
}

// int main ()
//{
//     Actuator actuator("can0");
//
//     actuator.enable(0x01);
//     sleep(2);
//     float * result = actuator.command(0x01,0,0,0,0,0);
//     float initialPosition= result[1];
//     float finalPosition = initialPosition + 2;
//     cout<<initialPosition<<endl;
//     cout<<finalPosition<<endl;
//     int i = 0;
//     while( finalPosition >= initialPosition )
//     {
//
////        cout<<initialPosition<<endl;
//        initialPosition += 0.1;
//        sleep(0.2);
//        float * result = actuator.command(0x01,initialPosition,0,20,0.8,0);
//        sleep(0.2);
////        initialPosition = result[1];
//        cout<<"result: "<<result[1]<<endl;
//    }
//
//
//
//    while(1)
//    {
//        float * result2 = actuator.command(0x01,initialPosition,0,20,0.8,0);
//        cout<<"end: "<<result2[1]<<endl;
//    }
////    sleep(0.5);
////    result = actuator.command(0x01,0,0,0,0,0);
////    cout<<"end: "<<result[1]<<endl;
////    sleep(0.5);
////    result = actuator.command(0x01,0,0,0,0,0);
////    cout<<"end: "<<result[1]<<endl;
////    actuator.disable(0x01);
//}