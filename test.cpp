
#include <iostream>
#include <fstream>
#include <string>
#include <yaml-cpp/yaml.h>


using namespace std;

using namespace YAML;
int main()
{
    Node config = LoadFile("DATA.yaml");
    string test = config["hip"].as<string>();

    cout<<test<<endl;
    // str
    // Node doc;
    // while(parser(doc)){

    // }
    // // Read data from the YAML file
    // hip << config["hip"].as<double>();
    // hip_vel << config["hip_vel"].as<double>();
    // hip_f << config["hip_f"].as<double>();
    // hip_f_vel << config["hip_f_vel"].as<double>();

    // thigh << config["thigh"].as<double>();
    // thigh_vel << config["thigh_vel"].as<double>();
    // thigh_f << config["thigh_f"].as<double>();
    // thigh_f_vel << config["thigh_f_vel"].as<double>();

    // calf << config["calf"].as<double>();
    // calf_vel << config["calf_vel"].as<double>();
    // calf_f << config["calf_f"].as<double>();
    // calf_f_vel << config["calf_f_vel"].as<double>();

    // time_f << config["time_f"].as<double>();

    return 0;
}