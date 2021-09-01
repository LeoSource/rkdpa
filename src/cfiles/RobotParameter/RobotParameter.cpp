#include <string>
#include <iostream>
#include <fstream>
#include "json/json.h"
#include "../GlobalParams.h"
//#include "../task/Common.h"
using namespace std;

//全局变量改为放在.cpp里，GlobalParams.h里去申明；
double l1 = 0.111, l2 = 0.0935, l3 = 0.066, l4 = 0.068;
double h = 0.403, w = 0.518;
double g_yaw0_x0 = l2-l3;
double g_dh[] = { 0,0,0,0,
                pi/2,0,0,0,
                0,l2,-l1,pi/2,
                pi/2,0,0,pi/2,
                0,-l4,-l3,pi/2 };
int g_type[] = { 0,1,0,1,0};
double g_offset[] = { 0, h, pi/2, w, 0 };
double g_qlimit[] = { -pi/2, pi/2, 0.01, 0.78, -pi/2, pi/2, 0.01, 0.53, -2*pi, 2*pi};
double g_cycle_time = 0.005;
double g_stowed_jpos[] = { 0,0.6,0,0,0 };
double g_stowed_cpos[] = { 0,0.7,0.9 };
double g_return_err[] = { 0.01, 0.007, 0.01, 0.007, 0.01, 0.01 };

double g_jvmax_default[] = {pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4};
double g_jamax_default[] = {pi/4, pi/4, 2 * pi/4, pi/4, pi/4, pi/4};
double g_cvmax_default[] = {0.15, 0.15};
double g_camax_default[] = {0.3, 0.3};

double g_jvmax[] = {pi/8, pi/8, 0.8*pi/4, pi/4, pi/4, pi/4};
double g_jamax[] = {pi/4, pi/4, 2 * pi/4, pi/4, pi/4, pi/4};
double g_cvmax[] = {0.15, 0.15};
double g_camax[] = {0.3, 0.3};

double g_tool_mirror[] = {0, 0, 0.39, 0.0 *pi/180, 0, 0};
double g_tool_table[] = {0, 0, 0.39, 0.0 *pi/180, 0, 0};
double g_tool_water_press[] = {0, 0, 0.29, 0.0 *pi/180, 0, 0};
double g_tool_toilet[] = {0, 0, 0.47, 0.0 *pi/180, 0, 0};
double g_tool_toilet_outer_ring[] = {0, 0, 0.47, 0.0 *pi/180, 0, 0};

double g_tool_default[] = {0, 0, 0.215, 0, 0, 0};
double g_tool_current[] = {0, 0, 0.215, 0, 0, 0};

double a3 = 0.51, a4 = 0.51, d1 = 0.048, d4 = 0.11, d5 = 0.08662, d6 = 0.035;
double g_dh_ur[] = { 0,  d1,  0,  0,
0,  0,  0,  -pi / 2,
0,  0,  a3,  0,
0,  d4,  a4, 0,
0,  d5,  0,  -pi / 2,
0,  d6,  0,  pi / 2 };
int g_type_ur[] = { 0,0,0,0,0,0 };

double g_offset_ur[] = { 0, -pi / 2, pi / 2, -pi / 2, 0, 0 };
double g_qlimit_ur[] = { -3*pi/2.0, 3*pi/2.0, -pi/2, 160.0*pi/180.0, -250.0*pi/180.0, 75*pi/180.0, -pi, pi, -pi, pi,-2 * pi, 2 * pi };

namespace RobotParameter
{
    //    输入参数：文件路径以及文件名
    //    输出参数：全局变量的参数
    int readRobotParameterFileJson(char *filename)
    {
        Json::Reader reader;
        Json::Value root;
        int jointnum = 0;

        //从文件中读取，保证当前文件有xxx.json文件
        ifstream in(filename, ios::binary);

        if (!in.is_open())
        {
            cout << "Error opening file\n";
            return -1;
        }

        if (reader.parse(in, root))
        {
            //读取根节点信息
            string name = root["name"].asString();

            if(name != "ROBOTPARAMETER")
            {
                return -1;
            }

            //读取子节点信息
            jointnum = root["robotdhpara"]["jointnum"].asInt();
            g_cycle_time = root["robotdhpara"]["cycletime"].asInt() * 0.001;

            //读取数组信息
            for (int i = 0; i < jointnum; i++)
            {
                g_type_ur[i] = root["joint_limits"]["jointtype"][i].asInt();
                double coeff = (g_type_ur[i] > 0 ? 1 : pi / 180);
                g_qlimit_ur[2 * i] = root["joint_limits"]["jointnegative"][i].asDouble() * coeff;
                g_qlimit_ur[2 * i + 1] = root["joint_limits"]["jointpositive"][i].asDouble() * coeff;

                g_jvmax_default[i] = root["joint_limits"]["maxvelocity"][i].asDouble() * coeff;
                g_jamax_default[i] = root["joint_limits"]["maxacceleration"][i].asDouble() * coeff;

                g_jvmax[i] = g_jvmax_default[i];
                g_jamax[i] = g_jamax_default[i];

                g_return_err[i] = root["joint_threshold_value"][i].asDouble();
            }

            if(jointnum == 5)
            {
                l1 = root["robotdhpara"]["L1"].asDouble();
                l2 = root["robotdhpara"]["L2"].asDouble();
                l3 = root["robotdhpara"]["L3"].asDouble();
                l4 = root["robotdhpara"]["L4"].asDouble();
                h = root["robotdhpara"]["h"].asDouble();
                w = root["robotdhpara"]["w"].asDouble();

                g_yaw0_x0 = l2 - l3;

                double dh[] = { 0,0,0,0,
                            pi/2,0,0,0,
                            0,l2,-l1,pi/2,
                            pi/2,0,0,pi/2,
                            0,-l4,-l3,pi/2 };

                memcpy(g_dh, dh, sizeof(g_dh));

                double offset[] = { 0, h, pi/2, w, 0 };
                memcpy(g_offset, offset, sizeof(g_offset));

                memcpy(g_type, g_type_ur, sizeof(g_type));
                memcpy(g_qlimit, g_qlimit_ur, sizeof(g_qlimit));
            }
            else if(jointnum == 6)
            {
                a3 = root["robotdhpara"]["A3"].asDouble();
                a4 = root["robotdhpara"]["A4"].asDouble();
                d1 = root["robotdhpara"]["D1"].asDouble();
                d4 = root["robotdhpara"]["D4"].asDouble();
                d5 = root["robotdhpara"]["D5"].asDouble();
                d6 = root["robotdhpara"]["D6"].asDouble();

                double dh_ur[] = { 0,  d1,  0,  0,
                0,  0,  0,  -pi / 2,
                0,  0,  a3,  0,
                0,  d4,  a4, 0,
                0,  d5,  0,  -pi / 2,
                0,  d6,  0,  pi / 2 };

                memcpy(g_dh_ur, dh_ur, sizeof(g_dh_ur));
            }

            g_cvmax_default[0] = root["cartsetting"]["vpath"].asDouble();
            g_cvmax_default[1] = root["cartsetting"]["vori"].asDouble();
            g_camax_default[0] = root["cartsetting"]["apath"].asDouble();
            g_camax_default[1] = root["cartsetting"]["aori"].asDouble();

            for(int i = 0; i < 2; i++)
            {
                g_cvmax[i] = g_cvmax_default[i];
                g_camax[i] =g_camax_default[i];
            }

        }
        else
        {
            cout << "parse error\n" << endl;
            return -1;
        }

        in.close();

        return jointnum;
    }

    int readRobotToolFileJson(char *filename)
    {
        Json::Reader reader;
        Json::Value root;

        //从文件中读取，保证当前文件有xxx.json文件
        ifstream in(filename, ios::binary);

        if (!in.is_open())
        {
            cout << "Error opening file\n";
            return -1;
        }

        if (reader.parse(in, root))
        {
            //读取根节点信息
            string name = root["name"].asString();

            if(name != "ROBOTTOOL")
            {
                return -1;
            }

            //读取子节点信息
            g_tool_mirror[0] = root["tool_mirror"]["x"].asDouble();
            g_tool_mirror[1] = root["tool_mirror"]["y"].asDouble();
            g_tool_mirror[2] = root["tool_mirror"]["z"].asDouble();
            g_tool_mirror[3] = root["tool_mirror"]["rx"].asDouble() * pi/180;
            g_tool_mirror[4] = root["tool_mirror"]["ry"].asDouble() * pi/180;
            g_tool_mirror[5] = root["tool_mirror"]["rz"].asDouble() * pi/180;

            g_tool_table[0] = root["tool_table"]["x"].asDouble();
            g_tool_table[1] = root["tool_table"]["y"].asDouble();
            g_tool_table[2] = root["tool_table"]["z"].asDouble();
            g_tool_table[3] = root["tool_table"]["rx"].asDouble() * pi/180;
            g_tool_table[4] = root["tool_table"]["ry"].asDouble() * pi/180;
            g_tool_table[5] = root["tool_table"]["rz"].asDouble() * pi/180;

            g_tool_water_press[0] = root["tool_water_press"]["x"].asDouble();
            g_tool_water_press[1] = root["tool_water_press"]["y"].asDouble();
            g_tool_water_press[2] = root["tool_water_press"]["z"].asDouble();
            g_tool_water_press[3] = root["tool_water_press"]["rx"].asDouble() * pi/180;
            g_tool_water_press[4] = root["tool_water_press"]["ry"].asDouble() * pi/180;
            g_tool_water_press[5] = root["tool_water_press"]["rz"].asDouble() * pi/180;

            g_tool_toilet[0] = root["tool_toilet"]["x"].asDouble();
            g_tool_toilet[1] = root["tool_toilet"]["y"].asDouble();
            g_tool_toilet[2] = root["tool_toilet"]["z"].asDouble();
            g_tool_toilet[3] = root["tool_toilet"]["rx"].asDouble() * pi/180;
            g_tool_toilet[4] = root["tool_toilet"]["ry"].asDouble() * pi/180;
            g_tool_toilet[5] = root["tool_toilet"]["rz"].asDouble() * pi/180;

            g_tool_toilet_outer_ring[0] = root["tool_toilet_outer_ring"]["x"].asDouble();
            g_tool_toilet_outer_ring[1] = root["tool_toilet_outer_ring"]["y"].asDouble();
            g_tool_toilet_outer_ring[2] = root["tool_toilet_outer_ring"]["z"].asDouble();
            g_tool_toilet_outer_ring[3] = root["tool_toilet_outer_ring"]["rx"].asDouble() * pi/180;
            g_tool_toilet_outer_ring[4] = root["tool_toilet_outer_ring"]["ry"].asDouble() * pi/180;
            g_tool_toilet_outer_ring[5] = root["tool_toilet_outer_ring"]["rz"].asDouble() * pi/180;

            g_tool_default[0] = root["tool_default"]["x"].asDouble();
            g_tool_default[1] = root["tool_default"]["y"].asDouble();
            g_tool_default[2] = root["tool_default"]["z"].asDouble();
            g_tool_default[3] = root["tool_default"]["rx"].asDouble() * pi/180;
            g_tool_default[4] = root["tool_default"]["ry"].asDouble() * pi/180;
            g_tool_default[5] = root["tool_default"]["rz"].asDouble() * pi/180;

        }
        else
        {
            cout << "parse error\n" << endl;
            return -1;
        }

        in.close();

        return 0;
    }

    int readGlobalPosDataFileJson(char *filename)
    {
        Json::Reader reader;
        Json::Value root;

        //从文件中读取，保证当前文件有xxx.json文件
        ifstream in(filename, ios::binary);

        if (!in.is_open())
        {
            cout << "Error opening file\n";
            return -1;
        }

        if (reader.parse(in, root))
        {
            //读取根节点信息
            string name = root["name"].asString();
            if(name != "GlobalPosData")
            {
                return -1;
            }

            //读取子节点信息            
            for(int i =0; i < 6; i++)
            {
//                jntpos_home[i] = root["home_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_standby[i] = root["standby_jpos"]["joint"][i].asDouble() * pi/180;

//                jntpos_lefttool_pre[i] = root["lefttool_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_lefttool_middle[i] = root["lefttool_jpos"]["joint"][i+6].asDouble() * pi/180;
//                jntpos_lefttool[i] = root["lefttool_jpos"]["joint"][i + 2*6].asDouble() * pi/180;

//                jntpos_middletool_pre[i] = root["middletool_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_middletool_middle[i] = root["middletool_jpos"]["joint"][i+6].asDouble() * pi/180;
//                jntpos_middletool[i] = root["middletool_jpos"]["joint"][i + 2*6].asDouble() * pi/180;

//                jntpos_righttool_pre[i] = root["righttool_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_righttool_middle[i] = root["righttool_jpos"]["joint"][i+6].asDouble() * pi/180;
//                jntpos_righttool[i] = root["righttool_jpos"]["joint"][i + 2*6].asDouble() * pi/180;

//                jntpos_camer_mirror[i] = root["camer_mirror_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_camer_basin[i] = root["camer_basin_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_camer_toilet[i] = root["camer_toilet_jpos"]["joint"][i].asDouble() * pi/180;
//                jntpos_camer_toilet_tool[i] = root["camer_toilet_tool_jpos"]["joint"][i].asDouble() * pi/180;
            }

        }
        else
        {
            cout << "parse error\n" << endl;
            return -1;
        }

        in.close();

        return 0;
    }
}

