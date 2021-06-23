#include <string>
#include <iostream>
#include <fstream>
#include "json/json.h"
#include "../GlobalParams.h"
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
double g_return_err[] = { 0.01, 0.007, 0.01, 0.007, 0.01 };
double g_jvmax[] = {pi*0.5, 0.15, 0.8*pi*0.5, 0.5, 0.8*pi*0.5, 0};
double g_jamax[] = { 2*pi*0.5, 0.3, 1.6*pi*0.5, 1, 1.6*pi*0.5, 0};
double g_cvmax = 0.15;
double g_camax = 0.3;

double g_tool_mirror[] = {0, 0.2196, -0.05578, 30*pi/180, 0, 0};
double g_tool_table[] = {0, 0.2196, -0.05578, 0, 0, 0};
double g_tool_default[] = {0, 0.2196, -0.05578, 0, 0, 0};

double a3 = 0.51, a4 = 0.51, d1 = 0.048, d4 = 0.11, d5 = 0.08662, d6 = 0.035;
double g_dh_ur[] = { 0,  d1,  0,  0,
0,  0,  0,  -pi / 2,
0,  0,  a3,  0,
0,  d4,  a4, 0,
0,  d5,  0,  -pi / 2,
0,  d6,  0,  pi / 2 };
int g_type_ur[] = { 0,0,0,0,0,0 };
double g_offset_ur[] = { 0, -pi / 2, pi / 2, -pi / 2, 0, 0 };
double g_qlimit_ur[] = { -pi, pi, -pi, pi, -3 * pi / 2, pi / 2, -pi, pi, -pi, pi,-2 * pi, 2 * pi };

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
				g_jvmax[i] = root["joint_limits"]["maxvelocity"][i].asDouble() * coeff;
				g_jamax[i] = root["joint_limits"]["maxacceleration"][i].asDouble() * coeff;

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

            g_cvmax = root["cartsetting"]["vpath"].asDouble();
            g_camax = root["cartsetting"]["apath"].asDouble();

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
        ifstream in(filename, ios::binary); // "/home/danad/QT_Projects/TLog/ConfigureFiles_json/RobotTool.json"

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
            g_tool_mirror[3] = root["tool_mirror"]["roll"].asDouble() * pi/180;
            g_tool_mirror[4] = root["tool_mirror"]["pitch"].asDouble() * pi/180;
            g_tool_mirror[5] = root["tool_mirror"]["yaw"].asDouble() * pi/180;

            g_tool_table[0] = root["tool_table"]["x"].asDouble();
            g_tool_table[1] = root["tool_table"]["y"].asDouble();
            g_tool_table[2] = root["tool_table"]["z"].asDouble();
            g_tool_table[3] = root["tool_table"]["roll"].asDouble() * pi/180;
            g_tool_table[4] = root["tool_table"]["pitch"].asDouble() * pi/180;
            g_tool_table[5] = root["tool_table"]["yaw"].asDouble() * pi/180;

            g_tool_default[0] = root["tool_usered"]["x"].asDouble();
            g_tool_default[1] = root["tool_usered"]["y"].asDouble();
            g_tool_default[2] = root["tool_usered"]["z"].asDouble();
            g_tool_default[3] = root["tool_usered"]["roll"].asDouble() * pi/180;
            g_tool_default[4] = root["tool_usered"]["pitch"].asDouble() * pi/180;
            g_tool_default[5] = root["tool_usered"]["yaw"].asDouble() * pi/180;

        }
        else
        {
            cout << "parse error\n" << endl;
            return -1;
        }

        in.close();

        return 0;
    }

    int readHomingDataFileJson(char *filename)
    {
        Json::Reader reader;
        Json::Value root;

        //从文件中读取，保证当前文件有xxx.json文件
        ifstream in(filename, ios::binary); // "/home/danad/QT_Projects/TLog/ConfigureFiles_json/HomingData.json"

        if (!in.is_open())
        {
            cout << "Error opening file\n";
            return -1;
        }

        if (reader.parse(in, root))
        {
            //读取根节点信息
            string name = root["name"].asString();
            if(name != "HomeDataPos")
            {
                return -1;
            }

            //读取子节点信息
            g_stowed_jpos[0] = root["stowed_jpos"]["j1"].asDouble();
            g_stowed_jpos[1] = root["stowed_jpos"]["j2"].asDouble();
            g_stowed_jpos[2] = root["stowed_jpos"]["j3"].asDouble();
            g_stowed_jpos[3] = root["stowed_jpos"]["j4"].asDouble();
            g_stowed_jpos[4] = root["stowed_jpos"]["j5"].asDouble();
//            g_stowed_jpos[5] = root["stowed_jpos"]["j6"].asDouble();

            g_stowed_cpos[0] = root["stowed_cpos"]["x"].asDouble();
            g_stowed_cpos[1] = root["stowed_cpos"]["y"].asDouble();
            g_stowed_cpos[2] = root["stowed_cpos"]["z"].asDouble();
//            g_stowed_cpos[3] = root["stowed_cpos"]["roll"].asDouble();
//            g_stowed_cpos[4] = root["stowed_cpos"]["pitch"].asDouble();
//            g_stowed_cpos[5] = root["stowed_cpos"]["yaw"].asDouble();

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

