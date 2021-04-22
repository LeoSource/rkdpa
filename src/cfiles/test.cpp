// test.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <Windows.h>
#include <conio.h>
#include "GlobalParams.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include "PolyTrajPlanner.h"
#include "ArcPathPlanner.h"
#include "MirrorTask.h"
#include "InitialTask.h"
#include "HoldTask.h"

using namespace std;
using namespace Eigen;

char operation[10] = {};

enum ID_test
{
	dhmodel = 0,
	jtrajpoly = 1,
	jtrajlspb = 2,
	ctrajline = 3,
	ctrajarc = 4,
	ctrajcircle = 5,
	ctrajarctrans = 6,
	ctrajpoly = 7,
	ctrajbspline = 8,
	jacobian = 9,
	iksolver = 10,
	mirrortask = 11,
	other = 12,
	simulation = 13
};

CleanRobot CreatRobot()
{
	Matrix<double, 5, 4, RowMajor> mdh_table(g_dh);
	Matrix<int, 5, 1> jnt_type(g_type);
	Matrix<double, 5, 1> offset(g_offset);
	RobotTools::Pose pose_tool;
	pose_tool.rot.setIdentity();
	pose_tool.pos<<0, 0.2*cos(-pi/6), 0.2*sin(-pi/6);
	Matrix<double, 5, 2, RowMajor> qlimit(g_qlimit);
	CleanRobot rbt(mdh_table, jnt_type, offset, pose_tool);
	rbt.SetJntLimit(qlimit);

	return rbt;
}

void Testdhmodel()
{
	CleanRobot rbt = CreatRobot();
	VectorXd q(5);
	q<<0.2, -0.3, 0.45, 0.67, pi/2;
	RobotTools::Pose pose = rbt.FKSolve(q);

	cout<<pose.pos<<endl<<endl;
	cout<<pose.rot<<endl<<endl;
}

void Testjtrajpoly()
{
	Vector3d pos;
	pos << 0, 10, 5;
	double time[] = { 0,1.2,2 };
	PolyTrajPlanner planner(pos, time, 3);
	cout << planner._poly_params << endl;
	RobotTools::JAVP avp = planner.GenerateMotion(1.3547);
	cout << avp.pos << endl;
	cout << avp.vel << endl;
	cout << avp.acc << endl;
}

void Testjtrajlspb()
{
	Vector2d pos;
	pos << 20, 10;
	LspbTrajPlanner planner(pos, 2, 16, 10, "limitvel");
	RobotTools::JAVP avp = planner.GenerateMotion(2);
	cout << avp.pos << endl;
	cout << avp.vel << endl;
	cout << avp.acc << endl;
}

void Testctrajarc()
{
	Vector3d pos1, pos2, pos3;
	pos1<<0, -2, 0; pos2<<1, 0, 1; pos3<<0, 3, 3;
	ArcPathPlanner arcpath(pos1, pos2, pos3);
	Vector2d via_pos(0, arcpath._theta);
	double tf = 2;
	LspbTrajPlanner planner(via_pos, tf, 2, 2, "limitvel");
	int ntime = tf/g_cycle_time+1;

	//const char* file_name = "../data/traj_pos.csv";
	const char* file_name = "C:/00Work/01projects/XProject/src/data/ctrajarc_pos.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start saving data"<<endl;
		for (int idx = 0; idx<ntime; idx++)
		{
			double t = idx*g_cycle_time;
			RobotTools::JAVP javp = planner.GenerateMotion(t);
			RobotTools::CLineAVP avp = arcpath.GenerateMotion(javp.pos, javp.vel, javp.acc);
			ofile<<avp.pos<<endl;
		}
		cout<<"succeed to save the data"<<endl;
	}
	ofile.close();

}

void Testctrajcircle()
{
	Vector3d center(1, 2, 3), n_vec(1, 0, 0);
	double radius = 0.5;
	ArcPathPlanner circlepath(center, n_vec, radius);
	Vector2d via_pos(0, circlepath._theta);
	LspbTrajPlanner planner(via_pos, 2, 4, 2, "limitvel");
	RobotTools::JAVP javp = planner.GenerateMotion(2);
	RobotTools::CLineAVP avp = circlepath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	cout<<avp.pos<<endl;
	cout<<avp.vel<<endl;
	cout<<avp.acc<<endl;
}

void Testctrajarctrans()
{
	Vector3d pos1, pos2, pos3, pos4;
	pos1<<0.8, 0.2, 0.7; pos2<<-0.8, 0.2, 0.7; pos3<<-0.8, 0.8, 0.7; pos4<<0.8, 0.8, 0.7;
	Matrix<double, 3, 4> corner_pos;
	corner_pos<<pos1, pos2, pos3, pos4;
	double radius = 0.04;
	double tf = 60;
	MatrixXd via_pos = RobotTools::CalcRectanglePath(corner_pos, 16, "m");
	ArcTransPathPlanner cpath(via_pos, radius);
	Vector2d via_path(0, cpath._distance);
	LspbTrajPlanner planner(via_path, tf, 0.5, 2, "limitvel");
	RobotTools::JAVP javp = planner.GenerateMotion(54);
	RobotTools::CLineAVP avp = cpath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	cout<<avp.pos<<endl;
	cout<<avp.vel<<endl;
	cout<<avp.acc<<endl;
}

void Testiksolver()
{
	CleanRobot rbt = CreatRobot();
	Vector3d pos(0.7,0.8,1);
	//pos.setRandom();
	VectorXd q = rbt.IKSolve(pos, "q2first", 0);
	Vector3d pos_err = pos-rbt.FKSolve(q).pos;

	cout<<q<<endl<<endl;
	cout<<pos_err<<endl;
	cout<<pos<<endl;
}

void Testjacobian()
{
	CleanRobot rbt = CreatRobot();
	VectorXd q(5), dq(5);
	q<<0.3, 0.98, -0.91, 0.91, 0.56;
	dq<<0, 0, 0, 0, 0;
	MatrixXd jaco = rbt.CalcJaco(q);

	cout<<jaco<<endl<<endl;
}

void Testmirrortask()
{
	Vector3d pos1, pos2, pos3, pos4;
	pos1<<0.7, 0.8, 1; pos2<<-0.7, 0.8, 1; pos3<<-0.7, 0.8, 2.4; pos4<<0.7, 0.8, 2.4;
	CleanRobot rbt = CreatRobot();
	MirrorTask mirror_task(&rbt, 60, 0.04);
	InitialTask initial_task(&rbt);
	Matrix<double, 5, 1> q_fdb;
	q_fdb.setZero();
	mirror_task.InitTask(&rbt, pos1, pos2, pos3, pos4, q_fdb);
	vector<BaseTask*> task_data;
	task_data.push_back(&initial_task);
	task_data.push_back(&mirror_task);
	int running_state = 0;
	
	const char* file_name = "C:/00Work/01projects/XProject/src/data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start to save data"<<endl;
		while (!task_data[running_state]->_task_completed)
		{
			 VectorXd q_cmd = task_data[running_state]->RunTask(q_fdb);
			 q_fdb = q_cmd;
			 ofile<<q_cmd<<endl;
		}
	}
	cout<<"succeed to save the data"<<endl;
	ofile.close();
}

void Testother()
{

	int a[2][3] = { 1,2,3,4,5,6 };
	ofstream ofile;
	ofile.open("result.csv", ios::out|ios::trunc);
	ofile<<"First,Second,Third"<<endl;
	for (int i = 0; i < 2; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ofile<<a[i][j]<<",";
		}
		ofile<<"\n";
	}
	ofile.close();
}

DWORD WINAPI ThreadProc(LPVOID lpParameter)
{
	//char* a = (char*)lpParameter;
	while (!_kbhit())
	{
		cin>>operation;
		Sleep(10);
	}

	return 0;
}


void Simulation()
{
	CleanRobot rbt = CreatRobot();
	MirrorTask mirror_task(&rbt, 60, 0.04);
	InitialTask initial_task(&rbt);
	HoldTask hold_task(&rbt);
	vector<BaseTask*> task_data;
	task_data.push_back(&initial_task);
	task_data.push_back(&hold_task);
	task_data.push_back(&mirror_task);

	Vector3d pos1, pos2, pos3, pos4;
	pos1<<0.7, 0.8, 1; pos2<<-0.7, 0.8, 1; pos3<<-0.7, 0.8, 2.4; pos4<<0.7, 0.8, 2.4;	
	Matrix<double, 3, 4> corner_pos;
	corner_pos<<pos1, pos2, pos3, pos4;
	Matrix<double, 5, 1> q_fdb;
	q_fdb<<0.4, 0.2, 0, 0.1, 0;//initial joint position
	int running_state=0,pre_state = 0;

	const char* file_name = "C:/00Work/01projects/XProject/src/data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	HANDLE thread = CreateThread(NULL, 0, ThreadProc, NULL, 0, NULL);

	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start to simulation"<<endl;
		while (true)
		{
			running_state = task_data[running_state]->RunLogicOperation(running_state, pre_state, operation);
			if (running_state!=pre_state)
			{
				task_data[running_state]->SetTrajPos(corner_pos, q_fdb);
				cout<<"running state: "<<running_state<<endl;
				if (running_state==2)
				{
					*operation = {};
					cout<<"start to clean the mirror"<<endl;
				}
			}
			VectorXd q_cmd = task_data[running_state]->RunTask(q_fdb);
			q_fdb = q_cmd;
			ofile<<q_cmd<<endl;

			pre_state = running_state;
			Sleep(0.01);
		}

	}
	cout<<"succeed to simulation and save the data"<<endl;
	CloseHandle(thread);
	ofile.close();
}

int main()
{
	ID_test test_mode = simulation;
	switch (test_mode)
	{
	case dhmodel:
		Testdhmodel();
		break;

	case jtrajpoly:
		Testjtrajpoly();
		break;

	case jtrajlspb:
		Testjtrajlspb();
		break;

	case ctrajarc:
		Testctrajarc();
		break;

	case ctrajcircle:
		Testctrajcircle();
		break;

	case ctrajarctrans:
		Testctrajarctrans();
		break;

	case iksolver:
		Testiksolver();
		break;

	case jacobian:
		Testjacobian();
		break;

	case mirrortask:
		Testmirrortask();
		break;

	case simulation:
		Simulation();
		break;

	case other:
		Testother();
		break;
	}

    return 0;
}

