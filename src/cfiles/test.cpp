// test.cpp : 定义控制台应用程序的入口点。
//

#include <Windows.h>
#include <conio.h>
#include "GlobalParams.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include "PolyTrajPlanner.h"
#include "MirrorTask.h"
#include "InitialTask.h"
#include "HoldTask.h"
#include "SurfaceTask.h"
#include "RotXPlaneTask.h"
#include "TableTask.h"
#include "ShowerRoomTask.h"
#include <time.h>
#include "TaskTrajPlanner.h"

#include "InterruptStop/InterruptStop.h"
#include "RobotParameter/RobotParameter.h"
#pragma warning(disable : 4996)
//#define lzx
#define wyc
//#define xnd
#define urrobot
using namespace std;
using namespace Eigen;

char operation[10] = {};

enum ID_test
{
	dhmodel = 0,
	jtrajpoly = 1,
	jtrajlspb = 2,
	ctrajline = 3,
	ctrajarctrans = 4,
	ctrajpoly = 5,
	ctrajbspline = 6,
	jacobian = 7,
	iksolver = 8,
	bspline = 9,
	other = 10,
	simulation = 11
};

CleanRobot CreatRobot()
{
	Matrix<double, 5, 4, RowMajor> mdh_table(&g_dh[0]);
	Matrix<int, 5, 1> jnt_type(&g_type[0]);
	Matrix<double, 5, 1> offset(&g_offset[0]);
//    double tool_rx = 30*pi/180;
//    Vector3d tool_pos(0, 0.2196, -0.05578);
    double tool_rx = g_tool_default[3];
    Vector3d tool_pos(g_tool_default[0], g_tool_default[1], g_tool_default[2]);

	Matrix<double, 5, 2, RowMajor> qlimit(&g_qlimit[0]);
	//CleanRobot rbt(mdh_table, jnt_type, offset, pose_tool);
    CleanRobot rbt(mdh_table, jnt_type, offset, tool_rx, tool_pos);
	rbt.SetJntLimit(qlimit);

	return rbt;
}

UrRobot CreatUrRobot()
{
	Matrix<double, 6, 4, RowMajor> mdh_table(&g_dh_ur[0]);
	Matrix<int, 6, 1> jnt_type(&g_type_ur[0]);
	Matrix<double, 6, 1> offset(&g_offset_ur[0]);
	RobotTools::Pose pose_tool;
	pose_tool.rot.setIdentity();
	pose_tool.pos << 0, 0, 0.116;
	Matrix<double, 6, 2, RowMajor> qlimit(&g_qlimit_ur[0]);
	UrRobot rbt(mdh_table, jnt_type, offset, pose_tool);
	rbt.SetJntLimit(qlimit);

	return rbt;
}

void Testdhmodel()
{
	CleanRobot rbt = CreatRobot();
	VectorXd q(5);
	q<<0.2, -0.3, 0.45, 0.67, pi/2;
	RobotTools::Pose pose = rbt.FKSolveTool(q);

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
	Vector2d duration(0, 3);
	LspbTrajPlanner planner(pos, 16, 10, duration);
	RobotTools::JAVP avp = planner.GenerateMotion(2.2);
	cout << avp.pos << endl;
	cout << avp.vel << endl;
	cout << avp.acc << endl;
}


void Testctrajarctrans()
{
	Vector3d pos1, pos2, pos3, pos4;
	pos1<<0.7, 0.8, 1; pos2<<-0.7, 0.8, 1; pos3<<-0.7, 0.8, 2.4; pos4<<0.7, 0.8, 2.4;
	MatrixXd corner_pos;
	corner_pos.setZero(3, 4);
	corner_pos<<pos1, pos2, pos3, pos4;
	double radius = 0.04;
	double tf = 60;
	double interval = 0.08;
	MatrixXd via_pos = RobotTools::CalcRectanglePath(&corner_pos,"s", interval);
	ArcTransPathPlanner cpath(via_pos, 0);
	Vector2d via_path(0, cpath._distance);
	LspbTrajPlanner planner(via_path, 0.5, 2);
	RobotTools::JAVP javp = planner.GenerateMotion(10);
	RobotTools::CLineAVP avp = cpath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	cout<<avp.pos<<endl;
	cout<<avp.vel<<endl;
	cout<<avp.acc<<endl;
}

void Testiksolver()
{
	CleanRobot rbt = CreatRobot();
	Vector3d pos(0.7, 0.8, 1);
	//pos.setRandom();
	Matrix<double, 5, 1> q_fdb;
	q_fdb << 0.2, 0.8, 0.7, 0.3, 0.5;//initial joint position
	VectorXd q = rbt.IKSolve(pos, "q2first", 0, q_fdb);
	Vector3d pos_err = pos - rbt.FKSolve(q).pos;

	cout << q << endl << endl;
	cout << pos_err << endl;
	cout << pos << endl;
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

void TestBSpline()
{
	double tf = 60;
	double dt = 0.01;
	int np = tf/dt+1;
	int npts[] = { 15,12,10,8,6,4 };
	double center[] = { 0,0,0,0,0,0, 0,0,0,0,0,0, 0,-0.06,-0.12,-0.18,-0.24,-0.3 };
	double elli_params[] = { 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.5, 0.4, 0.3, 0.2, 0.1, 0.05 };
	Matrix<double, 3, 6, RowMajor> origin(center);
	Matrix<double, 2, 6, RowMajor> elli(elli_params);
	MatrixXd via_pos;
	via_pos.setZero(3, 61);
	vector<Vector3d> vp;
	for (int midx = 0; midx<sizeof(npts)/sizeof(npts[0]); midx++)
	{
		VectorXd theta;
		theta.setLinSpaced(npts[midx]+1, 0, 2*pi);
		for (int nidx = 0; nidx<theta.size(); nidx++)
		{
			Vector3d tmp_pos;
			tmp_pos<<elli(0, midx)*cos(theta(nidx)), elli(1, midx)*sin(theta(nidx)), 0;
			tmp_pos += origin.col(midx);
			vp.push_back(tmp_pos);
		}
	}
	for (int idx = 0; idx<vp.size(); idx++)
	{
		via_pos.col(idx) = vp[idx];
	}
	//MatrixXd via_pos;
	//via_pos.setZero(3, 5);
	//via_pos<<1, 2, 3, 4, 5, 2, 3, -3, 4, 5, 0, 0, 0, 0, 0;
	CubicBSplinePlanner bspline(&via_pos, "approximation", 60);
	Vector2d via_u(bspline._uknot_vec(0), bspline._uknot_vec(bspline._uknot_vec.size()-1));
	LspbTrajPlanner uplanner(via_u, 2, 1, bspline._uknot_vec(bspline._uknot_vec.size()-1));
	//cout<<bspline.CalcBSplineCoeff(3,5,0.8)<<endl;
	//CLineAVP avp = bspline.GenerateMotion(0.1, 0.5, 0.1);
	//cout<<avp.pos<<endl;
	//cout<<avp.vel<<endl;
	//cout<<avp.acc<<endl;
	const char* file_name = "C:/00Work/01projects/XProject/src/data/test_bspline.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		for (int idx = 0; idx<np; idx++)
		{
			double t = idx*dt;
			JAVP uavp = uplanner.GenerateMotion(t);
			CLineAVP avp = bspline.GenerateMotion(uavp.pos, uavp.vel, uavp.acc);

			ofile<<avp.acc<<endl;
		}
	}
	cout<<"succeed to save the data"<<endl;

}


void Testother()
{
	VectorXd qmin;
	qmin.setZero(3);
	qmin.setConstant(1);
	VectorXd qmax;
	qmax.setZero(3);
	qmax.setConstant(5);
	VectorXd q;
	q.setZero(3);
	q<<0, 7, 3;
	cout<<q<<endl;
	MathTools::LimitVector(qmin, &q, qmax);
	cout<<q<<endl;


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
		Sleep(0.05);
	}

	return 0;
}


void Simulation()
{
	UrRobot urrbt = CreatUrRobot();
	Vector6d q_fdb;
	q_fdb << 0, 0., 0, -0, -pi / 2, 0;//initial joint position

	//add pos
	Pose pose0 = urrbt.FKSolveTool(q_fdb);
	Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
	//Vector3d rpy0 = pose0.rot.eulerAngles(0,1,2);
	//Quaterniond quat0(pose0.rot);
	TaskTrajPlanner tasktraj_planner;
	urrbt.UpdateJntHoldPos(q_fdb);

	fstream ofile;
	time_t timer = time(NULL);
	char time_format[1024];
	strftime(time_format, 1024, "%Y-%m-%d_%H.%M.%S", localtime(&timer));
	string timestring(time_format);
	string file_name = "../data/simulation2_" + timestring + ".txt";
	ofile.close();
	ofile.open(file_name, ios::out | ios::trunc);

	HANDLE thread = CreateThread(NULL, 0, ThreadProc, NULL, 0, NULL);

	if (!ofile.is_open())
	{
		cout << "failed to open the file" << endl;
	}
	else
	{
		cout << "start to simulation" << endl;
		while (true)
		{
			Vector6d q_cmd;
			if (!tasktraj_planner._task_completed)
			{
				if (strcmp(operation, "mirror") == 0)
				{
					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(&pose0.pos, &rpy0, true);
					Vector3d pos1, pos2, pos3;
					pos1 << 0.5, 0, 1;
					pos2 << 0.65, 0, 1;
					pos3 << 0.65, 0, 0.6;
					Vector3d rpy1, rpy2, rpy3;
					rpy1 << pi/2, pi/2, 0;
					rpy2 << pi / 2, pi / 2, 0;
					rpy3 << pi / 2, pi / 2, 0;
					tasktraj_planner.AddPosRPY(&pos1, &rpy1);
					tasktraj_planner.AddPosRPY(&pos2, &rpy2);
					tasktraj_planner.AddPosRPY(&pos3, &rpy3);
					*operation = {};
				}
				else if (strcmp(operation, "table") == 0)
				{

					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(&pose0.pos, &rpy0, false);
					Vector3d pos1, pos2, pos3, pos4;
					pos1 << 0.8, 0, 0.23;
					pos2 << 0.8, 0.4, 0.23;
					pos3 << 0.4, 0.4, 0.23;
					pos4 << 0.4, 0, 0.23;
					Vector3d rpy1, rpy2, rpy3, rpy4;
					rpy1 << -pi, pi / 6, 0;
					rpy2 << -5*pi/6, 0, 0;
					rpy3 << -pi, -pi / 6, 0;
					rpy4 << -7*pi/6, 0, 0;
					tasktraj_planner.AddPosRPY(&pos1, &rpy1);
					tasktraj_planner.AddPosRPY(&pos2, &rpy2);
					tasktraj_planner.AddPosRPY(&pos3, &rpy3);
					tasktraj_planner.AddPosRPY(&pos4, &rpy4);
					tasktraj_planner.AddPosRPY(&pos1, &rpy1);
					*operation = {};
				}
				else if (strcmp(operation, "washbasin") == 0)
				{
					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(&pose0.pos, &rpy0, false);
					Vector3d pos1, pos2, pos3, pos4;
					pos1 << 0.8, 0, 0.23;
					pos2 << 0.8, 0.4, 0.23;
					pos3 << 0.4, 0.4, 0.23;
					pos4 << 0.4, 0, 0.23;
					Vector3d rpy1, rpy2, rpy3, rpy4;
					rpy1 << pi / 2, pi / 2, 0;
					rpy2 << -pi/2, 0, -pi;
					rpy3 << pi / 2, -pi / 2, 0;
					rpy4 <<  pi / 2, 0, 0;
					tasktraj_planner.AddPosRPY(&pos1, &rpy1);
					tasktraj_planner.AddPosRPY(&pos2, &rpy2);
					tasktraj_planner.AddPosRPY(&pos3, &rpy3);
					tasktraj_planner.AddPosRPY(&pos4, &rpy4);
					tasktraj_planner.AddPosRPY(&pos1, &rpy1);
					*operation = {};
				}
				CAVP cavp = tasktraj_planner.GenerateMotion();
				Pose pose;
				pose.pos = cavp.pos.head(3);
				pose.rot = FixedZYX2Tr(cavp.pos.tail(3));
				int error = 0;
				error = urrbt.IKSolve(&q_cmd,&pose, &q_fdb);
				ofile << "iksolve_status: " <<error <<"\t" <<endl;
				//todo check error
				urrbt.UpdateJntHoldPos(q_cmd);
			}
			else
			{
				if ((strcmp(operation, "mirror") == 0) || (strcmp(operation, "table") == 0) || (strcmp(operation, "washbasin") == 0))
					tasktraj_planner._task_completed = false;
				q_cmd = urrbt.HoldJntPos();
				Sleep(1);
			}
			q_fdb = q_cmd;
			ofile << q_cmd(0) << "\t" << q_cmd(1) << "\t" << q_cmd(2) << "\t" << q_cmd(3) << "\t" << q_cmd(4) << "\t" << q_cmd(5) << endl;
		}
	}
	cout << "succeed to simulation and save the data" << endl;
	CloseHandle(thread);
	ofile.close();
}

void SimulationLZX()
{
	//initialize all the task data
	CleanRobot rbt = CreatRobot();
	InitialTask initial_task(&rbt);
	HoldTask hold_task(&rbt);
	char* mirror_type = "scrape";
	//MirrorTask mirror_task(&rbt, mirror_type);
	RotXPlaneTask mirror_task(&rbt);
	SurfaceTask surface_task(&rbt);
	TableTask table_task(&rbt);
	ShowerRoomTask showerroom_task(&rbt);
	vector<BaseTask*> task_data;
	task_data.push_back(&initial_task);
	task_data.push_back(&hold_task);
	task_data.push_back(&mirror_task);
	task_data.push_back(&surface_task);
	task_data.push_back(&table_task);
	task_data.push_back(&showerroom_task);
	int num_section[] = { 1,1,1,4,1,1 };

	//initialize robot joint position and running state
	MatrixXd via_pos;
	Vector5d q_fdb;
	q_fdb<<0.2, 0.5, 0.7, 0.2, 0.5;//initial joint position
	int running_state = 0, pre_state = 0;

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
				if (running_state==2)
				{
					//preparation for clean mirror task
					if (strcmp(mirror_type, "rectangle")==0)
					{
						Vector3d pos1, pos2, pos3, pos4;
						pos1<<0.4, 0.7, 0.7; pos2<<-0.4, 0.7, 0.7; pos3<<-0.4, 0.7, 1; pos4<<0.4, 0.7, 1;
						via_pos.setZero(3, 4);
						via_pos<<pos1, pos2, pos3, pos4;
					}
					else if (strcmp(mirror_type, "circle")==0)
					{
						Vector3d center(0, 0.7, 1);
						Vector3d norm_vec(0.03, 2, 0);
						double radius = 0.6;
						double interval = 0.1;
						via_pos.setZero(3, 3);
						via_pos.col(0) = center;
						via_pos.col(1) = norm_vec;
						via_pos(0, 2) = radius;
						via_pos(1, 2) = interval;
					}
					else
					{
						double pitch_x = 90*pi/180;
						Vector3d pos1, pos2, pos3;
						pos1<<g_yaw0_x0, 0.8, 1.62;
						pos2<<g_yaw0_x0, 0.8, 0.84;
						pos3<<pitch_x, 0, 0;
						via_pos.setZero(3, 3);
						via_pos<<pos1, pos2, pos3;
					}

					*operation = {};
					cout<<"start to clean the mirror"<<endl;
				}
				else if (running_state==3)
				{
					//preparation for clean washbasin task
					double npts[] = { 15,10,8,6 };
					double center[] = { 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.6-0, 0.6-0.1, 0.6-0.2, 0.6-0.3 };
					double radius[] = { 0.4, 0.3, 0.2, 0.1 };
					Matrix<double, 3, 4, RowMajor> origin(center);
					via_pos.setZero(3, 43);
					vector<Vector3d> vp;
					for (int midx = 0; midx<sizeof(npts)/sizeof(npts[0]); midx++)
					{
						VectorXd theta;
						theta.setLinSpaced(npts[midx]+1, 0, 2*pi);
						for (int nidx = 0; nidx<theta.size(); nidx++)
						{
							Vector3d tmp_pos;
							tmp_pos<<radius[midx]*cos(theta(nidx)), radius[midx]*sin(theta(nidx)), 0;
							tmp_pos += origin.col(midx);
							vp.push_back(tmp_pos);
						}
					}
					for (int idx = 0; idx<vp.size(); idx++)
					{
						via_pos.col(idx) = vp[idx];
					}

					*operation = {};
					cout<<"start to clean the surface"<<endl;
				}
				else if (running_state==4)
				{
					Vector3d pos1, pos2, pos3, pos4;
					pos1<<-0.3, 1, 0.86; pos2<<0.3, 1, 0.86; pos3<<0.3, 0.8, 0.86; pos4<<-0.3, 0.8, 0.86;
					via_pos.setZero(3, 4);
					via_pos<<pos1, pos2, pos3, pos4;

					*operation = {};
					cout<<"start to clean the table"<<endl;
				}
				else if (running_state==5)
				{
					Vector3d pos1, pos2, pos3, pos4;
					pos1<<-0.4, 0.6, 0.8; pos2<<0.4, 0.6, 0.8; pos3<<0.4, 0.6, 0.4; pos4<<-0.4, 0.6, 0.4;
					via_pos.setZero(3, 4);
					via_pos<<pos1, pos2, pos3, pos4;

					*operation = {};
					cout<<"start to clean the showerroom"<<endl;
				}

				task_data[running_state]->SetTrajPos(&via_pos, num_section[running_state], q_fdb);
				cout<<"running state: "<<running_state<<endl;
			}
			VectorXd q_cmd = task_data[running_state]->RunTask(q_fdb);
			q_fdb = q_cmd;
			ofile<<q_cmd<<endl;

			pre_state = running_state;
			Sleep(1);
		}

	}
	cout<<"succeed to simulation and save the data"<<endl;
	CloseHandle(thread);
	ofile.close();
}

void SimulationLZX1()
{
	CleanRobot rbt = CreatRobot();
	Vector5d q_fdb;
	q_fdb<<0.2, 0.5, 0.7, 0.2, 0.5;//initial joint position
	double pitch_x = 90*pi/180;
	double inc_angle[] = { 20*pi/180, 50*pi/180 };
	double pitch_high = pitch_x-inc_angle[0];
	double pitch_low = pitch_x-inc_angle[1];
	rbt.SetPitchRange(pitch_high, pitch_low);
	Vector3d pos0 = rbt.FKSolveTool(q_fdb).pos;
	double pitch0 = q_fdb(2)+rbt._tool_pitch;
	double yaw0 = q_fdb(0)+q_fdb(4);
	Vector3d rpy0(0, pitch0, yaw0);
	TaskTrajPlanner tasktraj_planner;
	rbt.UpdateJntHoldPos(q_fdb);

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
			Vector5d q_cmd;
			if (!tasktraj_planner._task_completed)
			{
				//Vector6d pos_rpy = tasktraj_planner.GeneratePoint();
				if (strcmp(operation, "mirror")==0)
				{
					pos0 = rbt.FKSolveTool(q_fdb).pos;
					pitch0 = q_fdb(2)+rbt._tool_pitch;
					yaw0 = q_fdb(0)+q_fdb(4);
					rpy0<<0, pitch0, yaw0;
					tasktraj_planner.Reset(&pos0, &rpy0, true);
					Vector3d pos1, pos2, pos3;
					pos1<<g_yaw0_x0, 0.8, 1.62;
					pos2<<g_yaw0_x0, 0.8, 0.84;
					pos3<<g_yaw0_x0, 0.8-0.1, 0.84;
					tasktraj_planner.AddPosRPY(&pos1, &Vector3d(0, rbt._pitch_high, 0));
					tasktraj_planner.AddPosRPY(&pos2, &Vector3d(0, rbt._pitch_low, 0));
					tasktraj_planner.AddPosRPY(&pos3, &Vector3d(0, rbt._pitch_low, 0));
					*operation = {};
				}
				else if (strcmp(operation, "table")==0)
				{
					pos0 = rbt.FKSolveTool(q_fdb).pos;
					pitch0 = q_fdb(2)+rbt._tool_pitch;
					yaw0 = q_fdb(0)+q_fdb(4);
					rpy0<<0, pitch0, yaw0;
					tasktraj_planner.Reset(&pos0, &rpy0, true);
					Vector3d pos1(0.5, 0.6, 0.5);
					Vector3d pos2(-0.5, 0.6, 0.5);
					Vector3d pos3(-0.5, 0.9, 0.5);
					Vector3d pos4(0.5, 0.9, 0.5);
					tasktraj_planner.AddPosRPY(&pos1, &Vector3d(0, 0, 0));
					tasktraj_planner.AddPosRPY(&pos2, &Vector3d(0, 0, 0));
					tasktraj_planner.AddPosRPY(&pos3, &Vector3d(0, 0, 0));
					tasktraj_planner.AddPosRPY(&pos4, &Vector3d(0, 0, 0));
					*operation = {};
				}
				CAVP cavp = tasktraj_planner.GenerateMotion();
				q_cmd = rbt.IKSolvePitchYaw(cavp.pos.head(3), cavp.pos(4), cavp.pos(5), q_fdb);
				rbt.UpdateJntHoldPos(q_cmd);
				//ofile<<cavp.pos<<endl;
			}
			else
			{
				if ((strcmp(operation, "mirror")==0)||(strcmp(operation, "table")==0))
					tasktraj_planner._task_completed = false;
				q_cmd = rbt.HoldJntPos();
				Sleep(1);
			}
			q_fdb = q_cmd;
			ofile<<q_cmd<<endl;
		}
	}


}

#include "InterruptStop/InterruptStop.h"
#include "RobotControl.h"
static RobotControl g_robot;

void Simulation_TestRobotControl()
{
	int jointnum = g_robot.CreatRobot();
	Vector6d q_fdb, q_cmd;
	q_fdb << 0.2, 0.5, 0.7, 0.2, 0.5, 0;//initial joint position

	time_t timer = time(NULL);
	char time_format[1024];
	strftime(time_format, 1024, "%Y-%m-%d_%H.%M.%S", localtime(&timer));
	string timestring(time_format);
	string file_name = "/mnt/hgfs/VMShare/traj_pos" + timestring + ".txt";

	ofstream ofile;
	ofile.open(file_name, ios::out | ios::trunc);

	int mainstate = 0;  //主状态
	int runstate = 0;   //子状态

	//测试时用的状态量，实际时由外部给信号
	static int counttest_num = 0; int pause = 0;  //test

	//暂停处理
	double joint_vel[6];
	InterruptStop interrupt_data(g_cycle_time);
	//规划处理
	TaskTrajPlanner tasktraj_planner;

	g_robot.UpdateJointHoldPos(q_fdb);

	while (1)
	{
		switch (mainstate)
		{
		case 0: //standby
		{
			q_cmd = q_fdb;
			//根据不同的任务，计算点位，添加路径
			int tasknum = 2; //test

			int change_toolflag = (tasknum == 1);//test  测试修改工具
			if (change_toolflag)
			{
				Vector3d toolpos, toolrpy;
				toolpos << g_tool_mirror[0], g_tool_mirror[1], g_tool_mirror[2];
				toolrpy << g_tool_mirror[3], g_tool_mirror[4], g_tool_mirror[5];

				g_robot.UpdateTool(toolpos, toolrpy);
			}

			MatrixXd via_pos;
			Vector3d posn[50], rpyn[50], seg_optn[50];

			if (tasknum == 1) //擦玻璃数据
			{
				//preparation for clean mirror task
				Vector3d pos1, pos2, pos3, pos4;
				pos1 << g_yaw0_x0, 0.7, 1.52;
				pos2 << g_yaw0_x0, 0.92, 1.52;
				pos3 << g_yaw0_x0, 0.92, 0.84;
				pos4 << g_yaw0_x0, 0.7, 0.84;
				int posnum = 4;
				via_pos.setZero(3, posnum);
				via_pos << pos1, pos2, pos3, pos4;

				//上面都是测试数据

				//via_pos由相机提供，然后内部做一层转化处理，转为cal_via_pos
				MatrixXd cal_via_pos = g_robot.CalTrajViaPos(&via_pos, q_fdb);

				Vector3d pos0 = cal_via_pos.col(0), rpy0 = cal_via_pos.col(1);
				bool conti_type = cal_via_pos.col(2)(0) > 0; //path smooth flag

				tasktraj_planner.Reset(&pos0, &rpy0, conti_type);  //add startpos

				for (int i = 0; i < cal_via_pos.cols() / 3 - 1; i++)
				{
					posn[i] = cal_via_pos.col((i + 1) * 3), rpyn[i] = cal_via_pos.col((i + 1) * 3 + 1), seg_optn[i] = cal_via_pos.col((i + 1) * 3 + 2);
					tasktraj_planner.AddPosRPY(&posn[i], &rpyn[i]);
				}
			}
			else if (tasknum == 2) //擦桌子数据
			{
				posn[0] << -0.2, 0.8, 0.65; posn[1] << 0.2, 0.8, 0.65; posn[2] << 0.2, 0.65, 0.65; posn[3] << -0.2, 0.65, 0.65;

				int posnum = 4; //点位个数根据界面设置值进行修改
				via_pos.setZero(3, posnum);

				for (int i = 0; i < posnum; ++i)
				{
					via_pos.col(i) = posn[i];
				}

				cout << "start to clean the table" << endl;
				//上面都是测试数据

				MatrixXd cal_via_pos = g_robot.CalTrajViaPos_AbsolatePos(&via_pos, q_fdb);

				Vector3d pos0 = cal_via_pos.col(0), rpy0 = cal_via_pos.col(1);
				bool conti_type = cal_via_pos.col(2)(0) > 0; //path smooth flag

				tasktraj_planner.Reset(&pos0, &rpy0, conti_type);  //add startpos

				for (int i = 0; i < cal_via_pos.cols() / 3 - 1; i++)
				{
					posn[i] = cal_via_pos.col((i + 1) * 3), rpyn[i] = cal_via_pos.col((i + 1) * 3 + 1), seg_optn[i] = cal_via_pos.col((i + 1) * 3 + 2);
					tasktraj_planner.AddPosRPY(&posn[i], &rpyn[i]);
				}
			}
			else if (tasknum == 3)  //自测擦镜子添加点位数据
			{
				//add pos
				double pitch_x = 90 * pi / 180;
				double inc_angle[] = { 20 * pi / 180, 50 * pi / 180 };
				double pitch_high = pitch_x - inc_angle[0];
				double pitch_low = pitch_x - inc_angle[1];
				//                rbt.SetPitchRange(pitch_high, pitch_low);
				Vector3d pos0, rpy0;
				g_robot.FKSolveTool(q_fdb, pos0, rpy0);

				Vector3d pos1, pos2, pos3, pos4;
				pos1 << g_yaw0_x0, 0.8 - 0.3, 1.52;  //g_yaw0_x0 = 0.0275;
				pos2 << g_yaw0_x0, 0.8, 1.52;
				pos3 << g_yaw0_x0, 0.8, 0.84;
				pos4 << g_yaw0_x0, 0.8 - 0.3, 0.84;

				Vector3d rpy1, rpy2, rpy3, rpy4;
				rpy1 << 0, pitch_high, 0;
				rpy2 << 0, pitch_high, 0;
				rpy3 << 0, pitch_low, 0;
				rpy4 << 0, pitch_low, 0;

				tasktraj_planner.Reset(&pos0, &rpy0, false);  //add startpos
				tasktraj_planner.AddPosRPY(&pos1, &rpy1);
				tasktraj_planner.AddPosRPY(&pos2, &rpy2);
				tasktraj_planner.AddPosRPY(&pos3, &rpy3);
				tasktraj_planner.AddPosRPY(&pos4, &rpy4);
			}

			//跳主状态running.moving
			mainstate = 1;
			runstate = 2;
		}
		break;
		case 1: //running
			switch (runstate)
			{
			case 1:  //interrupt
				interrupt_data.InitInterruptData(g_jamax, joint_vel, q_cmd);

				counttest_num++;  //test
				pause = 0;

				//暂停规划过程中
				if (interrupt_data.GetInterruptStopPlanState())
				{
					q_cmd = interrupt_data.RefreshInterruptStopPlan(q_fdb);
					g_robot.UpdateJointHoldPos(q_cmd);

					if (interrupt_data.GetInterruptStopPlanState() == eInterruptPlanFinished)
					{						
						//外部触发重新开始
						int resume = 0;  //test						

						if (resume == 1)
						{
							//reinit tasktraj_planner and trans to moving
							Vector3d pos0, rpy0;
							g_robot.FKSolveTool(q_fdb, pos0, rpy0);

							tasktraj_planner.ReInitSegTrajPlanner(pos0, rpy0);

							interrupt_data.InitInterruptData(0);

							runstate = 2; //moving
						}
						else if (resume == 2)  //直接放弃该任务，则跳入standby
						{
							interrupt_data.InitInterruptData(0);
							tasktraj_planner.Reset(false);
							mainstate = 0;
						}
					}
				}

				break;
			case 2:  //moving
				CAVP cavp = tasktraj_planner.GenerateMotion();
				g_robot.TranCavpToJointPos(cavp, q_fdb, q_cmd);

				cout << q_cmd << endl;				

				//所有路径结束，跳回主状态standby；接着判断是否有任务过来，在standby里去添加路径
				if (tasktraj_planner.GetSegTrajPlannerDone())
				{
					mainstate = 2;
				}
				else if (pause)//外部触发pause
				{
					runstate = 1;  //interrupt
				}

				break;
			}

			break;
		case 2: //errorstop
			break;
		default:
			break;
		}

		for (int i = 0; i < 6; i++)
			joint_vel[i] = (q_cmd[i] - q_fdb[i]) / g_cycle_time;

		q_fdb = q_cmd;
		ofile << q_cmd[0] << "\t" << q_cmd[1] << "\t" << q_cmd[2] << "\t" << q_cmd[3] << "\t" << q_cmd[4] << endl;
	}
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

	case ctrajarctrans:
		Testctrajarctrans();
		break;

	case iksolver:
		Testiksolver();
		break;

	case jacobian:
		Testjacobian();
		break;

	case bspline:
		TestBSpline();
		break;

	case simulation:
#if defined(lzx)
		SimulationLZX1();
#elif defined(wyc)
		Simulation();
#elif defined(xnd)
		Simulation_TestRobotControl();
#endif // lzx
		break;

	case other:
		Testother();
		break;
	}

    return 0;
}

