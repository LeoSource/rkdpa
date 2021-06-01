// test.cpp : 定义控制台应用程序的入口点。
//

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
#include "SurfaceTask.h"
#include "RotXPlaneTask.h"
#include "TableTask.h"
#include "ShowerRoomTask.h"
#include <time.h>
#include "TaskTrajPlanner.h"

#pragma warning(disable : 4996)
#define lzx

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
	bspline = 11,
	other = 12,
	simulation = 13
};

CleanRobot CreatRobot()
{
	Matrix<double, 5, 4, RowMajor> mdh_table(g_dh);
	Matrix<int, 5, 1> jnt_type(g_type);
	Matrix<double, 5, 1> offset(g_offset);
	double tool_pitch = 30*pi/180;
	Vector3d tool_pos(0, 0.2196, -0.05578);
	RobotTools::Pose pose_tool;
	pose_tool.rot.setIdentity();
	pose_tool.pos<<0, 0.2196, -0.05578;
	Matrix<double, 5, 2, RowMajor> qlimit(g_qlimit);
	//CleanRobot rbt(mdh_table, jnt_type, offset, pose_tool);
	CleanRobot rbt(mdh_table, jnt_type, offset, tool_pitch, tool_pos);
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

void Testctrajarc()
{
	Vector3d pos1, pos2, pos3;
	pos1<<0, -2, 0; pos2<<1, 0, 1; pos3<<0, 3, 3;
	ArcPathPlanner arcpath(pos1, pos2, pos3);
	Vector2d via_pos(0, arcpath._theta);
	double tf = 2;
	LspbTrajPlanner planner(via_pos, 2, 2);
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
	LspbTrajPlanner planner(via_pos, 4, 2);
	RobotTools::JAVP javp = planner.GenerateMotion(2);
	RobotTools::CLineAVP avp = circlepath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	cout<<avp.pos<<endl;
	cout<<avp.vel<<endl;
	cout<<avp.acc<<endl;
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
	//initialize all the task data
	CleanRobot rbt = CreatRobot();
	InitialTask initial_task(&rbt);
	HoldTask hold_task(&rbt);
	char* mirror_type = "rectangle";
	MirrorTask mirror_task(&rbt, mirror_type);
	SurfaceTask surface_task(&rbt);
	TableTask table_task(&rbt,true);
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
	Matrix<double, 5, 1> q_fdb;
	q_fdb << 0.4, 0.8, 0.5, 0.1, 0;//initial joint position
	int running_state = 0, pre_state = 0;

	fstream ofile;
	time_t timer = time(NULL);
	char time_format[1024];
	strftime(time_format, 1024, "%Y-%m-%d_%H.%M.%S", localtime(&timer));
	string timestring(time_format);
	string file_name = "C:/Users/Administrator/Desktop/XRobotAlgorithm/data/simulation2_" + timestring + ".txt";
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
			running_state = task_data[running_state]->RunLogicOperation(running_state, pre_state, operation);
			if (running_state != pre_state)
			{
				if (running_state == 2)
				{
					//preparation for clean mirror task
					if (strcmp(mirror_type, "rectangle") == 0)
					{
						Vector3d pos1, pos2, pos3, pos4;
						pos1 << 0.7, 0.8, 1; pos2 << -0.7, 0.8, 1; pos3 << -0.7, 0.8, 2.4; pos4 << 0.7, 0.8, 2.4;
						via_pos.setZero(3, 4);
						via_pos << pos1, pos2, pos3, pos4;
					}
					else if (strcmp(mirror_type, "circle"))
					{
						Vector3d center(0, 0.7, 1);
						double radius = 0.6;
						double interval = 0.1;
						via_pos.setZero(3, 2);
						via_pos.col(0) = center;
						via_pos(0, 1) = radius;
						via_pos(1, 1) = interval;
					}

					*operation = {};
					cout << "start to clean the mirror" << endl;
				}
				else if (running_state == 3)
				{
					//preparation for clean washbasin task
					double npts[] = { 15,10,8,6 };
					double center[] = { 0, 0, 0, 0, 0.7, 0.7, 0.7, 0.7, 0.6 - 0, 0.6 - 0.1, 0.6 - 0.2, 0.6 - 0.3 };
					double radius[] = { 0.4, 0.3, 0.2, 0.1 };
					Matrix<double, 3, 4, RowMajor> origin(center);
					via_pos.setZero(3, 43);
					vector<Vector3d> vp;
					for (int midx = 0; midx<sizeof(npts) / sizeof(npts[0]); midx++)
					{
						VectorXd theta;
						theta.setLinSpaced(npts[midx] + 1, 0, 2 * pi);
						for (int nidx = 0; nidx<theta.size(); nidx++)
						{
							Vector3d tmp_pos;
							tmp_pos << radius[midx] * cos(theta(nidx)), radius[midx] * sin(theta(nidx)), 0;
							tmp_pos += origin.col(midx);
							vp.push_back(tmp_pos);
						}
					}
					for (int idx = 0; idx<vp.size(); idx++)
					{
						via_pos.col(idx) = vp[idx];
					}

					*operation = {};
					cout << "start to clean the surface" << endl;
				}
				else if (running_state == 4)
				{
					Vector3d pos1, pos2, pos3, pos4;
					pos1 << -0.3, 1, 0.86; pos2 << 0.3, 1, 0.86; pos3 << 0.3, 0.8, 0.86; pos4 << -0.3, 0.8, 0.86;
					via_pos.setZero(3, 4);
					via_pos << pos1, pos2, pos3, pos4;

					*operation = {};
					cout << "start to clean the table" << endl;
				}
				else if (running_state == 5)
				{
					Vector3d pos1, pos2, pos3, pos4;
					pos1 << -0.4, 0.6, 0.8; pos2 << 0.4, 0.6, 0.8; pos3 << 0.4, 0.6, 0.4; pos4 << -0.4, 0.6, 0.4;
					via_pos.setZero(3, 4);
					via_pos << pos1, pos2, pos3, pos4;

					*operation = {};
					cout << "start to clean the showerroom" << endl;
				}

				task_data[running_state]->SetTrajPos(&via_pos, num_section[running_state], q_fdb);
				cout << "running state: " << running_state << endl;
			}
			VectorXd q_cmd = task_data[running_state]->RunTask(q_fdb);
			q_fdb = q_cmd;
			ofile << q_cmd(0) << "\t" << q_cmd(1) << "\t" << q_cmd(2) << "\t" << q_cmd(3) << "\t" << q_cmd(4) << endl;

			pre_state = running_state;
			Sleep(0.01);
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
	TaskTrajPlanner tasktraj_planner(pos0, rpy0);

	Vector3d pos1, pos2, pos3;
	pos1<<g_yaw0_x0, 0.8, 1.62;
	pos2<<g_yaw0_x0, 0.8, 0.84;
	pos3<<g_yaw0_x0, 0.8-0.1, 0.84;
	tasktraj_planner.AddPosRPY(pos1, Vector3d(0, rbt._pitch_high, 0), "both");
	tasktraj_planner.AddPosRPY(pos2, Vector3d(0, rbt._pitch_low, 0), "both");
	tasktraj_planner.AddPosRPY(pos3, Vector3d(0, rbt._pitch_low, 0), "pos");

	const char* file_name = "C:/00Work/01projects/XProject/src/data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);

	while (true)
	{
		Vector6d pos_rpy = tasktraj_planner.GeneratePoint();
		Vector5d q_cmd = rbt.IKSolveYaw(pos_rpy.head(3), pos_rpy(4), pos_rpy(5), q_fdb);

		q_fdb = q_cmd;
		ofile<<q_cmd<<endl;
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

	case bspline:
		TestBSpline();
		break;

	case simulation:
#ifdef lzx
		SimulationLZX1();
#else
		Simulation();
#endif // lzx
		break;

	case other:
		Testother();
		break;
	}

    return 0;
}

