// test.cpp : 定义控制台应用程序的入口点。
//

#include <Windows.h>
#include <conio.h>
#include "GlobalParams.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <time.h>
#include <map>

#include "TaskTrajPlanner.h"
#include "RobotDynIdenTest.h"
#include "InterruptStop/InterruptStop.h"
#include "RobotParameter/RobotParameter.h"
#include "RobotControl.h"

static RobotControl g_robot;

#pragma warning(disable : 4996)
#define lzx
//#define wyc
//#define xnd
//#define urrobot
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

typedef void(*PtrTest)();
struct TestInfo
{
	ID_test testID;
	PtrTest testfunc;
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
	pose_tool.pos << 0, 0, 0.29;
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

#ifdef DEBUG
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
	LspbPlanner planner(pos, 16, 10, duration);
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
	LspbPlanner planner(via_path, 0.5, 2);
	RobotTools::JAVP javp = planner.GenerateMotion(10);
	RobotTools::CLineAVP avp = cpath.GenerateMotion(javp.pos, javp.vel, javp.acc);
	cout<<avp.pos<<endl;
	cout<<avp.vel<<endl;
	cout<<avp.acc<<endl;
}

#endif

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
	int np = (int)(tf/dt)+1;
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
			Vector6d cpos;
			bspline.GeneratePath(cpos);

			ofile<<cpos<<endl;
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
	TaskTrajPlanner tasktraj_planner(&urrbt, q_fdb);
	MatrixXd via_pos;
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
					tasktraj_planner.Reset(q_fdb);
					Vector6d pos_rpy1, pos_rpy2, pos_rpy3;
					pos_rpy1 << 0.5, 0, 1, pi/2, pi/2, 0;
					pos_rpy2 << 0.65, 0, 1, pi/2, pi/2, 0;
					pos_rpy3 << 0.65, 0, 0.6, pi/2, pi/2, 0;
					via_pos.resize(6, 3);
					via_pos<<pos_rpy1, pos_rpy2, pos_rpy3;
					tasktraj_planner.AddTraj(&via_pos, eCartesianSpace, true);
					*operation = {};
				}
				else if (strcmp(operation, "table") == 0)
				{
		
					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(q_fdb);
					Vector3d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					pos_rpy1 << 0.8, 0, 0.23, -pi, pi/6, 0;
					pos_rpy2 << 0.8, 0.4, 0.23, -5*pi/6, 0, 0;
					pos_rpy3 << 0.4, 0.4, 0.23, -pi, -pi/6, 0;
					pos_rpy4 << 0.4, 0, 0.23, -7*pi/6, 0, 0;
					via_pos.resize(6, 4);
					tasktraj_planner.AddTraj(&via_pos,eCartesianSpace, true);
					*operation = {};
				}
				else if (strcmp(operation, "washbasin") == 0)
				{
					Pose pose0 = urrbt.FKSolveTool(q_fdb);
					Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
					tasktraj_planner.Reset(q_fdb);
					Vector3d pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
					pos_rpy1 << 0.8, 0, 0.23, pi/2, pi/2, 0;
					pos_rpy2 << 0.8, 0.4, 0.23, -pi/2, 0, -pi;
					pos_rpy3 << 0.4, 0.4, 0.23, pi/2, -pi/2, 0;
					pos_rpy4 << 0.4, 0, 0.23, pi/2, 0, 0;
					via_pos.resize(6, 4);
					tasktraj_planner.AddTraj(&via_pos, eCartesianSpace, true);
					*operation = {};
				}
				tasktraj_planner.GenerateJPath(q_cmd);
				int error = 0;
				//error = urrbt.IKSolve(&q_cmd,&pose, &q_fdb);
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
	UrRobot rbt = CreatUrRobot();
	Vector6d q_fdb;
	q_fdb<<0,-35,50,-100,-90,0;//initial joint position
	q_fdb = q_fdb*pi/180;
	TaskTrajPlanner task_planner(&rbt, q_fdb);
	rbt.UpdateJntHoldPos(q_fdb);

	string file_name = "D:/00Work/01projects/XProject/src/data/mirrortask_jpos1.csv";
	ofstream ofile;
	ofile.open(file_name, ios::out|ios::trunc);
	HANDLE thread = CreateThread(NULL, 0, ThreadProc, NULL, 0, NULL);
	//***toilet_lid task plan***//
	Vector3d a, b, c;
	a<<0.815, 0.431, -0.2876;
	b<<0.7674, 0.1089, -0.3071;
	c<<0.6015, 0.1089, -0.3071;
	MatrixXd vi_pos;
	vi_pos.resize(3, 3);
	vi_pos<<a, b, c;
	MatrixXd vp;
	PlanTools::CalcToiletlidPath(vp, &vi_pos, pi/2);
	//task_planner.AddTraj(&vp, eCartesianArc, false);
	//***joint plan for grasp&interim***//
	Vector6d q1, q2;
	q1<<0, 145, -240, 90, -90, 0;
	q2<<0, -35, 50, -100, -90, 0;
	q1 *= pi/180;
	q2 *= pi/180;
	MatrixXd via_pos;
	via_pos.setZero(6, 2);
	via_pos<<q1, q2;
	//task_planner.AddTraj(&via_pos, eJointSpace, true);
	//***mirror task plan***//
	Vector3d pos1, pos2, pos3, pos4;
	pos1<<0.8, -0.1, 0.45;
	pos2<<0.8, 0.4, 0.45;
	pos3<<0.8, 0.4, 0.81;
	pos4<<0.8, -0.1, 0.81;
	via_pos.resize(3, 4);
	via_pos<<pos1, pos2, pos3, pos4;
	MatrixXd via_posrpy;
	//PlanTools::CalcMirrorPath_Normal(via_posrpy, &via_pos, 0.15, 60*pi/180, 0.08);
	PlanTools::CalcMirrorPath(via_posrpy, &via_pos, 0.15, 10*pi/180, 50*pi/180);
	cout<<via_posrpy<<endl;
	task_planner.AddTraj(&via_posrpy, eCartesianSpace, false);
	//***table task plan***//
	via_pos.resize(6, 1);
	via_pos<<0, -35, 50, -105, -90, 0;
	via_pos *= pi/180;
	//task_planner.AddTraj(&via_pos, eJointSpace, true);
	Vector6d pos_rpy1, pos_rpy2, pos_rpy3;
	pos_rpy1<<0.8, 0, 0.23, -pi, pi/6, 0;
	pos_rpy2<<0.8, 0.4, 0.23, -5*pi/6, 0, 0;
	pos_rpy3<<0.4, 0.4, 0.23, -pi, -pi/6, 0;
	Vector6d pos_rpy4;
	pos_rpy4<<0.4, 0, 0.23, -7*pi/6, 0, 0;
	via_pos.resize(6, 4);
	via_pos<<pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4;
	//task_planner.AddTraj(&via_pos, eCartesianSpace, true);
	//***toilet task plan***//
	q1<<0, -35, 50, -105, -90, 0;
	q2<<0.35, 0.52, 0.52, -1.1, -1.4, 0.52;
	q1 *= pi/180;
	via_pos.resize(6, 2);
	via_pos<<q1, q2;
	//task_planner.AddTraj(&via_pos, eJointSpace, true);
	Vector6d pos_rpy5;
	pos_rpy1<<0.5297, 0.2516, -0.4929, -175*pi/180, 0.3*pi/180, -106*pi/180;
	pos_rpy2<<0.5208, 0.2905, -0.5424, -170*pi/180, -0.8*pi/180, -104.7*pi/180;
	pos_rpy3<<0.6039, 0.4115, -0.544, -168*pi/180, 3.7*pi/180, -114.8*pi/180;
	pos_rpy4<<0.7013, 0.3362, -0.5544, -177.7*pi/180, 8.6*pi/180, -113*pi/180;
	pos_rpy5<<0.6396, 0.2582, -0.567, -179.4*pi/180, 4.4*pi/180, -109.2*pi/180;
	via_pos.resize(6, 5);
	via_pos<<pos_rpy1, pos_rpy2, pos_rpy3, pos_rpy4, pos_rpy5;
	//task_planner.AddTraj(&via_pos, eBSpline, true);

	Vector6d q_traj;
	while (!task_planner._task_completed)
	{
		task_planner.GenerateJPath(q_traj);
		ofile<<q_traj<<endl;
	}

	if (!ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
	}
	else
	{
		cout<<"start to simulation"<<endl;
		//while (true)
		//{
		//	Vector6d q_cmd;
		//	if (!tasktraj_planner._task_completed)
		//	{
		//		if (strcmp(operation, "mirror")==0)
		//		{
		//			Pose pose0 = rbt.FKSolveTool(q_fdb);
		//			Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
		//			tasktraj_planner.Reset(&pose0.pos, &rpy0, true);
		//			Vector3d pos1, pos2, pos3;
		//			pos1<<0.5, 0, 1;
		//			pos2<<0.65, 0, 1;
		//			pos3<<0.65, 0, 0.6;
		//			Vector3d rpy1, rpy2, rpy3;
		//			rpy1<<pi/2, pi/2, 0;
		//			rpy2<<pi/2, pi/2, 0;
		//			rpy3<<pi/2, pi/2, 0;
		//			//vector<Vector3d*> pos_corner, rpy_corner;
		//			//pos_corner.push_back(&pos1);
		//			//pos_corner.push_back(&pos2);
		//			//pos_corner.push_back(&pos3);
		//			//rpy_corner.push_back(&rpy1);
		//			//rpy_corner.push_back(&rpy2);
		//			//rpy_corner.push_back(&rpy3);
		//			//tasktraj_planner.AddPosRPY(&pos_corner, &rpy_corner);
		//			tasktraj_planner.AddPosRPY(&pos1, &rpy1);
		//			tasktraj_planner.AddPosRPY(&pos2, &rpy2);
		//			tasktraj_planner.AddPosRPY(&pos3, &rpy3);
		//			*operation = {};
		//		}
		//		else if (strcmp(operation, "table")==0)
		//		{
		//			Pose pose0 = rbt.FKSolveTool(q_fdb);
		//			Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
		//			tasktraj_planner.Reset(&pose0.pos, &rpy0, false);
		//			Vector3d pos1, pos2, pos3, pos4;
		//			pos1<<0.8, 0, 0.23;
		//			pos2<<0.8, 0.4, 0.23;
		//			pos3<<0.4, 0.4, 0.23;
		//			pos4<<0.4, 0, 0.23;
		//			Vector3d rpy1, rpy2, rpy3, rpy4;
		//			rpy1<<-pi, pi/6, 0;
		//			rpy2<<-5*pi/6, 0, 0;
		//			rpy3<<-pi, -pi/6, 0;
		//			rpy4<<-7*pi/6, 0, 0;
		//			tasktraj_planner.AddPosRPY(&pos1, &rpy1);
		//			tasktraj_planner.AddPosRPY(&pos2, &rpy2);
		//			tasktraj_planner.AddPosRPY(&pos3, &rpy3);
		//			tasktraj_planner.AddPosRPY(&pos4, &rpy4);
		//			tasktraj_planner.AddPosRPY(&pos1, &rpy1);
		//			*operation = {};
		//		}
		//		else if (strcmp(operation, "arc")==0)
		//		{
		//			Vector6d pos_ryp1, pos_rpy3;
		//			Vector3d pos2;
		//			pos_ryp1<<0.3, 1.2, -0.1, 0, 0, 0;
		//			pos_rpy3<<-0.4, 0, 0.8, 0, 0, 0;
		//			pos2<<0, 0.4, 0.2;
		//			tasktraj_planner.Reset(&pos_ryp1, &pos2, &pos_rpy3);
		//			*operation = {};
		//		}
		//		else if(strcmp(operation,"toilet")==0)
		//		{
		//			tasktraj_planner.Reset(RobotTools::eBSpline);
		//			Pose pose0 = rbt.FKSolveTool(q_fdb);
		//			Vector3d rpy0 = RobotTools::Tr2FixedZYX(pose0.rot);
		//			Vector3d pos1, pos2, pos3, pos4, pos5;
		//			Vector3d rpy1, rpy2, rpy3, rpy4, rpy5;
		//			pos1<<0.5297, 0.2516, -0.4929;
		//			rpy1<<-175.34*pi/180, 0.3123*pi/180, -106.302*pi/180;
		//			pos2<<0.5208, 0.2905, -0.5424;
		//			rpy2<<-169.71*pi/180, -0.7918*pi/180, -104.718*pi/180;
		//			pos3<<0.6039, 0.4115, -0.544;
		//			rpy3<<-167.95*pi/180, 3.7387*pi/180, -114.773*pi/180;
		//			pos4<<0.7013, 0.3362, -0.5544;
		//			rpy4<<-177.72*pi/180, 8.613*pi/180, -113*pi/180;
		//			pos5<<0.6396, 0.2582, -0.567;
		//			rpy5<<-179.4*pi/180, 4.39*pi/180, -109.2*pi/180;
		//			vector<Vector3d*> pos_corner, rpy_corner;
		//			pos_corner.push_back(&pose0.pos);
		//			pos_corner.push_back(&pos1);
		//			pos_corner.push_back(&pos2);
		//			pos_corner.push_back(&pos3);
		//			pos_corner.push_back(&pos4);
		//			pos_corner.push_back(&pos5);
		//			rpy_corner.push_back(&rpy0);
		//			rpy_corner.push_back(&rpy1);
		//			rpy_corner.push_back(&rpy2);
		//			rpy_corner.push_back(&rpy3);
		//			rpy_corner.push_back(&rpy4);
		//			rpy_corner.push_back(&rpy5);
		//			tasktraj_planner.AddPosRPY(&pos_corner, &rpy_corner);
		//			*operation = {};
		//		}
		//		//CAVP cavp = tasktraj_planner.GenerateMotion();
		//		//Pose pose;
		//		//pose.pos = cavp.pos.head(3);
		//		//pose.rot = FixedZYX2Tr(cavp.pos.tail(3));
		//		//rbt.IKSolve(&q_cmd, &pose, &q_fdb);
		//		////q_cmd = rbt.IKSolvePitchYaw(cavp.pos.head(3), cavp.pos(4), cavp.pos(5), q_fdb);
		//		//rbt.UpdateJntHoldPos(q_cmd);
		//		//ofile<<cavp.pos<<endl;
		//	}
		//	else
		//	{
		//		if ((strcmp(operation, "mirror")==0)||(strcmp(operation, "table")==0)
		//			||(strcmp(operation, "arc")==0)||(strcmp(operation, "toilet")==0))
		//			tasktraj_planner._task_completed = false;
		//		q_cmd = rbt.HoldJntPos();
		//		Sleep(1);
		//	}
		//	q_fdb = q_cmd;
		//	ofile<<q_cmd<<endl;
		//}
	}
}

void RunDynIdenTraj()
{
	Vector6d q_fdb;
	q_fdb<<0.0, -0.0, 0.0, -0.0, 0.0, 0;
	cout<<"start to simulation"<<endl;
	string freq[2] = { "0.5","1" };
	for (int idx = 0; idx<2; idx++)
	{
		string file_read = "../data/traj_"+freq[idx]+"hz.csv";
		RobotDynIdenTest dyniden_test(5*(idx+1), q_fdb, file_read);
		//Vector6d q_cmd;
		//while (!dyniden_test._traj_completed)
		//{
		//	q_cmd = dyniden_test.GenerateTraj();
		//	q_fdb = q_cmd;
		//}
	}
}


void Simulation_TestRobotControl()
{
	int jointnum = g_robot.CreatRobot();
	const double D2R = pi / 180.0;
	cout << "current robot:" << jointnum << endl;
	Vector6d q_fdb, q_cmd;
	q_fdb << 14 * D2R, -33.0* D2R, 33.5*D2R, -30 * D2R, -90 * D2R, 0;

	time_t timer = time(NULL);
	char time_format[1024];
	strftime(time_format, 1024, "%Y-%m-%d_%H.%M.%S", localtime(&timer));
	string timestring(time_format);
	string file_name = "/mnt/hgfs/VMShare/kinematics_mdh/kinematics_mdh/traj_pos_ur" + timestring + ".txt";

	ofstream ofile;
	ofile.open(file_name, ios::out | ios::trunc);

	//测试时用的状态量，实际时由外部给信号
	static int counttest_num = 0; int pause = 0;  //test
	int mainstate = 0;  //主状态
	int runstate = 0;   //子状态

	TaskTrajPlanner tasktraj_planner;
	Vector3d posn[50], rpyn[50];
	Vector6d jntpos[50];

	//暂停处理
	double joint_vel[6];
	InterruptStop interrupt_data(g_cycle_time);

	g_robot.UpdateJointHoldPos(q_fdb);

	while (1)
	{
		switch (mainstate)
		{
		case 0: //standby
		{
			q_cmd = q_fdb;

			//test 转化相机数据
			Vector3d camer_pos[4], camer_rpy[4];
			//toilet
			camer_pos[0] << -0.60848, -0.706677, 0.53975; //pos_right_top
			camer_pos[1] << 0.1528, 0.3145, 0.6677;;    //lt
			camer_pos[2] << 0.1485, 0.0604, 0.5774; //pos_left_bot
			camer_pos[3] << -0.3372, 0.05521, 0.577; //pos_right_bottom													
			camer_rpy[0] << 0, 0, 0;
			camer_rpy[3] = camer_rpy[2] = camer_rpy[1] = camer_rpy[0];

			for (int i = 0; i < 4; i++)
				g_robot.TranCamerPosToRobotBase(q_fdb, camer_pos[i], camer_rpy[i], posn[i], rpyn[i]);				

			Vector3d temp;
			temp << rpyn[0][0] * 180 / pi, rpyn[0][1] * 180 / pi, rpyn[0][2] * 180 / pi;

			//对相机给的转为base后位置，做保护限制
			Vector3d max_xyz(1.0, 0.5, 1.05), min_xyz(0.4, -0.5, -0.05);
			for (int i = 0; i < 4; i++)
			{
				VectorXd temp_q_pos = posn[i];
				MathTools::LimitVector(min_xyz, &temp_q_pos, max_xyz);
				posn[i] = temp_q_pos;
			}
			//test 转化相机数据end			

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
							//tasktraj_planner.ReInitSegTrajPlanner(pos0, rpy0, posn, rpyn);
							interrupt_data.InitInterruptData(0);

							runstate = 2; //moving
						}
						else if (resume == 2)  //直接放弃该任务，则跳入standby
						{
							interrupt_data.InitInterruptData(0);
							//tasktraj_planner.Reset(false);
							mainstate = 0;
						}
					}
				}

				break;
			case 2:  //moving
				tasktraj_planner.GenerateJPath(q_cmd);					

				cout << q_cmd << endl;

				counttest_num++;  //test

				//所有路径结束，跳回主状态standby；接着判断是否有任务过来，在standby里去添加路径
				if (tasktraj_planner._task_completed)
				{
					mainstate = 0;
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
		ofile << q_cmd[0] << "\t" << q_cmd[1] << "\t" << q_cmd[2] << "\t" << q_cmd[3] << "\t" << q_cmd[4] << "\t" << q_cmd[5] << endl;
	}
}


int main()
{
	map<ID_test, PtrTest> test_map;
	TestInfo config[] = 
	{
		{dhmodel,	Testdhmodel},
		{other,		Testother},
		{iksolver,	Testiksolver},
		{jacobian,	Testjacobian},
		{bspline,	TestBSpline}
	};
	int test_count = sizeof(config)/sizeof(*config);
	while (test_count--)
		test_map[config[test_count].testID] = config[test_count].testfunc;

#if defined(lzx)
	test_map[simulation] = SimulationLZX;
#elif defined(wyc)
	test_map[simulation] = Simulation;
#elif defined(xnd)
	test_map[simulation] = Simulation_TestRobotControl;
#endif

	ID_test test_mode = simulation;
	test_map[test_mode]();

    return 0;
}

