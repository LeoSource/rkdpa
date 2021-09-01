#include "RobotDynIdenTest.h"

using namespace std;

RobotDynIdenTest::RobotDynIdenTest(int order, Vector6d q_cur, string filepath):_q_cur(q_cur),_order(order)
{
	_w0 = 2*pi*0.1;
	_q_b.setOnes(2*order+1);
	_qd_b.setZero(2*order+1);
	_qdd_b.setZero(2*order+1);
	_t = 0;
	_traj_completed = false;
	_traj_id<<1, 2, 3, 4, 5;
	LoadTrajParams(filepath);
	double vmax[] = { 1, 1, 1, 1, 1, 1 };
	double amax[] = { 2, 2, 2, 2, 2, 2 };
	_traj_idx = 0;
	_jtraj_planner = JointPlanner(q_cur, true);
	_jtraj_planner.SetKineConstraint(vmax, amax);
	MatrixXd via_jpos;
	via_jpos.setZero(6, 1);
	via_jpos = _q0[_traj_id[_traj_idx]];
	_jtraj_planner.AddViaPos(&via_jpos);
	//_jtraj_planner = JointPlanner(q_cur, _q0[_traj_id[_traj_idx]], vmax, amax, true);
}

void RobotDynIdenTest::LoadTrajParams(string filepath)
{
	ifstream fp(filepath);
	string line;
	int traj_idx = 0;
	while (getline(fp, line))
	{
		string num;
		istringstream readnum(line);
		vector<double> jnt_traj_params;
		while (getline(readnum, num, ','))
		{
			double param = atof(num.c_str());
			jnt_traj_params.push_back(param);
		}
		_traj_order_params.push_back(jnt_traj_params);
		Vector6d q = GenerateMotion(0, traj_idx);
		_q0.push_back(q);
		traj_idx += 1;
	}
	string freq;
	if (_order==5)
		freq = "0.5";
	else if (_order==10)
		freq = "1";
	string file_name = "traj_cpp_"+freq+"hz.csv";
	string file_path = "C:/00Work/01projects/RobotDynIden/slave/optimal_trajectory/";
	string file_save = file_path+file_name;
	_ofile.open(file_save, ios::out|ios::trunc);
}

Vector6d RobotDynIdenTest::GenerateMotion(double t, int traj_idx)
{
	Vector6d q_cmd;
	for (int jnt_idx = 0; jnt_idx<6; jnt_idx++)
	{
		RobotTools::JAVP javp = GenerateJntMotion(t, jnt_idx, traj_idx);
		q_cmd(jnt_idx) = javp.pos;
	}
	
	return q_cmd;
}

RobotTools::JAVP RobotDynIdenTest::GenerateJntMotion(double t, int jnt_idx, int traj_idx)
{
	for (int idx = 0; idx<_order; idx++)
	{
		double tmp = _w0*(idx+1);
		double sa = sin(tmp*t);
		double ca = cos(tmp*t);
		_q_b(idx) = sa/tmp;
		_q_b(_order+idx) = -ca/tmp;
		_qd_b(idx) = ca;
		_qd_b(_order+idx) = sa;
		_qdd_b(idx) = -sa*tmp;
		_qdd_b(_order+idx) = ca*tmp;
	}
	int nparams = 2*_order+1;
	Matrix<double, 1, Dynamic> traj_params;
	traj_params.setZero(nparams);
	RobotTools::JAVP javp;
	for (int idx = 0; idx<nparams; idx++)
	{
		traj_params(idx) = _traj_order_params[traj_idx][jnt_idx*nparams+idx];
	}
	javp.pos = traj_params*_q_b;
	javp.vel = traj_params*_qd_b;
	javp.acc = traj_params*_qdd_b;

	return javp;
}

Vector6d RobotDynIdenTest::GenerateTraj()
{
	Vector6d q_traj;
	if(!_ofile.is_open())
	{
		cout<<"failed to open the file"<<endl;
		q_traj = _q_cur;
	}
	else
	{
		_jtraj_planner.GeneratePath(q_traj);
		if (_jtraj_planner._plan_completed)
		{
			q_traj = GenerateMotion(_t, _traj_id[_traj_idx]);
			_t += g_cycle_time;
			MathTools::LimitMax(10, _t);
			if (fabs(_t-10)<EPS)
			{
				if (_traj_idx==_traj_id.size()-1)
				{
					_traj_completed = true;
					_ofile.close();
				}
				else
				{
					_t = 0;
					_traj_idx += 1;
					MatrixXd via_jpos;
					via_jpos.setZero(6, 1);
					via_jpos = _q0[_traj_id[_traj_idx]];
					_jtraj_planner.Reset(q_traj, true);
					_jtraj_planner.AddViaPos(&via_jpos);
					//_jtraj_planner = JointPlanner<6>(q_traj, _q0[_traj_id[_traj_idx]], vmax, amax, true);
				}
			}
			//_ofile<<q_traj<<endl;
		}
	}
	_ofile<<q_traj<<endl;

	return q_traj;
}
