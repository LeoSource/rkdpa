#pragma once
#include "../RobotMath.h"
using namespace std;
using namespace Eigen;

class InterruptStopPlanner
{
public:	
    Vector6d _amax;
private:
	
    Vector6d _tf;
    Vector6d _v0;
    Vector6d _q0;
    int _dir[6];
    bool _isdone[6];
public:
	InterruptStopPlanner() {}

    InterruptStopPlanner(VectorXd pos, VectorXd cur_vel, VectorXd max_acc);
	
    void InitPlanner(VectorXd max_acc);
    void InitPlanner(VectorXd pos, VectorXd cur_vel, VectorXd max_acc);

	~InterruptStopPlanner() {}

    void GenerateMotion(double t, VectorXd &q_cmd);

	bool AllJointVelocityZeroFlag();
	

private:

};

