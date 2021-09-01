#pragma once

#include "../RobotMath.h"
#include "InterruptStopPlanner.h"

using namespace std;
using namespace Eigen;

enum InterruptPlanState
{
	eInterruptInit = 0,
	eInterruptPlanning = 1,
	eInterruptPlanFinished = 2,
};

class InterruptStop
{
protected:
	InterruptStopPlanner _j_interrupt_planner;
	double _t;
	double _cycle_time;
	int _initstep;  //0 初始化; 1 减速规划过程中; 2 减速规划结束
public:
	InterruptStop() {}
	InterruptStop(double cycle_time);

	void InitInterruptData(int step);
	//给定当前速度和当前关节位置
	void InitInterruptData(const double amax[6], double vel[6], VectorXd joint_cur);

	VectorXd RefreshInterruptStopPlan(VectorXd q_cmd);

	int GetInterruptStopPlanState();

	~InterruptStop() {}

private:


};
