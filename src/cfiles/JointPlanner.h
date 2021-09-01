/**
* @file		JointPlanner.h
* @brief	Joint space trajectory plan
* @version	2.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/7/29
**/
#pragma once
#include "BaseTrajPlanner.h"
#include "LspbPlanner.h"
#include "GlobalParams.h"
#include <vector>

class JointPlanner : public BaseTrajPlanner
{
private:
	int _ntraj;
	int _traj_idx;
	bool _sync;
	vector<Vector6d> _jpos;
	double *_vmax, *_amax;
	vector<vector<LspbPlanner*>> _segplanner;
	vector<double> _tf;
	vector<vector<int>> _jplanned, _junplanned;

public:
	JointPlanner() {}

	JointPlanner(Vector6d q0, bool sync);

	void AddViaPos(MatrixXd* jpos) override;

	void GenerateMotion(Vector6d& jpos, Vector6d& jvel, Vector6d& jacc) override;

	void GeneratePath(Vector6d& jpos) override;

	void Reset(Vector6d q0, bool sync) override;

	void SetKineConstraint(double* vmax, double* amax);

	~JointPlanner() {}
private:
	void ClearTmp();
};

