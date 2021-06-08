#pragma once

#include "BaseTask.h"
#include "LspbTrajPlanner.h"
#include "GlobalParams.h"
#include "LineTrajPlanner.h"

using namespace std;
using namespace Eigen;

class RotXPlaneTask : public BaseTask
{
public:
	char* _clean_type;
protected:
	LspbTrajPlanner _pre_j1planner, _pre_j5planner, _pre_uplanner, _pre_pitchplanner;
	LspbTrajPlanner _uplanner, _pitchplanner, _post_uplanner;
	double _tf_clean, _tf1_pre, _tf2_pre, _tf_post;
	double _t;
	int _task_state;
	Vector5d _q_preclean, _q_postclean, _q_stow;
	Vector3d _pos_initial, _pos_preclean, _pos_postclean, _pos_stow;
	Vector3d _dir_initial, _dir_clean, _dir_stow;
	double _alph_initial;
	bool _pitch_return;
	Vector5d _jpos_initial;
private:
	LineTrajPlanner _line_traj[3];

public:
	RotXPlaneTask() {}

	RotXPlaneTask(CleanRobot* rbt):BaseTask(rbt) {}

	RotXPlaneTask(CleanRobot* rbt, char* clean_style);

	int RunLogicOperation(int state, int pre_state, char* operation) override;

	void SetTrajPos(MatrixXd* traj_pos, int num_section, VectorXd q_fdb) override;

	VectorXd RunTask(VectorXd q_fdb) override;

    MatrixXd CalTrajViaPos(MatrixXd* pos_info, VectorXd q_fdb);

	~RotXPlaneTask() {}

private:
	VectorXd RunPreCleanAction(VectorXd q_fdb);

	VectorXd RunScrapeAction(VectorXd q_fdb);

	VectorXd RunBrushCleanAction(VectorXd q_fdb);

	VectorXd RunPostCleanAction(VectorXd q_fdb);

	void CompletePreCleanAction(VectorXd q_fdb);

	void CompleteCleanAction(VectorXd q_fdb);

	void CompletePostCleanAction(VectorXd q_fdb);
};

