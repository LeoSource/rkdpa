#pragma once

#include "GlobalParams.h"
#include "CleanRobot.h"
#include "UrRobot.h"

using namespace Eigen;

class RobotControl
{
public:	

private:	
    int _jointnum;
    CleanRobot _rbt;
    UrRobot _ur_rbt;
public:
    RobotControl(){_jointnum = 6;}

    int CreatRobot();

    void UpdateTool(Vector3d tool_pos, Vector3d tool_rpy);

    void UpdateJointHoldPos(VectorXd q);

    MatrixXd CalTrajViaPos_AbsolatePos(MatrixXd* pos_info, VectorXd q_fdb);

    MatrixXd CalTrajViaPos(MatrixXd* pos_info, VectorXd q_fdb);

    ERROR_ID TranCavpToJointPos(__IN CAVP& cavp, __IN Vector6d &q_fdb, __OUT Vector6d &q_cmd);

    ERROR_ID FKSolveTool(VectorXd q_fdb, __OUT Vector3d &pos, __OUT Vector3d &rpy);  //joint to xyzabc

    ~RobotControl() {}

private:
    void Creat5AxisRobot();
    void Creat6AxisRobot();
};

