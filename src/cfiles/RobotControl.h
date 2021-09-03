#pragma once

#include "GlobalParams.h"
#include "CleanRobot.h"
#include "UrRobot.h"

using namespace Eigen;

class RobotControl
{
public:	
    UrRobot _ur_rbt;
private:	
    int _jointnum;
    CleanRobot _rbt;
public:
    RobotControl(){_jointnum = 6;}

    int CreatRobot();

    void UpdateTool(Vector3d tool_pos, Vector3d tool_rpy);

    void UpdateVelocity(unsigned int joint_percent, unsigned int cart_percent);

    void UpdateJointHoldPos(VectorXd q);

    ERROR_ID TranCavpToJointPos(__IN CAVP& cavp, __IN Vector6d &q_fdb, __OUT Vector6d &q_cmd);

    ERROR_ID FKSolveTool(VectorXd q_fdb, __OUT Vector3d &pos, __OUT Vector3d &rpy);  //joint to xyzabc

    ERROR_ID TranCamerPosToRobotBase(__IN VectorXd q_fdb, __IN Vector3d &camer_pos, __IN Vector3d &camer_rpy, __OUT Vector3d &pos, __OUT Vector3d &rpy);

    ~RobotControl() {}

private:
    void Creat5AxisRobot();
    void Creat6AxisRobot();
};

