#include "RobotControl.h"
#include "RobotParameter/RobotParameter.h"

int RobotControl::CreatRobot()
{
    //开机先读配置文件，然后creatRobot，故初始化不放在全局预编译阶段了
//    _jointnum = RobotParameter::readRobotParameterFileJson((char*)"../ConfigureFiles/RobotParameter.json");
//    RobotParameter::readHomingDataFileJson((char*)"../ConfigureFiles/HomingData.json");
//    RobotParameter::readRobotToolFileJson((char*)"../ConfigureFiles/RobotTool.json");

_jointnum = 6;

    switch(_jointnum)
    {
    case 5:
        Creat5AxisRobot();
        break;
    case 6:
        Creat6AxisRobot();
        break;
    }

	return _jointnum;
}

void RobotControl::Creat5AxisRobot()
{
    Matrix<double, 5, 4, RowMajor> mdh_table(&g_dh[0]);
    Matrix<int, 5, 1> jnt_type(&g_type[0]);
    Matrix<double, 5, 1> offset(&g_offset[0]);

    double tool_rx = g_tool_default[3];
    Vector3d tool_pos(g_tool_default[0], g_tool_default[1], g_tool_default[2]);

    Matrix<double, 5, 2, RowMajor> qlimit(&g_qlimit[0]);
    CleanRobot rbt(mdh_table, jnt_type, offset, tool_rx, tool_pos);
    rbt.SetJntLimit(qlimit);

    _rbt = rbt;
}

void RobotControl::Creat6AxisRobot()
{
    Matrix<double, 6, 4, RowMajor> mdh_table(&g_dh_ur[0]);
    Matrix<int, 6, 1> jnt_type(&g_type_ur[0]);
    Matrix<double, 6, 1> offset(&g_offset_ur[0]);
    RobotTools::Pose pose_tool;
    pose_tool.rot.setIdentity();
    pose_tool.rot = RotX(g_tool_default[3])*RotY(g_tool_default[4])*RotZ(g_tool_default[5]);
    pose_tool.pos << g_tool_default[0], g_tool_default[1], g_tool_default[2];

    Matrix<double, 6, 2, RowMajor> qlimit(&g_qlimit_ur[0]);
    UrRobot rbt(mdh_table, jnt_type, offset, pose_tool);
    rbt.SetJntLimit(qlimit);

    _ur_rbt = rbt;
}

void RobotControl::UpdateTool(Vector3d tool_pos, Vector3d tool_rpy)
{
    switch(_jointnum)
    {
    case 5:
        _rbt.UpdateTool(tool_rpy(0), tool_pos);
        break;
    case 6:
        _ur_rbt.UpdateTool(tool_pos, tool_rpy);
        break;
    }
}

void RobotControl::UpdateVelocity(unsigned int joint_percent, unsigned int cart_percent)
{
    for(int i = 0; i <_jointnum; i++)
    {
        g_jvmax[i] = g_jvmax_default[i] * joint_percent * 0.01;
        g_jamax[i] = g_jamax_default[i] * joint_percent * 0.01;
    }

    for(int i = 0; i < 2;i++)
    {
        g_cvmax[i] = g_cvmax_default[i]*cart_percent*0.01;
        g_camax[i] = g_camax_default[i]*cart_percent*0.01;
    }
}

void RobotControl::UpdateJointHoldPos(VectorXd q_cmd)
{
    switch(_jointnum)
    {
    case 5:
        _rbt.UpdateJntHoldPos(q_cmd);
        break;
    case 6:
        _ur_rbt.UpdateJntHoldPos(q_cmd);
        break;
    }
}

ERROR_ID RobotControl::TranCavpToJointPos(__IN CAVP& cavp, __IN Vector6d &q_fdb, __OUT Vector6d &q_cmd)
{
	int err = 0;

    switch(_jointnum)
    {
    case 5:
        {
            VectorXd tmp_q_cmd = _rbt.IKSolvePitchYaw(cavp.pos.head(3), cavp.pos(4), cavp.pos(5), q_fdb);
            _rbt.UpdateJntHoldPos(tmp_q_cmd);
            q_cmd << tmp_q_cmd(0),tmp_q_cmd(1),tmp_q_cmd(2),tmp_q_cmd(3),tmp_q_cmd(4), 0;
        }
        break;

    case 6:
        Pose pose;
        pose.pos = cavp.pos.head(3);
        pose.rot = FixedZYX2Tr(cavp.pos.tail(3));
        err = _ur_rbt.IKSolve(&q_cmd, &pose, &q_fdb);
        VectorXd tmp_q_cmd = q_cmd;
        err = MathTools::LimitVector(_ur_rbt._qlimit.col(0), &tmp_q_cmd, _ur_rbt._qlimit.col(1));
        if(err != eNoErr)
            err = eErrJointOutOfRange;

        q_cmd = tmp_q_cmd;
        _ur_rbt.UpdateJntHoldPos(q_cmd);
        break;
    }

	return (ERROR_ID)err;
}

ERROR_ID RobotControl::FKSolveTool(VectorXd q_fdb, __OUT Vector3d &pos, __OUT Vector3d &rpy)  //joint to xyzabc
{
    switch(_jointnum)
    {
    case 5:
        {
            pos = _rbt.FKSolveTool(q_fdb).pos;
            double pitch0 = q_fdb(2)+_rbt._tool_pitch;
            double yaw0 = q_fdb(0)+q_fdb(4);
            rpy<<0, pitch0, yaw0;
        }
        break;
    case 6:
        Pose pose0 = _ur_rbt.FKSolveTool(q_fdb);
        rpy = RobotTools::Tr2FixedZYX(pose0.rot);
        pos = pose0.pos;
        break;
    }

    return eNoErr;
}

ERROR_ID RobotControl::TranCamerPosToRobotBase(__IN VectorXd q_fdb, __IN Vector3d &camer_pos, __IN Vector3d &camer_rpy, __OUT Vector3d &pos, __OUT Vector3d &rpy)
{
    Pose pose_cur = _ur_rbt.FKSolve(q_fdb);

    Pose camerpos;
    camerpos.pos = camer_pos;
    camerpos.rot = RobotTools::FixedZYX2Tr(camer_rpy);

    Pose pose0 = RobotTools::PoseProduct(pose_cur, camerpos);
    rpy = RobotTools::Tr2FixedZYX(pose0.rot);
    pos = pose0.pos;

    return eNoErr;
}
