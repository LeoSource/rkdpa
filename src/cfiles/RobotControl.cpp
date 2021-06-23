#include "RobotControl.h"
#include "RobotParameter/RobotParameter.h"

int RobotControl::CreatRobot()
{
    //开机先读配置文件，然后creatRobot，故初始化不放在全局预编译阶段了
    _jointnum = RobotParameter::readRobotParameterFileJson((char*)"../ConfigureFiles/RobotParameter.json");
    RobotParameter::readHomingDataFileJson((char*)"../ConfigureFiles/HomingData.json");
    RobotParameter::readRobotToolFileJson((char*)"../ConfigureFiles/RobotTool.json");

//_jointnum = 5;

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
    pose_tool.pos << 0, 0, 0;
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

MatrixXd RobotControl::CalTrajViaPos_AbsolatePos(MatrixXd* pos_info, VectorXd q_fdb)
{
   MatrixXd via_pos;
   via_pos.setZero(3, (pos_info->cols()+1)*3);

   Vector3d pos0, rpy0;
   FKSolveTool(q_fdb, pos0, rpy0);

   via_pos.col(0) = pos0;
   via_pos.col(1) = rpy0; //startpos.RPY
   via_pos.col(2) = Vector3d(1,1,1);

   via_pos.col(3) = pos_info->col(0);
   via_pos.col(4) = Vector3d(0, 0 ,0);
   via_pos.col(5) = Vector3d(0, 0 ,1);  //type of path planning: both

   for(int i = 1; i < pos_info->cols(); ++i)
   {
       via_pos.col((i+1)*3) = pos_info->col(i);
       via_pos.col((i+1)*3 + 1) = Vector3d(0, 0 ,0);
       via_pos.col((i+1)*3 + 2) = Vector3d(1, 0 ,0); //type of path planning: pos
   }

   return via_pos;
}

MatrixXd RobotControl::CalTrajViaPos(MatrixXd* pos_info, VectorXd q_fdb)
{
    MatrixXd via_pos;
    via_pos.setZero(3, (1 + 4)*3); //startpos + posnum

    double pitch_x = 90*pi/180;
    double inc_angle[] = { 20*pi/180, 50*pi/180 };
    double pitch_high = pitch_x-inc_angle[0];
    double pitch_low = pitch_x-inc_angle[1];
    _rbt.SetPitchRange(pitch_high, pitch_low);

    Vector3d pos0 = _rbt.FKSolveTool(q_fdb).pos;
    double pitch0 = q_fdb(2)+_rbt._tool_pitch;
    double yaw0 = q_fdb(0)+q_fdb(4);
    Vector3d rpy0(0, pitch0, yaw0);

    via_pos.col(0) = pos0;
    via_pos.col(1) = rpy0; //startpos.rpy
    via_pos.col(2) = Vector3d(1,1,1); //

    via_pos.col(3) = pos_info->col(0);
    via_pos.col(4) = Vector3d(0, _rbt._pitch_high, 0); //rpy
    via_pos.col(5) = Vector3d(0, 0, 1); //

    via_pos.col(6) = pos_info->col(1);
    via_pos.col(7) = Vector3d(0, _rbt._pitch_high, 0);  //rpy
    via_pos.col(8) = Vector3d(1, 0, 0); //

    via_pos.col(9) = pos_info->col(2);
    via_pos.col(10) = Vector3d(0, _rbt._pitch_low, 0);  //rpy
    via_pos.col(11) = Vector3d(0, 0, 1); //

    via_pos.col(12) = pos_info->col(3);
    via_pos.col(13) = Vector3d(0, _rbt._pitch_low, 0);  //rpy
    via_pos.col(14) = Vector3d(1, 0, 0); //

    for(int i = 0; i < pos_info->cols(); ++i)
    {
        double line1_len = MathTools::Norm(via_pos.col((i+1)*3) - via_pos.col(i*3));
        if(line1_len < 0.2)
        {
            via_pos.col(2) = Vector3d(0, 0 ,0); //path smooth flag
            break;
        }
    }

    return via_pos;
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
