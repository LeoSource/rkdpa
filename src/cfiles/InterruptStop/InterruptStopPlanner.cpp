#include "InterruptStopPlanner.h"


InterruptStopPlanner::InterruptStopPlanner(VectorXd pos, VectorXd vel_init, VectorXd max_acc)
{
    InitPlanner(pos, vel_init, max_acc);
}

void InterruptStopPlanner::InitPlanner(VectorXd max_acc)
{
    _amax = max_acc;
}

void InterruptStopPlanner::InitPlanner(VectorXd pos, VectorXd vel_init, VectorXd max_acc)
{
     _amax = max_acc; //<< max_acc[0]*0.2, max_acc[1]*0.2, max_acc[2]*0.2, max_acc[3]*0.2, max_acc[4]*0.2, max_acc[5]*0.2;
    _v0 = vel_init;
    _q0 = pos;
    double t_max = 0.0;
	
    for(int i = 0; i < 6; i++)
    {
        if(_amax[i] < EPS)
        {
           _isdone[i] = 1;
           _tf[i] = 0;
           continue;
        }

        _tf[i] = fabs(_v0[i]/_amax[i]);
        _dir[i] = (vel_init[i] > 0 ? 1 : -1);
        _isdone[i] = 0;

        t_max = t_max > _tf[i] ? t_max:_tf[i];
    }

    for(int i = 0; i < 6; i++)
    {
        _tf[i] = t_max;
        _amax[i] = fabs(vel_init[i]/t_max);
    }
}

//基于当前已规划的总时间计算总progress，加上初始位置，得到新的位置
void InterruptStopPlanner::GenerateMotion(double t, VectorXd &q_cmd)
{
    for(int i = 0; i < 6; i++)
    {
        if(_isdone[i] || t > _tf[i])
            _isdone[i] = 1;
        else
            q_cmd[i] = _q0[i] + (_v0[i] * t - 0.5 * _dir[i] * _amax[i] * t * t);
    }
}

bool InterruptStopPlanner::AllJointVelocityZeroFlag()
{
    return _isdone[0] && _isdone[1] && _isdone[2] && _isdone[3] && _isdone[4] && _isdone[5];
}
