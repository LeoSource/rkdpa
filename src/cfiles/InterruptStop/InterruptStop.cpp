#include "InterruptStop.h"

InterruptStop::InterruptStop(double cycle_time)
{
    _cycle_time = cycle_time;
    _t = 0.0;
    _initstep = eInterruptInit;
}

void InterruptStop::InitInterruptData(int step)
{
    _initstep = step;
}

void InterruptStop::InitInterruptData(const double amax[6], double vel[6], VectorXd joint_cur)
{
    if(_cycle_time < EPS)
        return;

    if(_initstep == 0)
    {
        Vector6d vel_cur, amax_cur;
        vel_cur << vel[0], vel[1], vel[2], vel[3], vel[4], vel[5];
        amax_cur <<amax[0] * 0.2, amax[1] * 0.2, amax[2] * 0.2, amax[3] * 0.2, amax[4] * 0.2, amax[5] * 0.2;
        _t = 0.0;
        _j_interrupt_planner.InitPlanner(joint_cur, vel_cur, amax_cur);
        _initstep = eInterruptPlanning;
    }
}

VectorXd InterruptStop::RefreshInterruptStopPlan(VectorXd q_cmd)
{
    if(_initstep == eInterruptPlanFinished)
        return q_cmd;

    VectorXd tmp_q = q_cmd;

     _t += _cycle_time;
    _j_interrupt_planner.GenerateMotion(_t, tmp_q);

    VectorXd new_q = tmp_q;
    if(_j_interrupt_planner.AllJointVelocityZeroFlag())
    {
        _t = 0.0;
        _initstep = eInterruptPlanFinished;
        return new_q;
    }
	
    return new_q;
}

int InterruptStop::GetInterruptStopPlanState()
{
    return _initstep;
}

