/**
* @file		TaskTrajPlanner.h
* @brief	Cartesian trajectory plan for every task
* @version	1.0.0
* @author	zxliao
* @email	zhixiangleo@163.com
* @date		2021/6/7
**/
#pragma once

#include "LineTrajPlanner.h"
#include "ArcTrajPlanner.h"
#include "GlobalParams.h"
#include <vector>

class TaskTrajPlanner
{
public:
	bool _task_completed;
private:
	vector<BaseCTrajPlanner*> _segtraj_planner;
	int _ntraj;
	vector<Vector3d*> _pos_corner;
	vector<Vector3d*> _rpy_corner;
	vector<Vector3d*> _pos_seg;
	vector<Vector3d*> _rpy_seg;
	vector<double> _varc;
	double _t;
	int _traj_idx;
	bool _continuity;

public:
	TaskTrajPlanner() 
	{ 
		_task_completed = true;
		_continuity = false;
	}

	/**
	* @brief	constructor
	* @author	zxliao
	* @date		2021/6/7
	* @param	pos0	initial position
	* @param	rpy0	initial rotation: rpy
	* @param	conti_type	continuous arc transition between 2 lines
	**/
	TaskTrajPlanner(Vector3d* pos0, Vector3d* rpy0, bool conti_type);

	void AddStartPosRPY(Vector3d *pos0, Vector3d *rpy0);

	/**
	* @brief	add task corner positino and rpy
	* @author	zxliao
	* @date		2021/6/7
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	* @param	opt		option: both, pos or rot
	**/
	void AddPosRPY(Vector3d* pos, Vector3d* rpy, char* opt);

	/**
	* @brief	add task corner positino and rpy under continuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	* @param	opt		option: both, pos or rot
	**/
	void AddContiPosRPY(Vector3d* pos, Vector3d* rpy, char* opt);

	/**
	* @brief	add task corner positino and rpy under discontinuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @param	pos		corner position
	* @param	rpy		corner rotation: rpy
	* @param	opt		option: both, pos or rot
	**/
	void AddDiscontiPosRPY(Vector3d* pos, Vector3d* rpy, char* opt);

	/**
	* @brief	generate trajectory data: position, rpy, spatial velocity and acceleration
	* @author	zxliao
	* @date		2021/6/7
	* @return	cavp	trajectory data: position, rpy, spatial velocity and acceleration
	**/
	RobotTools::CAVP GenerateMotion();

	/**
	* @brief	generate trajectory data under continuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @return	cavp	trajectory data: position, rpy, spatial velocity and acceleration
	**/
	RobotTools::CAVP GenerateContiMotion();

	/**
	* @brief	generate trajectory data under discontinuous condition
	* @author	zxliao
	* @date		2021/6/8
	* @return	cavp	trajectory data: position, rpy, spatial velocity and acceleration
	**/
	RobotTools::CAVP GenerateDiscontiMotion();

	/**
	* @brief	reset task trajectory planner
	* @author	zxliao
	* @date		2021/6/7
	* @param	conti_type	continuous arc transition between 2 lines
	**/
	void Reset(bool conti_type);

	/**
	* @brief	reset task trajectory planner with position and rpy
	* @author	zxliao
	* @date		2021/6/7
	* @param	pos0	initial position
	* @param	rpy0	initial rotation: rpy
	* @param	conti_type	continuous arc transition between 2 lines
	**/
	void Reset(Vector3d* pos0, Vector3d* rpy0, bool conti_type);

	void AddPosRPY(Vector3d *pos, Vector3d *rpy, Vector3d &seg_opt);

	/**
	* @brief	generate trajectory position and rpy
	* @author	zxliao
	* @date		2021/6/7
	* @return	pos_rpy		trajectory position and rpy
	**/
	Vector6d GeneratePoint();

	void ReInitSegTrajPlanner(Vector3d &pos0, Vector3d &rpy0);

	bool GetSegTrajPlannerDone();

	~TaskTrajPlanner() {}

private:
	/**
	* @brief	get the 3rd position to define arc
	* @author	zxliao
	* @date		2021/6/7
	* @param	p1			1st position to define arc
	* @param	p2			2nd position to define arc
	* @param	p3_corner	3rd position on the p2p3 extended line
	* @return	pos3		the 3rd position to define arc
	**/
	Vector3d UpdateSegPos(Vector3d p1, Vector3d p2, Vector3d p3_corner);

	/**
	* @brief	clear temp positions and planners
	* @author	zxliao
	* @date		2021/6/7
	**/
	void ClearTemp();
};

