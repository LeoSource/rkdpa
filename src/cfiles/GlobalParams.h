#pragma once

#include "RobotMath.h"

extern double l1, l2, l3, l4;
extern double h, w;
extern double g_yaw0_x0;
extern double g_dh[];
extern int g_type[];
extern double g_offset[];
extern double g_qlimit[];
extern double g_cycle_time ;
extern double g_stowed_jpos[];
extern double g_stowed_cpos[] ;
extern double g_return_err[] ;
extern double g_jvmax[];
extern double g_jamax[];
extern double g_cvmax;
extern double g_camax;

//theta d a alpha
extern double a3, a4, d1, d4, d5, d6;
extern double g_dh_ur[];
extern int g_type_ur[];
extern double g_offset_ur[];
extern double g_qlimit_ur[];

#define __IN
#define __OUT
#define __IN_OUT

enum ERROR_ID
{
    eNoErr = 0,
    eErrDivisionByZero = 100,
    eArmSingularity = 101,
    eWristSingularity = 102,
    eCalculateOutsideReach = 103,

};
