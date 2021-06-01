#pragma once

#include "RobotMath.h"


const double l1 = 0.111, l2 = 0.0935, l3 = 0.066, l4 = 0.068;
const double h = 0.403, w = 0.518;
const double g_yaw0_x0 = l2-l3;
const double g_dh[] = { 0,0,0,0,
						pi/2,0,0,0,
						0,l2,-l1,pi/2,
						pi/2,0,0,pi/2,
						0,-l4,-l3,pi/2 };
const int g_type[] = { 0,1,0,1,0 };
const double g_offset[] = { 0, h, pi/2, w, 0 };
const double g_qlimit[] = { -pi/2, pi/2, 0, 0.8, -pi/2, pi/2, 0, 0.55, -2*pi, 2*pi };
const double g_cycle_time = 0.005;
const double g_stowed_jpos[] = { 0,0.6,0,0,0 };
const double g_stowed_cpos[] = { 0,0.7,0.9 };
const double g_return_err[] = { 0.01, 0.007, 0.01, 0.007, 0.01 };
const double g_jvmax[] = {pi*0.5, 0.15, 0.8*pi*0.5, 0.5, 0.8*pi*0.5};
const double g_jamax[] = { 2*pi*0.5, 0.3, 1.6*pi*0.5, 1, 1.6*pi*0.5};
const double g_cvmax = 0.15;
const double g_camax = 0.3;
