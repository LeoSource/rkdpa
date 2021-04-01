#pragma once

#include "RobotMath.h"


const double l1 = 0, l2 = 0, l3 = 0;
const double h = 0.5, w = 0.4;
const double g_dh[] = { 0,0,0,0,
						pi/2,0,0,0,
						0,l2,-l1,pi/2,
						pi/2,0,0,pi/2,
						0,0,-l3,pi/2 };
const int g_type[] = { 0,1,0,1,0 };
const double g_offset[] = { 0, h, pi/2, w, 0 };
const double g_qlimit[] = { -pi/2, pi/2, 0, 1, -pi/2, pi/2, 0, 0.6, -2*pi, 2*pi };
const double g_cycle_time = 0.001;

