#pragma once

#include "RobotMath.h"


double l1 = 0, l2 = 0, l3 = 0;
double g_dh[] = { 0,0,0,0,
				pi/2,0,0,0,
				0,l2,-l1,pi/2,
				pi/2,0,0,pi/2,
				0,0,-l3,pi/2 };
int g_type[] = { 0,1,0,1,0 };
double g_offset[] = { 0, 0, pi/2, 0, 0 };
double g_qlimit[] = { -pi/2, pi/2, 0.5, 1.5, -pi/2, pi/2, 0.2, 1, -2*pi, 2*pi };


