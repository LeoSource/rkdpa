#pragma once

extern double g_tool_mirror[];
extern double g_tool_table[];
extern double g_tool_water_press[];
extern double g_tool_toilet[];
extern double g_tool_toilet_outer_ring[];

extern double g_tool_default[];
extern double g_tool_current[];

namespace RobotParameter
{
    int readRobotParameterFileJson(char *filename);

    int readRobotToolFileJson(char *filename);

    int readGlobalPosDataFileJson(char *filename);

}
