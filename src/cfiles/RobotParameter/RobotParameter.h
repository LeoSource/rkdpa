#pragma once

extern double g_tool_mirror[];
extern double g_tool_table[];
extern double g_tool_default[];

namespace RobotParameter
{
    int readRobotParameterFileJson(char *filename);

    int readRobotToolFileJson(char *filename);

    int readHomingDataFileJson(char *filename);
}
