#include "Robotik/Plugin/PluginABI.h"

uint32_t robotik_plugin_abi_version(void)
{
    return 999u;
}

bool robotik_plugin_query(RobotikPluginInfo* info, RobotikPluginVTable* api)
{
    (void)info;
    (void)api;
    return false;
}

RobotikPlugin* robotik_plugin_create(RobotikHostAPI const* host)
{
    (void)host;
    return 0;
}

void robotik_plugin_destroy(RobotikPlugin* plugin)
{
    (void)plugin;
}
