#include "RobotIK/Package/Plugin/PluginABI.h"

uint32_t robotik_plugin_abi_version(void)
{
    return 999u;
}

bool robotik_plugin_query(RobotIKPluginInfo* info, RobotIKPluginVTable* api)
{
    (void)info;
    (void)api;
    return false;
}

RobotIKPlugin* robotik_plugin_create(RobotIKHostAPI const* host)
{
    (void)host;
    return 0;
}

void robotik_plugin_destroy(RobotIKPlugin* plugin)
{
    (void)plugin;
}
