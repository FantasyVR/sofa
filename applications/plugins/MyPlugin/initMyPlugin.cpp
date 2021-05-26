#include "initMyPlugin.h"

namespace sofa::component
{
    extern "C" {
        SOFA_MyPlugin_API void initExternalModule();
        SOFA_MyPlugin_API const char* getModuleName();
        SOFA_MyPlugin_API const char* getModuleVersion();
        SOFA_MyPlugin_API const char* getModuleLicense();
        SOFA_MyPlugin_API const char* getModuleDescription();
        SOFA_MyPlugin_API const char* getModuleComponentList();
    }
    void initExternalModule()
    {
        // Here is the place to write initialisation code, that will be executed
        // before any component is created.
    }

    const char* getModuleName()
    {
        return "MyPlugin";
    }

    const char* getModuleVersion()
    {
        return "0.1";
    }

    const char* getModuleLicense()
    {
        return "LGPL";
    }

    const char* getModuleDescription()
    {
        return "MyPlugin provides nothing for now.";
    }

    const char* getModuleComponentList()
    {
        // Comma-separated list of the components in this plugin, empty for now
        return "";
    }
}