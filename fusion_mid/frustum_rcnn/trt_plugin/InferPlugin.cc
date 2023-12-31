#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "./common/checkMacrosPlugin.h"
#include "./common/plugin.h"
#include <algorithm>
#include <array>
#include <iostream>
#include <memory>
#include <mutex>
#include <stack>
#include <unordered_set>
using namespace nvinfer1;
using namespace nvinfer1::plugin;


#include "FrcnnSegProposalPlugin/FrcnnSegProposalPlugin.h"
#include "CenterShiftCustomPlugin/CenterShiftCustomPlugin.h"

using namespace amirstan;
using namespace amirstan::plugin;

namespace nvinfer1
{

namespace plugin
{

extern ILogger* gLogger;

// This singleton ensures that each plugin is only registered once for a given
// namespace and type, and attempts of duplicate registration are ignored.
class PluginCreatorRegistry
{
public:
    static PluginCreatorRegistry& getInstance()
    {
        static PluginCreatorRegistry instance;
        return instance;
    }

    template <typename CreatorType>
    void addPluginCreator(void* logger, const char* libNamespace)
    {
        // Make accesses to the plugin creator registry thread safe
        std::lock_guard<std::mutex> lock(mRegistryLock);

        std::string errorMsg;
        std::string verboseMsg;

        std::unique_ptr<CreatorType> pluginCreator{new CreatorType{}};
        pluginCreator->setPluginNamespace(libNamespace);

        nvinfer1::plugin::gLogger = static_cast<nvinfer1::ILogger*>(logger);
        std::string pluginType = std::string{pluginCreator->getPluginNamespace()}
            + "::" + std::string{pluginCreator->getPluginName()} + " version "
            + std::string{pluginCreator->getPluginVersion()};

        if (mRegistryList.find(pluginType) == mRegistryList.end())
        {
            bool status = getPluginRegistry()->registerCreator(*pluginCreator, libNamespace);
            if (status)
            {
                mRegistry.push(std::move(pluginCreator));
                mRegistryList.insert(pluginType);
                verboseMsg = "Registered plugin creator - " + pluginType;
            }
            else
            {
                errorMsg = "Could not register plugin creator -  " + pluginType;
            }
        }
        else
        {
            verboseMsg = "Plugin creator already registered - " + pluginType;
        }

        if (logger)
        {
            if (!errorMsg.empty())
            {
                nvinfer1::plugin::gLogger->log(ILogger::Severity::kERROR, errorMsg.c_str());
            }
            if (!verboseMsg.empty())
            {
                nvinfer1::plugin::gLogger->log(ILogger::Severity::kVERBOSE, verboseMsg.c_str());
            }
        }
    }

    ~PluginCreatorRegistry()
    {
        std::lock_guard<std::mutex> lock(mRegistryLock);

        // Release pluginCreators in LIFO order of registration.
        while (!mRegistry.empty())
        {
            mRegistry.pop();
        }
        mRegistryList.clear();
    }

private:
    PluginCreatorRegistry() {}

    std::mutex mRegistryLock;
    std::stack<std::unique_ptr<IPluginCreator>> mRegistry;
    std::unordered_set<std::string> mRegistryList;

public:
    PluginCreatorRegistry(PluginCreatorRegistry const&) = delete;
    void operator=(PluginCreatorRegistry const&) = delete;
};

template <typename CreatorType>
void initializePlugin(void* logger, const char* libNamespace)
{
    PluginCreatorRegistry::getInstance().addPluginCreator<CreatorType>(logger, libNamespace);
}

} // namespace plugin
} // namespace nvinfer1
// New Plugin APIs

extern "C"
{
    bool initLibNvInferPlugins(void* logger, const char* libNamespace)
    {
        initializePlugin<amirstan::plugin::FrcnnSegProposalPluginCreator>(logger, libNamespace);
        initializePlugin<amirstan::plugin::CenterShiftCustomPluginCreator>(logger, libNamespace);
        return true;
    }
} // extern "C"

/*
namespace amirstan {
namespace plugin {

REGISTER_TENSORRT_PLUGIN(FrcnnSegProposalPluginCreator);
REGISTER_TENSORRT_PLUGIN(CenterShiftCustomPluginCreator);

}  // namespace plugin
}  // namespace amirstan

extern "C" {

bool initLibAmirstanInferPlugins() { return true; }
}  // extern "C"
*/