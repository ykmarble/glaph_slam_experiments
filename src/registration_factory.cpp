#include "ptx/registration_factory.h"

namespace ptx
{

RegistrationFactory& RegistrationFactory::getInstance()
{
    static RegistrationFactory inst;
    return inst;
}

bool RegistrationFactory::registerImpl(const std::string& name, CreateImplT impl)
{
    RegistrationFactory& inst = getInstance();
    const auto result = inst.impl_table_.emplace(name, impl);
    if(!result.second)
    {
        throw std::runtime_error("Failed to register RegistrationFactory implementation.");
    }
    return result.second;
}

RegistrationT::Ptr RegistrationFactory::create(const std::string& name, ros::NodeHandle& np)
{
    RegistrationFactory& inst = getInstance();
    const auto impl= inst.impl_table_.find(name);
    if (impl == inst.impl_table_.end())
    {
        throw std::runtime_error("Unknown Registration " + name);
    }
    ROS_INFO("Created %s ReigistrationFactory instance.", name.c_str());
    return impl->second(np);
}

}
