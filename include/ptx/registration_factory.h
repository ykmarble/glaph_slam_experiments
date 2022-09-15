#ifndef REGISTRATION_FACTORY_H_
#define REGISTRATION_FACTORY_H_

#include <ros/ros.h>
#include <pcl/registration/registration.h>
#include <ptx/types.h>
#include <boost/noncopyable.hpp>

namespace ptx
{

class RegistrationFactory : boost::noncopyable
{
    using CreateImplT = std::function<RegistrationT::Ptr(ros::NodeHandle&)>;
public:
    static bool registerImpl(const std::string& name, CreateImplT impl);
    static RegistrationT::Ptr create(const std::string& name, ros::NodeHandle& np);
private:
    std::map<std::string, CreateImplT> impl_table_;  // Must be singleton member, otherwise the initialization order is undefined.

    RegistrationFactory() = default;
    static RegistrationFactory& getInstance();
};

}

#endif  //REGISTRATION_FACTORY_H_
