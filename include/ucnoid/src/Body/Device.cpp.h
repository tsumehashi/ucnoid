/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_DEVICE_CPP_H
#define UCNOID_BODY_DEVICE_CPP_H

#include "Device.h"
#include "Link.h"

namespace cnoid {
inline namespace ucnoid {

inline Device::Device()
{
    ns = new NonState;
    ns->index = -1;
    ns->id = -1;
    ns->link = nullptr;
    T_local().setIdentity();
    setCycle(20.0);
}


inline Device::Device(const Device& org, bool copyStateOnly)
{
    if(copyStateOnly){
        ns = nullptr;
    } else {
        ns = new NonState;
        ns->index = -1;
        ns->id = org.ns->id;
        ns->name = org.ns->name;
        ns->link = nullptr;
        T_local() = org.T_local();
        setCycle(org.cycle());
    }
}


inline Device::~Device()
{
    if(ns){
        delete ns;
    }
}


inline void Device::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    func(typeid(Device));
}


inline void Device::clearState()
{

}


inline bool Device::on() const
{
    return true;
}


inline void Device::on(bool)
{

}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_DEVICE_CPP_H
