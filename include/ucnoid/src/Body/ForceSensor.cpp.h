/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_FORCE_SENSOR_CPP_H
#define UCNOID_BODY_FORCE_SENSOR_CPP_H

#include "ForceSensor.h"

namespace cnoid {
inline namespace ucnoid {

inline ForceSensor::ForceSensor()
    : spec(new Spec)
{
    spec->F_max.setConstant(std::numeric_limits<double>::max());
    ForceSensor::clearState();
}


inline const char* ForceSensor::typeName()
{
    return "ForceSensor";
}


inline void ForceSensor::copyStateFrom(const ForceSensor& other)
{
    F_ = other.F_;
}


inline void ForceSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(ForceSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const ForceSensor&>(other));
}


inline ForceSensor::ForceSensor(const ForceSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec.reset(new Spec);
        if(org.spec){
            spec->F_max = org.spec->F_max;
        } else {
            spec->F_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


inline DeviceState* ForceSensor::cloneState() const
{
    return new ForceSensor(*this, true);
}


inline Device* ForceSensor::clone() const
{
    return new ForceSensor(*this);
}


inline void ForceSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(ForceSensor))){
        Device::forEachActualType(func);
    }
}


inline void ForceSensor::clearState()
{
    F_.setZero();
}


inline int ForceSensor::stateSize() const
{
    return 6;
}


inline const double* ForceSensor::readState(const double* buf)
{
    F_ = Eigen::Map<const Vector6>(buf);
    return buf + 6;
}


inline double* ForceSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector6>(out_buf) << F_;
    return out_buf + 6;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_FORCE_SENSOR_CPP_H
