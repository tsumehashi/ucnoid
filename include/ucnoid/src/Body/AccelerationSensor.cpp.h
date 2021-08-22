/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_ACCELERATION_SENSOR_CPP_H
#define UCNOID_BODY_ACCELERATION_SENSOR_CPP_H

#include "AccelerationSensor.h"

namespace cnoid {
inline namespace ucnoid {

inline AccelerationSensor::AccelerationSensor()
    : spec(new Spec)
{
    spec->dv_max.setConstant(std::numeric_limits<double>::max());
    AccelerationSensor::clearState();
}


inline const char* AccelerationSensor::typeName()
{
    return "AccelerationSensor";
}


inline void AccelerationSensor::copyStateFrom(const AccelerationSensor& other)
{
    dv_ = other.dv_;
}


inline void AccelerationSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(AccelerationSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const AccelerationSensor&>(other));
}


inline AccelerationSensor::AccelerationSensor(const AccelerationSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec.reset(new Spec);
        if(org.spec){
            spec->dv_max = org.spec->dv_max;
        } else {
            spec->dv_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


inline DeviceState* AccelerationSensor::cloneState() const
{
    return new AccelerationSensor(*this, true);
}


inline Device* AccelerationSensor::clone() const
{
    return new AccelerationSensor(*this);
}


inline void AccelerationSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(AccelerationSensor))){
        Device::forEachActualType(func);
    }
}


inline void AccelerationSensor::clearState()
{
    dv_.setZero();
}


inline int AccelerationSensor::stateSize() const
{
    return 3;
}


inline const double* AccelerationSensor::readState(const double* buf)
{
    dv_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


inline double* AccelerationSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << dv_;
    return out_buf + 3;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_ACCELERATION_SENSOR_CPP_H
