/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_RATE_GYRO_SENSOR_CPP_H
#define UCNOID_BODY_RATE_GYRO_SENSOR_CPP_H

#include "RateGyroSensor.h"

namespace cnoid {
inline namespace ucnoid {

inline RateGyroSensor::RateGyroSensor()
    : spec(new Spec)
{
    spec->w_max.setConstant(std::numeric_limits<double>::max());
    RateGyroSensor::clearState();
}


inline const char* RateGyroSensor::typeName()
{
    return "RateGyroSensor";
}


inline void RateGyroSensor::copyStateFrom(const RateGyroSensor& other)
{
    w_ = other.w_;
}


inline void RateGyroSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RateGyroSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RateGyroSensor&>(other));
}


inline RateGyroSensor::RateGyroSensor(const RateGyroSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);

    if(!copyStateOnly){
        spec.reset(new Spec);
        if(org.spec){
            spec->w_max = org.spec->w_max;
        } else {
            spec->w_max.setConstant(std::numeric_limits<double>::max());
        }
    }
}


inline DeviceState* RateGyroSensor::cloneState() const
{
    return new RateGyroSensor(*this, true);
}


inline Device* RateGyroSensor::clone() const
{
    return new RateGyroSensor(*this);
}


inline void RateGyroSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RateGyroSensor))){
        Device::forEachActualType(func);
    }
}


inline void RateGyroSensor::clearState()
{
    w_.setZero();
}


inline int RateGyroSensor::stateSize() const
{
    return 3;
}


inline const double* RateGyroSensor::readState(const double* buf)
{
    w_ = Eigen::Map<const Vector3>(buf);
    return buf + 3;
}


inline double* RateGyroSensor::writeState(double* out_buf) const
{
    Eigen::Map<Vector3>(out_buf) << w_;
    return out_buf + 3;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_RATE_GYRO_SENSOR_CPP_H
