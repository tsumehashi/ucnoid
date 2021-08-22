/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_LIGHT_CPP_H
#define UCNOID_BODY_LIGHT_CPP_H

#include "Light.h"

namespace cnoid {
inline namespace ucnoid {

inline Light::Light()
{
    on_ = true;
    color_.setConstant(1.0f);
    intensity_ = 1.0f;
}


inline void Light::copyStateFrom(const Light& other)
{
    on_ = other.on_;
    color_ = other.color_;
    intensity_ = other.intensity_;
}


inline Light::Light(const Light& org, bool copyStateOnly)
    : Device(org, copyStateOnly)
{
    copyStateFrom(org);
}


inline void Light::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Light))){
        Device::forEachActualType(func);
    }
}


inline bool Light::on() const
{
    return on_;
}


inline void Light::on(bool on)
{
    on_ = on;
}


inline int Light::lightStateSize()
{
    return 5;
}


inline const double* Light::readState(const double* buf)
{
    on_ = buf[0];
    color_ = Eigen::Map<const Vector3>(buf + 1).cast<float>();
    intensity_ = buf[4];
    return buf + 5;
}


inline double* Light::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = color_[0];
    out_buf[2] = color_[1];
    out_buf[3] = color_[2];
    out_buf[4] = intensity_;
    return out_buf + 5;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_LIGHT_CPP_H
