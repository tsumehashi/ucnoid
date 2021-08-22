/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_POINT_LIGHT_CPP_H
#define UCNOID_BODY_POINT_LIGHT_CPP_H

#include "PointLight.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::point_light {

static inline const int LightStateSize = Light::lightStateSize();

}

inline PointLight::PointLight()
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


inline const char* PointLight::typeName()
{
    return "PointLight";
}


inline void PointLight::copyStateFrom(const PointLight& other)
{
    Light::copyStateFrom(other);

    constantAttenuation_ = other.constantAttenuation_;
    linearAttenuation_ = other.linearAttenuation_;
    quadraticAttenuation_ = other.quadraticAttenuation_;
}


inline void PointLight::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(PointLight)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const PointLight&>(other));
}


inline PointLight::PointLight(const PointLight& org, bool copyStateOnly)
    : Light(org, copyStateOnly)
{
    copyStateFrom(org);
}


inline DeviceState* PointLight::cloneState() const
{
    return new PointLight(*this, true);
}


inline Device* PointLight::clone() const
{
    return new PointLight(*this);
}


inline void PointLight::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(PointLight))){
        Light::forEachActualType(func);
    }
}


inline int PointLight::pointLightStateSize()
{
    return detail::point_light::LightStateSize + 3;
}


inline int PointLight::stateSize() const
{
    return detail::point_light::LightStateSize + 3;
}


inline const double* PointLight::readState(const double* buf)
{
    buf = Light::readState(buf);
    constantAttenuation_ = buf[0];
    linearAttenuation_ = buf[1];
    quadraticAttenuation_ = buf[2];
    return buf + 3;
}


inline double* PointLight::writeState(double* out_buf) const
{
    out_buf = Light::writeState(out_buf);
    out_buf[0] = constantAttenuation_;
    out_buf[1] = linearAttenuation_;
    out_buf[2] = quadraticAttenuation_;
    return out_buf + 3;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_POINT_LIGHT_CPP_H
