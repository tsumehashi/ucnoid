/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_SPOT_LIGHT_CPP_H
#define UCNOID_BODY_SPOT_LIGHT_CPP_H

#include "SpotLight.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::spot_light {

static inline const int PointLightStateSize = PointLight::pointLightStateSize();

}


inline SpotLight::SpotLight()
{
    direction_ << 0.0f, 0.0f, -1.0f;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


inline const char* SpotLight::typeName()
{
    return "SpotLight";
}


inline void SpotLight::copyStateFrom(const SpotLight& other)
{
    PointLight::copyStateFrom(other);
    direction_ = other.direction_;
    beamWidth_ = other.beamWidth_;
    cutOffAngle_ = other.cutOffAngle_;
    cutOffExponent_ = other.cutOffExponent_;
}


inline void SpotLight::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(SpotLight)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const SpotLight&>(other));
}


inline SpotLight::SpotLight(const SpotLight& org, bool copyStateOnly)
    : PointLight(org, copyStateOnly)
{
    copyStateFrom(org);
}


inline DeviceState* SpotLight::cloneState() const
{
    return new SpotLight(*this, true);
}


inline Device* SpotLight::clone() const
{
    return new SpotLight(*this);
}


inline void SpotLight::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(SpotLight))){
        PointLight::forEachActualType(func);
    }
}


inline int SpotLight::stateSize() const
{
    return detail::spot_light::PointLightStateSize + 6;
}


inline const double* SpotLight::readState(const double* buf)
{
    buf = PointLight::readState(buf);
    direction_ = Eigen::Map<const Vector3>(buf);
    beamWidth_ = buf[3];
    cutOffAngle_ = buf[4];
    cutOffExponent_ = buf[5];
    return buf + 6;
}


inline double* SpotLight::writeState(double* out_buf) const
{
    out_buf = PointLight::writeState(out_buf);
    Eigen::Map<Vector3>(out_buf) << direction_;
    out_buf[3] = beamWidth_;
    out_buf[4] = cutOffAngle_;
    out_buf[5] = cutOffExponent_;
    return out_buf + 6;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_SPOT_LIGHT_CPP_H
