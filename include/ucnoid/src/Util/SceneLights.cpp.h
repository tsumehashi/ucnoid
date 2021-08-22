/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_SCENE_LIGHTS_CPP_H
#define UCNOID_UTIL_SCENE_LIGHTS_CPP_H

#include "SceneLights.h"

namespace cnoid {
inline namespace ucnoid {

inline SgLight::SgLight(int polymorhicId)
    : SgPreprocessed(polymorhicId)
{
    on_ = true;
    color_.setOnes();
    intensity_ = 1.0f;
    ambientIntensity_ = 0.0f;
}


inline SgLight::SgLight(const SgLight& org)
    : SgPreprocessed(org)
{
    on_ = org.on_;
    color_ = org.color_;
    intensity_ = org.intensity_;
    ambientIntensity_ = org.ambientIntensity_;
}


inline SgObject* SgLight::clone(SgCloneMap&) const
{
    return new SgLight(*this);
}


inline SgDirectionalLight::SgDirectionalLight(int polymorhicId)
    : SgLight(polymorhicId)
{
    direction_ << 0.0, 0.0, -1.0;
}


inline SgDirectionalLight::SgDirectionalLight()
    : SgDirectionalLight(findPolymorphicId<SgDirectionalLight>())
{

}


inline SgDirectionalLight::SgDirectionalLight(const SgDirectionalLight& org)
    : SgLight(org)
{
    direction_ = org.direction_;
}


inline SgObject* SgDirectionalLight::clone(SgCloneMap&) const
{
    return new SgDirectionalLight(*this);
}


inline SgPointLight::SgPointLight(int polymorhicId)
    : SgLight(polymorhicId)
{
    constantAttenuation_ = 1.0f;
    linearAttenuation_ = 0.0f;
    quadraticAttenuation_ = 0.0f;
}


inline SgPointLight::SgPointLight()
    : SgPointLight(findPolymorphicId<SgPointLight>())
{

}


inline SgPointLight::SgPointLight(const SgPointLight& org)
    : SgLight(org)
{
    constantAttenuation_ = org.constantAttenuation_;
    linearAttenuation_ = org.linearAttenuation_;
    quadraticAttenuation_ = org.quadraticAttenuation_;
}


inline SgObject* SgPointLight::clone(SgCloneMap&) const
{
    return new SgPointLight(*this);
}


inline SgSpotLight::SgSpotLight(int polymorhicId)
    : SgPointLight(polymorhicId)
{
    direction_ << 0.0, 0.0, -1.0;
    beamWidth_ = 1.570796f;
    cutOffAngle_ = 0.785398f;
    cutOffExponent_ = 1.0f;
}


inline SgSpotLight::SgSpotLight()
    : SgSpotLight(findPolymorphicId<SgSpotLight>())
{

}


inline SgSpotLight::SgSpotLight(const SgSpotLight& org)
    : SgPointLight(org)
{
    direction_ = org.direction_;
    beamWidth_ = org.beamWidth_;
    cutOffAngle_ = org.cutOffAngle_;
    cutOffExponent_ = org.cutOffExponent_;
}


inline SgObject* SgSpotLight::clone(SgCloneMap&) const
{
    return new SgSpotLight(*this);
}

namespace detail::scene_light {

inline struct NodeTypeRegistration {
    NodeTypeRegistration() {
        SgNode::registerType<SgLight, SgPreprocessed>();
        SgNode::registerType<SgDirectionalLight, SgLight>();
        SgNode::registerType<SgPointLight, SgLight>();
        SgNode::registerType<SgSpotLight, SgPointLight>();
    }
} registration;

}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_SCENE_LIGHTS_CPP_H
