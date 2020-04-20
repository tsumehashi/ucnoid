/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef CNOID_UTIL_SCENE_LIGHTS_H
#define CNOID_UTIL_SCENE_LIGHTS_H

#include "SceneGraph.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT SgLight : public SgPreprocessed
{
protected:
    SgLight(int polymorhicId);
    SgLight(const SgLight& org);
        
public:
    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    bool on() const { return on_; }
    void on(bool on) { on_ = on; }
    const Vector3f& color() const { return color_; }
    template<typename Derived> void setColor(const Eigen::MatrixBase<Derived>& c) {
        color_ = c.template cast<Vector3f::Scalar>(); }
    float intensity() const { return intensity_; }
    void setIntensity(float intensity) { intensity_ = intensity; }
    float ambientIntensity() const { return ambientIntensity_; }
    void setAmbientIntensity(float intensity) { ambientIntensity_ = intensity; }
        
private:
    Vector3f color_;
    float intensity_;
    float ambientIntensity_;
    bool on_;
};
typedef ref_ptr<SgLight> SgLightPtr;


class CNOID_EXPORT SgDirectionalLight : public SgLight
{
protected:
    SgDirectionalLight(int polymorhicId);
    
public:
    SgDirectionalLight();
    SgDirectionalLight(const SgDirectionalLight& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    const Vector3& direction() const { return direction_; }
    template<typename Derived> void setDirection(const Eigen::MatrixBase<Derived>& d) {
        direction_ = d.template cast<Vector3::Scalar>(); }

private:
    Vector3 direction_;
};
typedef ref_ptr<SgDirectionalLight> SgDirectionalLightPtr;


class CNOID_EXPORT SgPointLight : public SgLight
{
protected:
    SgPointLight(int polymorhicId);
    
public:
    SgPointLight();
    SgPointLight(const SgPointLight& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    float constantAttenuation() const { return constantAttenuation_; }
    void setConstantAttenuation(float a) { constantAttenuation_ = a; }

    float linearAttenuation() const { return linearAttenuation_; }
    void setLinearAttenuation(float a) { linearAttenuation_ = a; }

    float quadraticAttenuation() const { return quadraticAttenuation_; }
    void setQuadraticAttenuation(float a) { quadraticAttenuation_ = a; }

private:
    float constantAttenuation_;
    float linearAttenuation_;
    float quadraticAttenuation_;
};
typedef ref_ptr<SgPointLight> SgPointLightPtr;


class CNOID_EXPORT SgSpotLight : public SgPointLight
{
protected:
    SgSpotLight(int polymorhicId);
    
public:
    SgSpotLight();
    SgSpotLight(const SgSpotLight& org);
    virtual SgObject* clone(SgCloneMap& cloneMap) const;

    const Vector3& direction() const { return direction_; }
    template<typename Derived> void setDirection(const Eigen::MatrixBase<Derived>& d) {
        direction_ = d.template cast<Vector3::Scalar>(); }

    float beamWidth() const { return beamWidth_; }
    void setBeamWidth(float w) { beamWidth_ = w; }
    float cutOffAngle() const { return cutOffAngle_; }
    void setCutOffAngle(float a) { cutOffAngle_ = a; }
    float cutOffExponent() const { return cutOffExponent_; }
    void setCutOffExponent(float e) { cutOffExponent_ = e; }

private:
    Vector3 direction_;
    float beamWidth_;
    float cutOffAngle_;
    float cutOffExponent_;
};
typedef ref_ptr<SgSpotLight> SgSpotLightPtr;

}

#endif
