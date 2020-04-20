/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_POINT_LIGHT_H
#define CNOID_BODY_POINT_LIGHT_H

#include "Light.h"
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT PointLight : public Light
{
public:
    PointLight();
    PointLight(const PointLight& org, bool copyStateOnly = false);

    virtual const char* typeName();
    void copyStateFrom(const PointLight& other);
    virtual void copyStateFrom(const DeviceState& other);
    virtual DeviceState* cloneState() const;
    virtual Device* clone() const;
    virtual void forEachActualType(std::function<bool(const std::type_info& type)> func);

    static int pointLightStateSize();
    virtual int stateSize() const;
    virtual const double* readState(const double* buf);
    virtual double* writeState(double* out_buf) const;

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

typedef ref_ptr<PointLight> PointLightPtr;

}

#endif
