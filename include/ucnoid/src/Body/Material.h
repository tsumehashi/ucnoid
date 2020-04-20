/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_MATERIAL_H
#define UCNOID_BODY_MATERIAL_H

#include <ucnoid/Referenced>
#include <ucnoid/ValueTree>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class UCNOID_EXPORT Material : public Referenced
{
public:
    Material();
    Material(const Mapping* info);
    Material(const Material& org);
    ~Material();

    static int id(const std::string name);
    static std::string name(int id);
    
    const std::string& name() const { return name_; }
    void setName(const std::string& name) { name_ = name; }
    double roughness() const { return roughness_; }
    void setRoughness(double r) { roughness_ = r; }
    double viscosity() const { return viscosity_; }
    void setViscosity(double v) { viscosity_ = v; }

    Mapping* info() { return info_; }
    const Mapping* info() const { return info_; }
    template<typename T> T info(const std::string& key, const T& defaultValue) const;
    
    void resetInfo(Mapping* info) { info_ = info; }
    
private:
    std::string name_;
    double roughness_;
    double viscosity_;
    MappingPtr info_;
};

template<> UCNOID_EXPORT double Material::info(const std::string& key, const double& defaultValue) const;
template<> UCNOID_EXPORT bool Material::info(const std::string& key, const bool& defaultValue) const;

typedef ref_ptr<Material> MaterialPtr;

}   // inline namespace ucnoid
}

#include "Material.cpp.h"

#endif
