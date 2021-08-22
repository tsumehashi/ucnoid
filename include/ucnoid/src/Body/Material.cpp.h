/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_MATERIAL_CPP_H
#define UCNOID_BODY_MATERIAL_CPP_H

#include "Material.h"
#include <mutex>
#include <unordered_map>
#include <vector>

namespace cnoid {
inline namespace ucnoid {

namespace detail::material  {

inline std::mutex idMutex;
inline std::unordered_map<std::string, int> nameToIdMap;
inline std::vector<std::string> idToNameMap;

// register "default" material as id = 0
struct DefaultIdIntialization {
    DefaultIdIntialization(){
        nameToIdMap["default"] = 0;
        nameToIdMap["Default"] = 0;
        idToNameMap.push_back("default");
    }
};

}   // namespace detail::material


inline int Material::id(const std::string name)
{
    using namespace detail::material;
    std::lock_guard<std::mutex> guard(idMutex);

    static DefaultIdIntialization defaultIdInitialization;

    if(name.empty()){
        return 0;
    }

    int id;
    auto iter = nameToIdMap.find(name);
    if(iter != nameToIdMap.end()){
        id = iter->second;
    } else {
        id = idToNameMap.size();
        nameToIdMap.insert(make_pair(name, id));
        idToNameMap.push_back(name);
    }
    return id;
}


inline std::string Material::name(int id)
{
    using namespace detail::material;
    std::lock_guard<std::mutex> guard(idMutex);
    if(id < static_cast<int>(idToNameMap.size())){
        return idToNameMap[id];
    }
    return std::string();
}


inline Material::Material()
{
    roughness_ = 0.5;
    viscosity_ = 0.0;
    info_ = new Mapping;
}


inline Material::Material(const Material& org)
    : name_(org.name_)
{
    roughness_ = org.roughness_;
    viscosity_ = org.viscosity_;
    info_ = org.info_->cloneMapping();
}


inline Material::Material(const Mapping* info)
{
    roughness_ = 0.5;
    viscosity_ = 0.0;

    info_ = info->cloneMapping();
    info_->extract("name", name_);
    info_->extract("roughness", roughness_);
    info_->extract("viscosity", viscosity_);
}


inline Material::~Material()
{

}


template<> inline double Material::info(const std::string& key, const double& defaultValue) const
{
    double value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}


template<> inline bool Material::info(const std::string& key, const bool& defaultValue) const
{
    bool value;
    if(info_->read(key, value)){
        return value;
    }
    return defaultValue;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_MATERIAL_CPP_H
