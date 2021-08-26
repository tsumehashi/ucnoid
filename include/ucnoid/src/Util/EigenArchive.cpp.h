#ifndef UCNOID_UTIL_EIGEN_ARCHIVE_CPP_H
#define UCNOID_UTIL_EIGEN_ARCHIVE_CPP_H

#include "EigenArchive.h"

namespace cnoid {
inline namespace ucnoid {

namespace detail::eigen_archive {

template<typename Scalar>
bool read_(const Mapping& mapping, const std::string& key, Eigen::AngleAxis<Scalar>& r)
{
    const Listing& s = *mapping.findListing(key);
    if(s.isValid() && s.size() == 4){
        r.axis() << s[0].to<Scalar>(), s[1].to<Scalar>(), s[2].to<Scalar>();
        r.angle() = s[3].to<Scalar>();
        return true;
    }
    return false;
}

}   // namespace detail::eigen_archive

inline bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisd& r)
{
    return detail::eigen_archive::read_(mapping, key, r);
}


inline bool read(const Mapping& mapping, const std::string& key, Eigen::AngleAxisf& r)
{
    return detail::eigen_archive::read_(mapping, key, r);
}

namespace detail::eigen_archive {

template<typename Scalar>
Listing& write_(Mapping& mapping, const std::string& key, const Eigen::AngleAxis<Scalar>& r)
{
    Listing& s = *mapping.createFlowStyleListing(key);
    s.setDoubleFormat("%.9g");
    s.append(r.axis()[0]);
    s.append(r.axis()[1]);
    s.append(r.axis()[2]);
    s.append(r.angle());
    return s;
}

}   // namespace detail::eigen_archive

inline Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisd& r)
{
    return detail::eigen_archive::write_(mapping, key, r);
}


inline Listing& write(Mapping& mapping, const std::string& key, const Eigen::AngleAxisf& r)
{
    return detail::eigen_archive::write_(mapping, key, r);
}

namespace detail::eigen_archive {

template<typename ValueType>
bool read_(const Mapping& mapping, const std::string& key, std::function<void(const ValueType& value)> setterFunc)
{
    ValueType value;
    if(read(mapping, key, value)){
        setterFunc(value);
        return true;
    }
    return false;
}

}   // namespace detail::eigen_archive

inline bool read(const Mapping& mapping, const std::string& key, std::function<void(const Eigen::Vector3d& value)> setterFunc)
{
    return detail::eigen_archive::read_(mapping, key, setterFunc);
}


}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_EIGEN_ARCHIVE_CPP_H
