/**
   \file
   \author Shin'ichiro Nakaoka
   \note This file can only be included from gettext.h defining CNOID_GETTEXT_DOMAIN_NAME
*/

#ifndef UCNOID_UTIL_GETTEXT_UTIL_H
#define UCNOID_UTIL_GETTEXT_UTIL_H

#if UCNOID_NOT_SUPPORTED

#include <ucnoid/Config>
#include "exportdecl.h"

#include <boost/format.hpp>

#define UCNOID_ENABLE_GETTEXT 1

#if UCNOID_ENABLE_GETTEXT
# include <libintl.h>

# ifdef UCNOID_USE_GETTEXT_WRAPPER
#  define _(text) cnoid::getText(UCNOID_GETTEXT_DOMAIN_NAME, text)
# else
#  define _(text) dgettext(UCNOID_GETTEXT_DOMAIN_NAME, text)
# endif

#else
namespace cnoid {
inline const char* bindtextdomain(const char* domainname, const char* dirname) {
    return dirname;
}
inline const char* dgettext(const char* domainname, const char* msgid){
    return msgid;
}
}
#define _(string) string
#endif

#define N_(string) string

namespace cnoid {
inline namespace ucnoid {

//! \deprecated
inline boost::format fmt(const char* f_string) {
    boost::format f(f_string);
    f.exceptions(boost::io::no_error_bits);
    return f;
}

//! \deprecated
inline boost::format fmt(const std::string& f_string) {
    boost::format f(f_string);
    f.exceptions(boost::io::no_error_bits);
    return f;
}

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
UCNOID_EXPORT const char* getText(const char* domainname, const char* msgid);
#else
inline const char* getText(const char* domainname, const char* msgid) {
    return dgettext(domainname, msgid);
}
#endif

UCNOID_EXPORT void bindGettextDomain(const char* domainname);

class GettextDomainBinder
{
public:
    GettextDomainBinder(const char* domainname){
        bindGettextDomain(domainname);
    }
};

}   // inline namespace ucnoid
}

/**
   Implement this once in a shared library to bind a gettext domain.
   The "gettext.h" header must be included before using this macro.
*/
#define UCNOID_BIND_GETTEXT_DOMAN() \
    namespace { cnoid::GettextDomainBinder cnoidGettextDomainBinder(UCNOID_GETTEXT_DOMAIN_NAME); }

#else   // UCNOID_NOT_SUPPORTED

namespace cnoid {
inline namespace ucnoid {

#include <tuple>
#include <string>
#include <sstream>

inline const std::string& _(const std::string& str)
{
    return str;
}

template <class... Args>
inline std::string ssformat(const Args&... args)
{
    std::stringstream ss;
    auto t = std::make_tuple(args...);
    std::apply([&ss](auto&&... args) {((ss << args), ...);}, t);
    return ss.str();
}

}
}

#endif  // UCNOID_NOT_SUPPORTED

#include "GettextUtil.cpp.h"

#endif
