/**
   @file
   @author Shin'ichiro Nakaoka
*/
#ifndef UCNOID_UTIL_GETTEXT_UTIL_CPP_H
#define UCNOID_UTIL_GETTEXT_UTIL_CPP_H

#if UCNOID_NOT_SUPPORTED

#include "GettextUtil.h"
#include <ucnoid/ExecutablePath>
#include <ucnoid/FileUtil>
#include "exportdecl.h"

namespace filesystem = std::filesystem;

namespace cnoid {
inline namespace ucnoid {

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(__NT__)
# if UCNOID_ENABLE_GETTEXT
const char* getText(const char* domainname, const char* msgid)
{
    return dgettext(domainname, msgid);
}
# else
const char* getText(const char* domainname, const char* msgid)
{
    return msgid;
}
# endif
#endif

void bindGettextDomain(const char* domainname)
{
#if UCNOID_ENABLE_GETTEXT
    filesystem::path localePath = filesystem::path(executableTopDirectory()) / "share" / "locale";
    if(filesystem::is_directory(localePath)){
        bindtextdomain(domainname, getPathString(localePath).c_str());
    } else {
        localePath = filesystem::path(shareDirectory()) / "locale";
        if(filesystem::is_directory(localePath)){
            bindtextdomain(domainname, getPathString(localePath).c_str());
        }
    }
#endif
}

}   // inline namespace ucnoid
}

#endif  // UCNOID_NOT_SUPPORTED

#endif  // UCNOID_UTIL_GETTEXT_UTIL_CPP_H
