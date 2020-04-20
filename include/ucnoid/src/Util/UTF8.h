/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_UTF8_H_INCLUDED
#define UCNOID_UTIL_UTF8_H_INCLUDED

#include <string>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {
#ifdef _WIN32
UCNOID_EXPORT const std::string toUTF8(const std::string& text);
UCNOID_EXPORT const std::string fromUTF8(const std::string& text);
UCNOID_EXPORT const std::string toUTF8(const char* text);
UCNOID_EXPORT const std::string fromUTF8(const char* text);
#else
inline const std::string& toUTF8(const std::string& text) { return text; }
inline const std::string& fromUTF8(const std::string& text) { return text; }
inline const std::string toUTF8(const char* text) { return text; }
inline const std::string fromUTF8(const char* text) { return text; }
//inline std::string toUTF8(const std::string& text) { return text; }
//inline std::string fromUTF8(const std::string& text) { return text; }
#endif
}   // inline namespace ucnoid
}

#include "UTF8.cpp.h"

#endif
