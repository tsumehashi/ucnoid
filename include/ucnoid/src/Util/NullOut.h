/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_NULL_OUT_H
#define UCNOID_UTIL_NULL_OUT_H

#include <iosfwd>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

UCNOID_EXPORT std::ostream& nullout();

}   // inline namespace ucnoid
}

#include "NullOut.cpp.h"

#endif
