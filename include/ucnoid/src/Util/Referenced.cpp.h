/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_IMPL_REFERENCED_CPP_H
#define UCNOID_UTIL_IMPL_REFERENCED_CPP_H

#include "Referenced.h"

namespace cnoid {
inline namespace ucnoid {

Referenced::~Referenced()
{
    if(weakCounter_){
        weakCounter_->setDestructed();
    }
}

}   // inline namespace ucnoid
}

#endif  // UCNOID_UTIL_IMPL_REFERENCED_CPP_H
