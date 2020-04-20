/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED
#define UCNOID_BODY_INVERSE_DYNAMICS_H_INCLUDED

#include <ucnoid/EigenTypes>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Link;

/**
   @return force being applied to the root link
*/
UCNOID_EXPORT Vector6 calcInverseDynamics(Link* link);

}   // inline namespace ucnoid
}

#include "InverseDynamics.cpp.h"

#endif
