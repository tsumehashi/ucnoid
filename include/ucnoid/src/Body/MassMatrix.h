/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_MASS_MATRIX_H
#define UCNOID_BODY_MASS_MATRIX_H

#include "Body.h"
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

UCNOID_EXPORT void calcMassMatrix(Body* body, const Vector3& g, MatrixXd& out_M);
UCNOID_EXPORT void calcMassMatrix(Body* body, MatrixXd& out_M);

}   // inline namespace ucnoid
}

#endif
