/**
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H
#define UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H

#include "AbstractSceneLoader.h"

namespace cnoid {
inline namespace ucnoid {

inline AbstractSceneLoader::~AbstractSceneLoader()
{

}


inline void AbstractSceneLoader::setMessageSink(std::ostream& /* os */)
{

}


inline void AbstractSceneLoader::setDefaultDivisionNumber(int /* n */)
{

}


inline void AbstractSceneLoader::setDefaultCreaseAngle(double /* theta */)
{

}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_ABSTRACT_SCENE_LOADER_CPP_H
