/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H
#define UCNOID_UTIL_POLYGON_MESH_TRIANGULATOR_H

#include <string>
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class SgMesh;
class SgPolygonMesh;
class PolygonMeshTriangulatorImpl;

class UCNOID_EXPORT PolygonMeshTriangulator
{
public:
    PolygonMeshTriangulator();
    PolygonMeshTriangulator(const PolygonMeshTriangulator& org);
    ~PolygonMeshTriangulator();

    void setDeepCopyEnabled(bool on);

    SgMesh* triangulate(SgPolygonMesh* polygonMesh);

    const std::string& errorMessage() const;

private:
    PolygonMeshTriangulatorImpl * impl;
};

}   // inline namespace ucnoid
}

#include "PolygonMeshTriangulator.cpp.h"

#endif

