/** 
    \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_CONSTRAINT_FORCE_SOLVER_H
#define UCNOID_BODY_CONSTRAINT_FORCE_SOLVER_H

#if UCNOID_NOT_SUPPORTED
#include <cnoid/CollisionSeq>
#endif  // UCNOID_NOT_SUPPORTED
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Link;
class ConstraintForceSolverImpl;
class WorldBase;
#if UCNOID_NOT_SUPPORTED
class CollisionDetector;
class ContactMaterial;
class MaterialTable;
#endif  // UCNOID_NOT_SUPPORTED
	
class UCNOID_EXPORT ConstraintForceSolver
{
    ConstraintForceSolverImpl* impl;
		
public:
    ConstraintForceSolver(WorldBase& world);
    ~ConstraintForceSolver();
		
#if UCNOID_NOT_SUPPORTED
    void setCollisionDetector(CollisionDetector* detector);
    CollisionDetector* collisionDetector();

    void setMaterialTable(MaterialTable* table);

    void setFriction(double staticFriction, double slipFliction);
    double staticFriction() const;
    double slipFriction() const;

    void setSelfCollisionDetectionEnabled(int bodyIndex, bool on);
    bool isSelfCollisionDetectionEnabled(int bodyIndex) const;

    void setContactCullingDistance(double thresh);
    double contactCullingDistance() const;
    
    void setContactCullingDepth(double depth);
    double contactCullingDepth();
#endif  // UCNOID_NOT_SUPPORTED
    
    void setCoefficientOfRestitution(double epsilon);
    double coefficientOfRestitution() const;

    void setGaussSeidelErrorCriterion(double e);
    double gaussSeidelErrorCriterion();

    void setGaussSeidelMaxNumIterations(int n);
    int gaussSeidelMaxNumIterations();

#if UCNOID_NOT_SUPPORTED
    void setContactDepthCorrection(double depth, double velocityRatio);
    double contactCorrectionDepth();
    double contactCorrectionVelocityRatio();
#endif  // UCNOID_NOT_SUPPORTED

    void set2Dmode(bool on);
    void enableConstraintForceOutput(bool on);

    void initialize(void);
    void solve();
    void clearExternalForces();

#if UCNOID_NOT_SUPPORTED
    CollisionLinkPairListPtr getCollisions();
#endif  // UCNOID_NOT_SUPPORTED

#ifdef ENABLE_SIMULATION_PROFILING
    double getCollisionTime();
#endif

#if UCNOID_NOT_SUPPORTED
    // experimental functions
    typedef std::function<bool(Link* link1, Link* link2,
                               const CollisionArray& collisions,
                               ContactMaterial* contactMaterial)>  CollisionHandler;
    
    void registerCollisionHandler(const std::string& name, CollisionHandler handler);
    bool unregisterCollisionHandler(const std::string& name);
#endif  // UCNOID_NOT_SUPPORTED
};

}
};

#include "ConstraintForceSolver.cpp.h"

#endif
