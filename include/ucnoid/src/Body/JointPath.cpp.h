/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_JOINT_PATH_CPP_H
#define UCNOID_BODY_JOINT_PATH_CPP_H
  
#include "JointPath.h"
#include "Jacobian.h"
#include "Body.h"
#if UCNOID_NOT_SUPPORTED
#include "BodyCustomizerInterface.h"
#endif  // UCNOID_NOT_SUPPORTED
#include <ucnoid/EigenUtil>
#include <ucnoid/TruncatedSVD>

namespace cnoid {
inline namespace ucnoid {

inline double JointPath::numericalIKdefaultDeltaScale()
{
    return 0.9;
}

inline double JointPath::numericalIKdefaultTruncateRatio()
{
    return TruncatedSVD<MatrixXd>::defaultTruncateRatio();
}

inline int JointPath::numericalIKdefaultMaxIterations()
{
    return 50;
}

inline double JointPath::numericalIKdefaultMaxIKerror()
{
    return 1.0e-6;
}

inline double JointPath::numericalIKdefaultDampingConstant()
{
    return 1.0e-6;
}


class JointPathIkImpl
{
public:
    bool isBestEffortIKmode;
    double deltaScale;
    int maxIterations;
    int iteration; 
    double maxIKerrorSqr;
    double dampingConstantSqr;
    MatrixXd J;
    VectorXd dTask;
    VectorXd dq;
    std::vector<double> q0;
    MatrixXd JJ;
    Eigen::ColPivHouseholderQR<MatrixXd> QR;
    TruncatedSVD<MatrixXd> svd;
    std::function<double(VectorXd& out_error)> errorFunc;
    std::function<void(MatrixXd& out_Jacobian)> jacobianFunc;

    JointPathIkImpl() {
        deltaScale = JointPath::numericalIKdefaultDeltaScale();
        maxIterations = JointPath::numericalIKdefaultMaxIterations();
        iteration = 0;
        dTask.resize(6);
        isBestEffortIKmode = false;
        double e = JointPath::numericalIKdefaultMaxIKerror();
        maxIKerrorSqr = e * e;
        double d = JointPath::numericalIKdefaultDampingConstant();
        dampingConstantSqr = d * d;
    }

    void resize(int numJoints){
        J.resize(dTask.size(), numJoints);
        dq.resize(numJoints);
    }
};


inline JointPath::JointPath()
{
    initialize();
}


inline JointPath::JointPath(Link* base, Link* end)
    : linkPath(base, end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


inline JointPath::JointPath(Link* end)
    : linkPath(end), 
      joints(linkPath.size())
{
    initialize();
    extractJoints();
}


inline void JointPath::initialize()
{
    nuIK = 0;
    needForwardKinematicsBeforeIK = false;
}    


inline JointPath::~JointPath()
{
    if(nuIK){
        delete nuIK;
    }
}


inline bool JointPath::setPath(Link* base, Link* end)
{
    if(linkPath.setPath(base, end)){
        extractJoints();
    }
    onJointPathUpdated();

    return (!joints.empty());
}


inline bool JointPath::setPath(Link* end)
{
    linkPath.setPath(end);
    extractJoints();
    onJointPathUpdated();
	
    return !joints.empty();
}


inline void JointPath::extractJoints()
{
    numUpwardJointConnections = 0;

    int n = linkPath.size();
    if(n <= 1){
        joints.clear();
    } else {
        int i = 0;
        if(linkPath.isDownward(i)){
            i++;
        }
        joints.resize(n); // reserve size n buffer
        joints.clear();
        int m = n - 1;
        while(i < m){
            Link* link = linkPath[i];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints.push_back(link);
                    if(!linkPath.isDownward(i)){
                        numUpwardJointConnections++;
                    }
                }
            }
            ++i;
        }
        if(linkPath.isDownward(m-1)){
            Link* link = linkPath[m];
            if(link->jointId() >= 0){
                if(link->isRotationalJoint() || link->isSlideJoint()){
                    joints.push_back(link);
                }
            }
        }
    }
}


inline int JointPath::indexOf(const Link* link) const
{
    for(size_t i=0; i < joints.size(); ++i){
        if(joints[i] == link){
            return i;
        }
    }
    return -1;
}


inline void JointPath::onJointPathUpdated()
{
    if(nuIK){
        nuIK->errorFunc = nullptr;
        nuIK->jacobianFunc = nullptr;
    }
}


inline void JointPath::calcJacobian(Eigen::MatrixXd& out_J) const
{
    const int n = joints.size();
    out_J.resize(6, n);
	
    if(n > 0){

        //! \todo compare the efficiency for the following codes
        if(false){
            setJacobian<0x3f, 0, 0>(*this, linkPath.endLink(), out_J);

        } else {
        
            Link* targetLink = linkPath.endLink();
		
            for(int i=0; i < n; ++i){
			
                Link* link = joints[i];
			
                switch(link->jointType()){
				
                case Link::REVOLUTE_JOINT:
                {
                    Vector3 omega = link->R() * link->a();
                    const Vector3 arm = targetLink->p() - link->p();
                    if(!isJointDownward(i)){
                        omega = -omega;
                    }
                    out_J.col(i) << omega.cross(arm), omega;
                }
                break;
				
                case Link::PRISMATIC_JOINT:
                {
                    Vector3 dp = link->R() * link->d();
                    if(!isJointDownward(i)){
                        dp = -dp;
                    }
                    out_J.col(i) << dp, Vector3::Zero();
                }
                break;
				
                default:
                    out_J.col(i).setZero();
                }
            }
        }
    }
}


inline JointPathIkImpl* JointPath::getOrCreateNumericalIK()
{
    if(!nuIK){
        nuIK = new JointPathIkImpl();
    }
    return nuIK;
}


inline bool JointPath::isBestEffortIKmode() const
{
    return nuIK ? nuIK->isBestEffortIKmode : false;
}


inline void JointPath::setBestEffortIKmode(bool on)
{
    getOrCreateNumericalIK()->isBestEffortIKmode = on;
}


inline void JointPath::setNumericalIKmaxIKerror(double e)
{
    getOrCreateNumericalIK()->maxIKerrorSqr = e * e;
}


inline void JointPath::setNumericalIKdeltaScale(double s)
{
    getOrCreateNumericalIK()->deltaScale = s;
}


inline void JointPath::setNumericalIKtruncateRatio(double r)
{
    getOrCreateNumericalIK()->svd.setTruncateRatio(r);
}

    
inline void JointPath::setNumericalIKmaxIterations(int n)
{
    getOrCreateNumericalIK()->maxIterations = n;
}


inline void JointPath::setNumericalIKdampingConstant(double lambda)
{
    getOrCreateNumericalIK()->dampingConstantSqr = lambda * lambda;
}


inline void JointPath::customizeTarget
(int numTargetElements,
 std::function<double(VectorXd& out_error)> errorFunc,
 std::function<void(MatrixXd& out_Jacobian)> jacobianFunc)
{
    getOrCreateNumericalIK();
    nuIK->dTask.resize(numTargetElements);
    nuIK->errorFunc = errorFunc;
    nuIK->jacobianFunc = jacobianFunc;
}


inline JointPath& JointPath::setBaseLinkGoal(const Position& T)
{
    linkPath.baseLink()->setPosition(T);
    needForwardKinematicsBeforeIK = true;
    return *this;
}


inline bool JointPath::calcInverseKinematics()
{
    if(nuIK->errorFunc){
        Position T; // dummy
        return calcInverseKinematics(T);
    }
    return false;
}


inline bool JointPath::calcInverseKinematics(const Position& T)
{
    const bool USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM = false;
    const bool USE_SVD_FOR_BEST_EFFORT_IK = false;

    if(joints.empty()){
        if(linkPath.empty()){
            return false;
        }
        if(baseLink() == endLink()){
            baseLink()->setPosition(T);
            return true;
        } else {
            // \todo implement here
            return false;
        }
    }
    const int n = numJoints();
    
    if(!nuIK){
        nuIK = new JointPathIkImpl();
    }
    if(!nuIK->jacobianFunc){
        nuIK->jacobianFunc = [&](MatrixXd& out_Jacobian){ setJacobian<0x3f, 0, 0>(*this, endLink(), out_Jacobian); };
    }
    nuIK->resize(n);
    
    Link* target = linkPath.endLink();

    if(needForwardKinematicsBeforeIK){
        calcForwardKinematics();
        needForwardKinematicsBeforeIK = false;
    }

    nuIK->q0.resize(n);
    if(!nuIK->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            nuIK->q0[i] = joints[i]->q();
        }
    }

    double prevErrsqr = std::numeric_limits<double>::max();
    bool completed = false;

    bool useUsualInverseSolution = false;
    if(USE_USUAL_INVERSE_SOLUTION_FOR_6x6_NON_BEST_EFFORT_PROBLEM){
        if(!nuIK->isBestEffortIKmode && (n == 6) && (nuIK->dTask.size() == 6)){
            useUsualInverseSolution = true;
        }
    }
        
    if(USE_SVD_FOR_BEST_EFFORT_IK && !useUsualInverseSolution && !nuIK->isBestEffortIKmode){
        // disable truncation
        nuIK->svd.setTruncateRatio(std::numeric_limits<double>::max());
    }

    for(nuIK->iteration = 0; nuIK->iteration < nuIK->maxIterations; ++nuIK->iteration){

        double errorSqr;
        if(nuIK->errorFunc){
            errorSqr = nuIK->errorFunc(nuIK->dTask);
        } else {
            nuIK->dTask.head<3>() = T.translation() - target->p();
            nuIK->dTask.segment<3>(3) = target->R() * omegaFromRot(target->R().transpose() * T.linear());
            errorSqr = nuIK->dTask.squaredNorm();
        }
        if(errorSqr < nuIK->maxIKerrorSqr){
            completed = true;
            break;
        }
        if(prevErrsqr - errorSqr < nuIK->maxIKerrorSqr){
            if(nuIK->isBestEffortIKmode && (errorSqr > prevErrsqr)){
                for(int j=0; j < n; ++j){
                    joints[j]->q() = nuIK->q0[j];
                }
                calcForwardKinematics();
            }
            break;
        }
        prevErrsqr = errorSqr;

        nuIK->jacobianFunc(nuIK->J);

        if(useUsualInverseSolution){
            nuIK->dq = nuIK->QR.compute(nuIK->J).solve(nuIK->dTask);
        } else {
            if(USE_SVD_FOR_BEST_EFFORT_IK){
                nuIK->svd.compute(nuIK->J).solve(nuIK->dTask, nuIK->dq);
            } else {
                // The damped least squares (singurality robust inverse) method
                nuIK->JJ = nuIK->J * nuIK->J.transpose() + nuIK->dampingConstantSqr * MatrixXd::Identity(nuIK->J.rows(), nuIK->J.rows());
                nuIK->dq = nuIK->J.transpose() * nuIK->QR.compute(nuIK->JJ).solve(nuIK->dTask);
            }
        }

        if(nuIK->isBestEffortIKmode){
            for(int j=0; j < n; ++j){
                double& q = joints[j]->q();
                nuIK->q0[j] = q;
                q += nuIK->deltaScale * nuIK->dq(j);
            }
        } else {
            for(int j=0; j < n; ++j){
                joints[j]->q() += nuIK->deltaScale * nuIK->dq(j);
            }
        }

        calcForwardKinematics();
    }

    if(!completed && !nuIK->isBestEffortIKmode){
        for(int i=0; i < n; ++i){
            joints[i]->q() = nuIK->q0[i];
        }
        calcForwardKinematics();
    }
    
    return completed;
}


inline int JointPath::numIterations() const
{
    return nuIK ? nuIK->iteration : 0;
}


inline bool JointPath::hasAnalyticalIK() const
{
    return false;
}


/*
JointPath& JointPath::setGoal
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    targetTranslationGoal = end_p;
    targetRotationGoal = end_R;
    
    Link* baseLink = linkPath.baseLink();
    baseLink->p() = base_p;
    baseLink->R() = base_R;

    needForwardKinematicsBeforeIK = true;

    return *this;
}
*/


inline bool JointPath::calcInverseKinematics
(const Vector3& base_p, const Matrix3& base_R, const Vector3& end_p, const Matrix3& end_R)
{
    Position T_base;
    T_base.linear() = base_R;
    T_base.translation() = base_p;

    Position T_end;
    T_end.linear() = end_R;
    T_end.translation() = end_p;
    
    return setBaseLinkGoal(T_base).calcInverseKinematics(T_end);
}


inline std::ostream& operator<<(std::ostream& os, JointPath& path)
{
    int n = path.numJoints();
    for(int i=0; i < n; ++i){
        Link* link = path.joint(i);
        os << link->name();
        if(i != n){
            os << (path.isJointDownward(i) ? " => " : " <= ");
        }
    }
    os << std::endl;
    return os;
}

#if UCNOID_NOT_SUPPORTED
namespace {

class CustomJointPath : public JointPath
{
    BodyPtr body;
    int ikTypeId;
    bool isCustomizedIkPathReversed;
    virtual void onJointPathUpdated();
public:
    CustomJointPath(const BodyPtr& body, Link* baseLink, Link* targetLink);
    virtual ~CustomJointPath();
    virtual bool calcInverseKinematics(const Position& T) override;
    virtual bool hasAnalyticalIK() const override;
};

}


CustomJointPath::CustomJointPath(const BodyPtr& body, Link* baseLink, Link* targetLink)
    : JointPath(baseLink, targetLink),
      body(body)
{
    onJointPathUpdated();
}


CustomJointPath::~CustomJointPath()
{

}


void CustomJointPath::onJointPathUpdated()
{
    ikTypeId = body->customizerInterface()->initializeAnalyticIk(
        body->customizerHandle(), baseLink()->index(), endLink()->index());
    if(ikTypeId){
        isCustomizedIkPathReversed = false;
    } else {
        // try reversed path
        ikTypeId = body->customizerInterface()->initializeAnalyticIk(
            body->customizerHandle(), endLink()->index(), baseLink()->index());
        if(ikTypeId){
            isCustomizedIkPathReversed = true;
        }
    }
}


bool CustomJointPath::hasAnalyticalIK() const
{
    return (ikTypeId != 0);
}


bool CustomJointPath::calcInverseKinematics(const Position& T)
{
    if(isNumericalIkEnabled() || ikTypeId == 0){
        return JointPath::calcInverseKinematics(T);
    }
        
    const Link* baseLink_ = baseLink();
    Vector3 p;
    Matrix3 R;
    
    if(!isCustomizedIkPathReversed){
        p = baseLink_->R().transpose() * (T.translation() - baseLink_->p());
        R.noalias() = baseLink_->R().transpose() * T.linear();
    } else {
        p = T.linear().transpose() * (baseLink_->p() - T.translation());
        R.noalias() = T.linear().transpose() * baseLink_->R();
    }

    bool solved = body->customizerInterface()->calcAnalyticIk(body->customizerHandle(), ikTypeId, p, R);

    if(solved){
        calcForwardKinematics();
    }

    return solved;
}
#endif  // UCNOID_NOT_SUPPORTED

inline JointPathPtr getCustomJointPath(Body* body, Link* baseLink, Link* targetLink)
{
#if UCNOID_NOT_SUPPORTED
    if(body->customizerInterface() && body->customizerInterface()->initializeAnalyticIk){
        return std::make_shared<CustomJointPath>(body, baseLink, targetLink);
    } else {
        return std::make_shared<JointPath>(baseLink, targetLink);
    }
#else   // UCNOID_NOT_SUPPORTED
    return std::make_shared<JointPath>(baseLink, targetLink);
#endif  // UCNOID_NOT_SUPPORTED
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_JOINT_PATH_CPP_H
