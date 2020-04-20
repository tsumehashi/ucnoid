/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H
#define UCNOID_BODY_BASIC_SENSOR_SIMULATION_HELPER_H

#include "DeviceList.h"
#include "ForceSensor.h"
#include "RateGyroSensor.h"
#include "AccelerationSensor.h"
#include "exportdecl.h"

namespace cnoid {
inline namespace ucnoid {

class Body;
class BasicSensorSimulationHelperImpl;

class UCNOID_EXPORT BasicSensorSimulationHelper
{
public:
    BasicSensorSimulationHelper();
    ~BasicSensorSimulationHelper();

    void setOldAccelSensorCalcMode(bool on);
    
    void initialize(Body* body, double timeStep, const Vector3& gravityAcceleration);

    bool isActive() const { return isActive_; }
    bool hasGyroOrAccelerationSensors() const { return !rateGyroSensors_.empty() || !accelerationSensors_.empty(); }

    const DeviceList<ForceSensor>& forceSensors() const { return forceSensors_; }
    const DeviceList<RateGyroSensor>& rateGyroSensors() const { return rateGyroSensors_; }
    const DeviceList<AccelerationSensor>& accelerationSensors() const { return accelerationSensors_; }
        
    void updateGyroAndAccelerationSensors();

private:
    BasicSensorSimulationHelperImpl* impl;
    bool isActive_;
    DeviceList<ForceSensor> forceSensors_;
    DeviceList<RateGyroSensor> rateGyroSensors_;
    DeviceList<AccelerationSensor> accelerationSensors_;

    friend class BasicSensorSimulationHelperImpl;
};

}   // inline namespace ucnoid
}

#include "BasicSensorSimulationHelper.cpp.h"

#endif
