/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_RANGE_SENSOR_CPP_H
#define UCNOID_BODY_RANGE_SENSOR_CPP_H

#include "RangeSensor.h"
#include <ucnoid/EigenUtil>

namespace cnoid {
inline namespace ucnoid {

inline const char* RangeSensor::typeName()
{
    return "RangeSensor";
}


inline RangeSensor::RangeSensor()
{
    on_ = true;
    isRangeDataStateClonable_ = false;
    yawRange_ = radian(120.0);
    yawStep_ = 1.0;
    pitchRange_ = 0.0;
    pitchStep_ = 0.0;
    minDistance_ = 0.1;
    maxDistance_ = 10.0;
    scanRate_ = 10.0;
    delay_ = 0.0;
    rangeData_ = std::make_shared<RangeData>();
}


inline RangeSensor::RangeSensor(const RangeSensor& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      rangeData_(org.rangeData_)
{
    if(copyStateOnly){
        isRangeDataStateClonable_ = true;
    } else {
        isRangeDataStateClonable_ = org.isRangeDataStateClonable_;
    }

    copyRangeSensorStateFrom(org);
}


inline void RangeSensor::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(RangeSensor)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const RangeSensor&>(other));
}


inline void RangeSensor::copyStateFrom(const RangeSensor& other)
{
    copyRangeSensorStateFrom(other);
    rangeData_ = other.rangeData_;
}


inline void RangeSensor::copyRangeSensorStateFrom(const RangeSensor& other)
{
    on_ = other.on_;
    yawRange_ = other.yawRange_;
    yawStep_ = other.yawStep_;
    pitchRange_ = other.pitchRange_;
    pitchStep_ = other.pitchStep_;
    minDistance_ = other.minDistance_;
    maxDistance_ = other.maxDistance_;
    scanRate_ = other.scanRate_;
    delay_ = other.delay_;

    if(other.isRangeDataStateClonable_){
        rangeData_ = other.rangeData_;
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


inline Device* RangeSensor::clone() const
{
    return new RangeSensor(*this, false);
}


inline DeviceState* RangeSensor::cloneState() const
{
    return new RangeSensor(*this, true);
}


inline void RangeSensor::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(RangeSensor))){
        Device::forEachActualType(func);
    }
}


inline int RangeSensor::numYawSamples() const
{
    if(yawStep_ > 0.0){
        return static_cast<int>(yawRange_ / yawStep_ + 1.0e-7) + 1;
    } else {
        return 1;
    }
}


inline int RangeSensor::numPitchSamples() const
{
    if(pitchStep_ > 0.0){
        return static_cast<int>(pitchRange_ / pitchStep_ + 1.0e-7) + 1;
    } else {
        return 1;
    }
}


inline RangeSensor::RangeData& RangeSensor::rangeData()
{
    if(rangeData_.use_count() > 1){
        rangeData_ = std::make_shared<RangeData>(*rangeData_);
    }
    return *rangeData_;
}


inline RangeSensor::RangeData& RangeSensor::newRangeData()
{
    rangeData_ = std::make_shared<RangeData>();
    return *rangeData_;
}


inline void RangeSensor::setRangeData(std::shared_ptr<RangeData>& data)
{
    if(data.use_count() == 1){
        rangeData_ = data;
    } else {
        rangeData_ = std::make_shared<RangeData>(*data);
    }
    data.reset();
}


inline void RangeSensor::clearState()
{
    clearRangeData();
}


inline void RangeSensor::clearRangeData()
{
    if(rangeData_.use_count() == 1){
        rangeData_->clear();
    } else {
        rangeData_ = std::make_shared<RangeData>();
    }
}


inline bool RangeSensor::on() const
{
    return on_;
}


inline void RangeSensor::on(bool on)
{
    on_ = on;
}

inline int RangeSensor::stateSize() const
{
    return 9;
}


inline const double* RangeSensor::readState(const double* buf)
{
    on_ = buf[0];
    yawRange_ = buf[1];
    yawStep_ = buf[2];
    pitchRange_ = buf[3];
    pitchStep_ = buf[4];
    minDistance_ = buf[5];
    maxDistance_ = buf[6];
    scanRate_ = buf[7];
    delay_ = buf[8];
    return buf + 9;
}


inline double* RangeSensor::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = yawRange_;
    out_buf[2] = yawStep_;
    out_buf[3] = pitchRange_;
    out_buf[4] = pitchStep_;
    out_buf[5] = minDistance_;
    out_buf[6] = maxDistance_;
    out_buf[7] = scanRate_;
    out_buf[8] = delay_;
    return out_buf + 9;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_RANGE_SENSOR_CPP_H
