/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_BODY_CAMERA_CPP_H
#define UCNOID_BODY_CAMERA_CPP_H

#include "Camera.h"

namespace cnoid {
inline namespace ucnoid {

inline const char* Camera::typeName()
{
    return "Camera";
}


inline Camera::Camera()
{
    on_ = true;
    isImageStateClonable_ = false;
    imageType_ = COLOR_IMAGE;
    lensType_ = NORMAL_LENS;
    resolutionX_ = 640;
    resolutionY_ = 480;
    fieldOfView_ = 0.785398;
    nearClipDistance_ = 0.04;
    farClipDistance_ = 200.0;
    frameRate_ = 30.0;
    delay_ = 0.0;
    image_ = std::make_shared<Image>();
}


inline Camera::Camera(const Camera& org, bool copyStateOnly)
    : Device(org, copyStateOnly),
      image_(org.image_)
{
    if(copyStateOnly){
        isImageStateClonable_ = true;
    } else {
        isImageStateClonable_ = org.isImageStateClonable_;
    }

    copyCameraStateFrom(org);
}


inline void Camera::copyStateFrom(const DeviceState& other)
{
    if(typeid(other) != typeid(Camera)){
        throw std::invalid_argument("Type mismatch in the Device::copyStateFrom function");
    }
    copyStateFrom(static_cast<const Camera&>(other));
}


inline void Camera::copyStateFrom(const Camera& other)
{
    copyCameraStateFrom(other);
    image_ = other.image_;
}


inline void Camera::copyCameraStateFrom(const Camera& other)
{
    on_ = other.on_;
    imageType_ = other.imageType_;
    lensType_ = other.lensType_;
    resolutionX_ = other.resolutionX_;
    resolutionY_ = other.resolutionY_;
    fieldOfView_ = other.fieldOfView_;
    nearClipDistance_ = other.nearClipDistance_;
    farClipDistance_ = other.farClipDistance_;
    frameRate_ = other.frameRate_;
    delay_ = other.delay_;

    if(other.isImageStateClonable_){
        image_ = other.image_;
    } else {
        image_ = std::make_shared<Image>();
    }
}


inline Device* Camera::clone() const
{
    return new Camera(*this, false);
}


inline DeviceState* Camera::cloneState() const
{
    return new Camera(*this, true);
}


inline void Camera::forEachActualType(std::function<bool(const std::type_info& type)> func)
{
    if(!func(typeid(Camera))){
        Device::forEachActualType(func);
    }
}


inline void Camera::clearState()
{
    clearImage();
}


inline void Camera::clearImage()
{
    if(image_.use_count() == 1){
        image_->clear();
    } else {
        image_ = std::make_shared<Image>();
    }
}    


inline bool Camera::on() const
{
    return on_;
}


inline void Camera::on(bool on)
{
    on_ = on;
}


inline Image& Camera::image()
{
    if(image_.use_count() > 1){
        image_ = std::make_shared<Image>(*image_);
    }
    return *image_;
}


inline Image& Camera::newImage()
{
    image_ = std::make_shared<Image>();
    return *image_;
}


inline void Camera::setImage(std::shared_ptr<Image>& image)
{
    if(image.use_count() == 1){
        image_ = image;
    } else {
        image_ = std::make_shared<Image>(*image);
    }
    image.reset();
}


inline int Camera::stateSize() const
{
    return 8;
}


inline const double* Camera::readState(const double* buf)
{
    on_ = buf[0];
    resolutionX_ = buf[1];
    resolutionY_ = buf[2];
    nearClipDistance_ = buf[3];
    farClipDistance_ = buf[4];
    fieldOfView_ = buf[5];
    frameRate_ = buf[6];
    delay_ = buf[7];
    return buf + 8;
}


inline double* Camera::writeState(double* out_buf) const
{
    out_buf[0] = on_ ? 1.0 : 0.0;
    out_buf[1] = resolutionX_;
    out_buf[2] = resolutionY_;
    out_buf[3] = nearClipDistance_;
    out_buf[4] = farClipDistance_;
    out_buf[5] = fieldOfView_;
    out_buf[6] = frameRate_;
    out_buf[7] = delay_;
    return out_buf + 8;
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_BODY_CAMERA_CPP_H
