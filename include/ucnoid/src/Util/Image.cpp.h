/*!
  @file
  @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_IMAGE_CPP_H
#define UCNOID_UTIL_IMAGE_CPP_H

#include "Image.h"
#include "ImageIO.h"
#include "Exception.h"
#if UCNOID_NOT_SUPPORTED
#include <fmt/format.h>
#endif  // UCNOID_NOT_SUPPORTED

namespace cnoid {
inline namespace ucnoid {

inline Image::Image()
{
    width_ = 0;
    height_ = 0;
    numComponents_ = 3;
}


inline Image::Image(const Image& org)
    : pixels_(org.pixels_)
{
    width_ = org.width_;
    height_ = org.height_;
    numComponents_ = org.numComponents_;
}


inline Image::~Image()
{
    
}


inline Image& Image::operator=(const Image& rhs)
{
    pixels_ = rhs.pixels_;
    width_ = rhs.width_;
    height_ = rhs.height_;
    numComponents_ = rhs.numComponents_;
    return *this;
}


inline void Image::reset()
{
    pixels_.clear();
    width_ = 0;
    height_ = 0;
}


inline void Image::setSize(int width, int height, int nComponents)
{
    if(nComponents > 0 && nComponents <= 4){
        numComponents_ = nComponents;
    } else {
        exception_base exception;
#if UCNOID_NOT_SUPPORTED
        exception << error_info_message(
            fmt::format("Invalid number ({}) of image components", nComponents));
        BOOST_THROW_EXCEPTION(exception);
#else   // UCNOID_NOT_SUPPORTED
        exception << "error_info_message : " << "Invalid number (" << std::to_string(nComponents) << ") of image components\n";
        UCNOID_THROW_EXCEPTION(exception);
#endif  // UCNOID_NOT_SUPPORTED
    }
    setSize(width, height);
}


inline void Image::setSize(int width, int height)
{
    width_ = width;
    height_ = height;
    pixels_.resize(numComponents_ * width_ * height_);
}


inline void Image::clear()
{
    std::fill(pixels_.begin(), pixels_.end(), 0);
}


inline void Image::applyVerticalFlip()
{
    const int heightHalf = height_ / 2;
    const int lineSize = width_ * numComponents_;
    unsigned char* upperLine = pixels();
    unsigned char* lowerLine = pixels() + lineSize * (height_ - 1);
    for(int y = 0; y < heightHalf; ++y){
        for(int x=0; x < lineSize; ++x){
            std::swap(upperLine[x], lowerLine[x]);
        }
        upperLine += lineSize;
        lowerLine -= lineSize;
    }
}


    
inline void Image::load(const std::string& filename)
{
    ImageIO iio;
    iio.load(*this, filename);
}


inline void Image::save(const std::string& filename) const
{
    ImageIO iio;
    iio.save(*this, filename);
}

}   // inline namespace ucnoid
}   // namespace cnoid

#endif  // UCNOID_UTIL_IMAGE_CPP_H
