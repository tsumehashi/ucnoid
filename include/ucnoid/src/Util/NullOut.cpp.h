/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_NULL_OUT_CPP_H
#define UCNOID_UTIL_NULL_OUT_CPP_H

#include "NullOut.h"
#if UCNOID_NOT_SUPPORTED
#include <boost/iostreams/concepts.hpp>
#include <boost/iostreams/stream_buffer.hpp>
#else  // UCNOID_NOT_SUPPORTED
#include <streambuf>
#endif  // UCNOID_NOT_SUPPORTED

namespace cnoid {
inline namespace ucnoid {

namespace detail::null_out {

#if UCNOID_NOT_SUPPORTED
class NullSink : public boost::iostreams::sink
{
public:
    std::streamsize write(const char*, std::streamsize n) { return n; }
};

#else  // UCNOID_NOT_SUPPORTED

class NullStreamBuf : public std::streambuf {
public:
};
#endif  // UCNOID_NOT_SUPPORTED

}   // namespace detail::null_out

//! \todo check if this is thread safe ?
inline std::ostream& nullout()
{
#if UCNOID_NOT_SUPPORTED
    static detail::null_out::NullSink nullSink;
    static boost::iostreams::stream_buffer<detail::null_out::NullSink> sbuf(nullSink);
#else  // UCNOID_NOT_SUPPORTED
    static detail::null_out::NullStreamBuf sbuf;
#endif  // UCNOID_NOT_SUPPORTED
    static std::ostream os(&sbuf);
        
    return os;
}

}   // inline namespace ucnoid
}

#endif  // UCNOID_UTIL_NULL_OUT_CPP_H
