/**
   @author Shin'ichiro Nakaoka
*/

#ifndef UCNOID_UTIL_EXCEPTION_H
#define UCNOID_UTIL_EXCEPTION_H

#if UCNOID_NOT_SUPPORTED
#include <boost/exception/all.hpp>
#else   // UCNOID_NOT_SUPPORTED
#include <string>
#endif  // UCNOID_NOT_SUPPORTED
#include <exception>

namespace cnoid {
inline namespace ucnoid {

#if UCNOID_NOT_SUPPORTED

struct exception_base : virtual std::exception, virtual boost::exception { };

typedef boost::error_info<struct tag_error_info_message, std::string> error_info_message;

struct nonexistent_key_error : virtual exception_base { };

typedef boost::error_info<struct tag_error_info_key, std::string> error_info_key;

struct type_mismatch_error : virtual exception_base { };

struct file_read_error : virtual exception_base { };

struct empty_data_error : virtual exception_base { };

#else   // UCNOID_NOT_SUPPORTED

class exception_base : public virtual std::exception
{
public:
    exception_base() {}
    virtual ~exception_base() {}

    void set_info(const char* current_function, const char* file, int line) {
        throw_function_ = current_function;
        throw_file_ = file;
        throw_line_ = line;
    }

    void set_message(const std::string& m) {
        message_ = m;
    }

    void add_message(const std::string& m) {
        message_ += m;
    }

    const std::string& throw_function() const noexcept { return throw_function_; }
    const std::string& throw_file() const noexcept { return throw_file_; }
    int throw_line() const noexcept { return throw_line_; }

    const std::string& message() const noexcept { return message_; }
    virtual const char* what() const noexcept override { return message_.c_str(); }

private:
    std::string throw_function_;
    std::string throw_file_;
    int throw_line_ = -1;
    std::string message_;
};

inline exception_base& operator<<(exception_base& e, const std::string& message)
{
    e.add_message(message);
    return e;
}

inline std::string diagnostic_information(exception_base& e)
{
    std::string s;
    s += "[<throw_file>] = " + e.throw_file() + "\n";
    s += "[<throw_function>] = " + e.throw_function() + "\n";
    s += "[<throw_line>] = " + std::to_string(e.throw_line()) + "\n";

    return s;
}

template <typename E>
inline void throw_exception(E& e, const char* current_function, const char* file, int line)
{
    e.set_info(current_function, file, line);
    throw e;
}

#define UCNOID_THROW_EXCEPTION(x) throw_exception((x),__func__,__FILE__,__LINE__)

struct nonexistent_key_error : virtual exception_base { };

struct type_mismatch_error : virtual exception_base { };

struct file_read_error : virtual exception_base { };

struct empty_data_error : virtual exception_base { };

#endif  // UCNOID_NOT_SUPPORTED

}   // inline namespace ucnoid
}

#endif

