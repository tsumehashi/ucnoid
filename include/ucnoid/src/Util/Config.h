#ifndef UCNOID_CONFIG_H
#define UCNOID_CONFIG_H

#define UCNOID_MAJOR_VERSION 1
#define UCNOID_MINOR_VERSION 7
#define UCNOID_PATCH_VERSION 0
#define UCNOID_INTERNAL_VERSION 1

#define UCNOID_VERSION_STRING "1.7"
#define UCNOID_FULL_VERSION_STRING "1.7.0"
#define UCNOID_PLUGIN_SUBDIR "lib/choreonoid-1.7"
#define UCNOID_SHARE_SUBDIR "share/choreonoid-1.7"

/* #define UCNOID_USE_PYBIND11 */
/* #undef UCNOID_USE_BOOST_PYTHON */
/* #undef UCNOID_USE_PYTHON2 */
/* #undef UCNOID_USE_BOOST_REGEX */

#if defined _WIN32 || defined __CYGWIN__
#define UCNOID_GENERAL_EXPORT __declspec(dllexport)
#else
#if __GNUC__ >= 4
#define UCNOID_GENERAL_EXPORT __attribute__ ((visibility("default")))
#endif
#endif

#endif
