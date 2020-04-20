#ifndef UCNOID_UTIL_EXPORTDECL_H_INCLUDED
# define UCNOID_UTIL_EXPORTDECL_H_INCLUDED

# if defined _WIN32 || defined __CYGWIN__
#  define UCNOID_UTIL_DLLIMPORT __declspec(dllimport)
#  define UCNOID_UTIL_DLLEXPORT __declspec(dllexport)
#  define UCNOID_UTIL_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define UCNOID_UTIL_DLLIMPORT __attribute__ ((visibility("default")))
#   define UCNOID_UTIL_DLLEXPORT __attribute__ ((visibility("default")))
#   define UCNOID_UTIL_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define UCNOID_UTIL_DLLIMPORT
#   define UCNOID_UTIL_DLLEXPORT
#   define UCNOID_UTIL_DLLLOCAL
#  endif
# endif

# ifdef UCNOID_UTIL_STATIC
#  define UCNOID_UTIL_DLLAPI
#  define UCNOID_UTIL_LOCAL
# else
#  ifdef CnoidUtil_EXPORTS
#   define UCNOID_UTIL_DLLAPI UCNOID_UTIL_DLLEXPORT
#  else
#   define UCNOID_UTIL_DLLAPI UCNOID_UTIL_DLLIMPORT
#  endif
#  define UCNOID_UTIL_LOCAL UCNOID_UTIL_DLLLOCAL
# endif

#endif

#ifdef UCNOID_EXPORT
# undef UCNOID_EXPORT
#endif
#define UCNOID_EXPORT UCNOID_UTIL_DLLAPI
