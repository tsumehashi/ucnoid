#ifndef UCNOID_BODY_EXPORTDECL_H_INCLUDED
# define UCNOID_BODY_EXPORTDECL_H_INCLUDED
# if defined _WIN32 || defined __CYGWIN__
#  define UCNOID_BODY_DLLIMPORT __declspec(dllimport)
#  define UCNOID_BODY_DLLEXPORT __declspec(dllexport)
#  define UCNOID_BODY_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define UCNOID_BODY_DLLIMPORT __attribute__ ((visibility("default")))
#   define UCNOID_BODY_DLLEXPORT __attribute__ ((visibility("default")))
#   define UCNOID_BODY_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define UCNOID_BODY_DLLIMPORT
#   define UCNOID_BODY_DLLEXPORT
#   define UCNOID_BODY_DLLLOCAL
#  endif
# endif

# ifdef UCNOID_BODY_STATIC
#  define UCNOID_BODY_DLLAPI
#  define UCNOID_BODY_LOCAL
# else
#  ifdef CnoidBody_EXPORTS
#   define UCNOID_BODY_DLLAPI UCNOID_BODY_DLLEXPORT
#  else
#   define UCNOID_BODY_DLLAPI UCNOID_BODY_DLLIMPORT
#  endif
#  define UCNOID_BODY_LOCAL UCNOID_BODY_DLLLOCAL
# endif

#endif

#ifdef UCNOID_EXPORT
# undef UCNOID_EXPORT
#endif
#define UCNOID_EXPORT UCNOID_BODY_DLLAPI
