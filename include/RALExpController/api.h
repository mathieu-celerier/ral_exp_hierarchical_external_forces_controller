#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RALExpController_DLLIMPORT __declspec(dllimport)
#  define RALExpController_DLLEXPORT __declspec(dllexport)
#  define RALExpController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RALExpController_DLLIMPORT __attribute__((visibility("default")))
#    define RALExpController_DLLEXPORT __attribute__((visibility("default")))
#    define RALExpController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RALExpController_DLLIMPORT
#    define RALExpController_DLLEXPORT
#    define RALExpController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RALExpController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RALExpController_DLLAPI
#  define RALExpController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RALExpController_EXPORTS
#    define RALExpController_DLLAPI RALExpController_DLLEXPORT
#  else
#    define RALExpController_DLLAPI RALExpController_DLLIMPORT
#  endif // RALExpController_EXPORTS
#  define RALExpController_LOCAL RALExpController_DLLLOCAL
#endif // RALExpController_STATIC