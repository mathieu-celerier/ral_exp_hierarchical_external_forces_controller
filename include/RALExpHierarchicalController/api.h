#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RALExpHierarchicalController_DLLIMPORT __declspec(dllimport)
#  define RALExpHierarchicalController_DLLEXPORT __declspec(dllexport)
#  define RALExpHierarchicalController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RALExpHierarchicalController_DLLIMPORT __attribute__((visibility("default")))
#    define RALExpHierarchicalController_DLLEXPORT __attribute__((visibility("default")))
#    define RALExpHierarchicalController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RALExpHierarchicalController_DLLIMPORT
#    define RALExpHierarchicalController_DLLEXPORT
#    define RALExpHierarchicalController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RALExpHierarchicalController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RALExpHierarchicalController_DLLAPI
#  define RALExpHierarchicalController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RALExpHierarchicalController_EXPORTS
#    define RALExpHierarchicalController_DLLAPI RALExpHierarchicalController_DLLEXPORT
#  else
#    define RALExpHierarchicalController_DLLAPI RALExpHierarchicalController_DLLIMPORT
#  endif // RALExpHierarchicalController_EXPORTS
#  define RALExpHierarchicalController_LOCAL RALExpHierarchicalController_DLLLOCAL
#endif // RALExpHierarchicalController_STATIC