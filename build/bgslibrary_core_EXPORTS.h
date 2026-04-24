
#ifndef bgslibrary_core_EXPORTS_H
#define bgslibrary_core_EXPORTS_H

#ifdef BGSLIBRARY_CORE_EXPORTS_BUILT_AS_STATIC
#  define bgslibrary_core_EXPORTS
#  define BGSLIBRARY_CORE_NO_EXPORT
#else
#  ifndef bgslibrary_core_EXPORTS
#    ifdef bgslibrary_core_EXPORTS
        /* We are building this library */
#      define bgslibrary_core_EXPORTS __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define bgslibrary_core_EXPORTS __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef BGSLIBRARY_CORE_NO_EXPORT
#    define BGSLIBRARY_CORE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef BGSLIBRARY_CORE_DEPRECATED
#  define BGSLIBRARY_CORE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef BGSLIBRARY_CORE_DEPRECATED_EXPORT
#  define BGSLIBRARY_CORE_DEPRECATED_EXPORT bgslibrary_core_EXPORTS BGSLIBRARY_CORE_DEPRECATED
#endif

#ifndef BGSLIBRARY_CORE_DEPRECATED_NO_EXPORT
#  define BGSLIBRARY_CORE_DEPRECATED_NO_EXPORT BGSLIBRARY_CORE_NO_EXPORT BGSLIBRARY_CORE_DEPRECATED
#endif

/* NOLINTNEXTLINE(readability-avoid-unconditional-preprocessor-if) */
#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef BGSLIBRARY_CORE_NO_DEPRECATED
#    define BGSLIBRARY_CORE_NO_DEPRECATED
#  endif
#endif

#endif /* bgslibrary_core_EXPORTS_H */
