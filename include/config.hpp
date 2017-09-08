#pragma once
#ifdef _MSC_VER
#  ifdef dpct_EXPORTS
#    define DPCT_API  __declspec(dllexport)
#  else
#    define DPCT_API  __declspec(dllimport)
#  endif
#else
#  define DPCT_API
#endif