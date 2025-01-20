find_library(IPOPT_LIBRARY
    NAMES ipopt
    PATH_SUFFIXES /usr/local/lib)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(ipopt DEFAULT_MSG IPOPT_LIBRARY)