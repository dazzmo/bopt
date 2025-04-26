# Try to find Clp and CoinUtils libraries and headers
# Defines:
#   Clp_FOUND
#   Clp_INCLUDE_DIRS
#   Clp_LIBRARIES
#   CoinUtils_INCLUDE_DIRS
#   CoinUtils_LIBRARIES
#   Clp::Clp (imported target)

# Find Clp headers and libraries
find_path(CLP_INCLUDE_DIR
  NAMES ClpSimplex.hpp
  PATH_SUFFIXES coin
  PATHS /usr/include /usr/local/include
)

find_library(CLP_LIBRARY
  NAMES Clp
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

# Find CoinUtils headers and libraries
find_path(COINUTILS_INCLUDE_DIR
  NAMES CoinPackedMatrix.hpp
  PATH_SUFFIXES coin
  PATHS /usr/include /usr/local/include
)

find_library(COINUTILS_LIBRARY
  NAMES CoinUtils
  PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
)

# Handle the find results and set variables
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Clp
  DEFAULT_MSG CLP_INCLUDE_DIR CLP_LIBRARY
)

find_package_handle_standard_args(CoinUtils
  DEFAULT_MSG COINUTILS_INCLUDE_DIR COINUTILS_LIBRARY
)

# If both Clp and CoinUtils are found, set up the imported targets
if(CLP_FOUND AND COINUTILS_FOUND)
  set(CLP_INCLUDE_DIRS ${CLP_INCLUDE_DIR} ${COINUTILS_INCLUDE_DIR})
  set(CLP_LIBRARIES ${CLP_LIBRARY} ${COINUTILS_LIBRARY})

  # Create imported targets
  if(NOT TARGET Clp::Clp)
    add_library(Clp::Clp UNKNOWN IMPORTED)
    set_target_properties(Clp::Clp PROPERTIES
      IMPORTED_LOCATION "${CLP_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${CLP_INCLUDE_DIR}"
    )
  endif()

  if(NOT TARGET CoinUtils::CoinUtils)
    add_library(CoinUtils::CoinUtils UNKNOWN IMPORTED)
    set_target_properties(CoinUtils::CoinUtils PROPERTIES
      IMPORTED_LOCATION "${COINUTILS_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${COINUTILS_INCLUDE_DIR}"
    )
  endif()
endif()
