find_package(Boost REQUIRED)
find_package(casadi REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(glog REQUIRED)

option(WITH_QPOASES "Build with qpOASES functionality" OFF)
option(WITH_IPOPT "Build with IPOPT functionality" OFF)

# Collect source files
SET(${PROJECT_NAME}_SOURCES
profiler.cpp
variable.cpp
solvers/base.cpp
ad/casadi/codegen.cpp
)

SET(${PROJECT_NAME}_LIBRARIES
Eigen3::Eigen
casadi
glog
)

include(GNUInstallDirs)

option(BOPT_USE_PROFILING "Use profiling" OFF)

if(BOPT_USE_PROFILING)
  add_definitions(-DBOPT_USE_PROFILING)
endif()

# Optional solvers to include 

# qpOASES
if(WITH_QPOASES)
  message(STATUS "Including qpOASES as numeric optimization solver")
  find_package(qpOASES REQUIRED)
  list(APPEND ${PROJECT_NAME}_LIBRARIES qpOASES)
  list(APPEND ${PROJECT_NAME}_SOURCES solvers/qpoases.cpp)
  add_compile_definitions(WITH_QPOASES)
endif(WITH_QPOASES)

# IPOPT
if(WITH_IPOPT)
  message(STATUS "Including IPOPT as numeric optimization solver")
  list(APPEND ${PROJECT_NAME}_LIBRARIES ipopt)
  list(APPEND ${PROJECT_NAME}_SOURCES solvers/ipopt.cpp)
  add_compile_definitions(WITH_IPOPT)
endif(WITH_IPOPT)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_LIBRARIES})

set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>"
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wno-unused-parameter -pipe)

add_library(bopt::bopt ALIAS ${PROJECT_NAME})

## First of all, we need to install the library itself.
install(TARGETS ${PROJECT_NAME}
    EXPORT bopt-export
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    INCLUDES DESTINATION include
    RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
