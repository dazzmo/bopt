find_package(Boost REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(CppAD REQUIRED)
find_package(glog REQUIRED)

option(WITH_QPOASES "Build with qpOASES functionality" OFF)
option(WITH_GUROBI "Build with qpOASES functionality" OFF)
option(WITH_IPOPT "Build with IPOPT functionality" OFF)

# Collect source files
SET(${PROJECT_NAME}_SOURCES
profiler.cpp
common.cpp
evaluator.cpp
costs.cpp
variable.cpp
)

SET(${PROJECT_NAME}_INCLUDE_DIRS
)

SET(${PROJECT_NAME}_LIBRARIES
Eigen3::Eigen
${CPPAD_LIBRARIES}
glog
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${${PROJECT_NAME}_LIBRARIES})

include(GNUInstallDirs)

option(BOPT_USE_PROFILING "Use profiling" OFF)

if(BOPT_USE_PROFILING)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_USE_PROFILING)
endif(BOPT_USE_PROFILING)

add_subdirectory(ad)
add_subdirectory(solvers)

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
