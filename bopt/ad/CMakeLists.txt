# Autodiff Utilities

option(WITH_CASADI "Build with CasADi functionality" OFF)

if(WITH_CASADI)
  message(STATUS "Adding CasADi functionality to bopt")
  find_package(casadi REQUIRED)
  target_sources(${PROJECT_NAME} PRIVATE casadi/utils.cpp casadi/evaluator.cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC casadi)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_WITH_CASADI)
endif(WITH_CASADI)