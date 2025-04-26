option(WITH_QPOASES "Build with qpOASES functionality" OFF)
option(WITH_GUROBI "Build with GUROBI functionality" OFF)
option(WITH_IPOPT "Build with IPOPT functionality" OFF)
option(WITH_CLP "Build with Clp functionality" OFF)

# Optional solvers to include 

# qpOASES
if(WITH_QPOASES)
  message(STATUS "Including qpOASES as numeric optimization solver")
  find_package(qpOASES REQUIRED)
  target_sources(${PROJECT_NAME} PRIVATE qpoases.cpp)
  target_link_libraries(${PROJECT_NAME} PUBLIC qpOASES)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_WITH_QPOASES)
endif(WITH_QPOASES)

# GUROBI
if(WITH_GUROBI)
  message(STATUS "Including GUROBI as a numeric optimization solver")
  find_package(GUROBI REQUIRED)
  target_include_directories(${PROJECT_NAME} PUBLIC ${GUROBI_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${GUROBI_CXX_LIBRARY})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${GUROBI_LIBRARY})
  target_sources(${PROJECT_NAME} PRIVATE gurobi.cpp)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_WITH_GUROBI)
endif(WITH_GUROBI)

# CLP
if(WITH_CLP)
  message(STATUS "Including Clp as a numeric optimization solver")
  find_package(Clp REQUIRED)
  target_include_directories(${PROJECT_NAME} PUBLIC ${CLP_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${CLP_LIBRARIES})
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_WITH_CLP)
endif(WITH_CLP)

# IPOPT
if(WITH_IPOPT)
  message(STATUS "Including IPOPT as a numeric optimization solver")
  find_package(ipopt REQUIRED)
  target_link_libraries(${PROJECT_NAME} PUBLIC ipopt)
  target_sources(${PROJECT_NAME} PRIVATE ipopt.cpp)
  target_compile_definitions(${PROJECT_NAME} PUBLIC BOPT_WITH_IPOPT)
endif(WITH_IPOPT)