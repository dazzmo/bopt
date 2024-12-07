find_package(GTest REQUIRED)
find_package(gflags REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

enable_testing()

# Create list of files
SET(TEST_FILES
casadi/codegen.cc
# casadi/constraint.cc
solvers/qpoases.cc
# core/sparse.cpp
# core/evaluator.cpp
# core/constraint.cpp
)

include(GoogleTest)

foreach(current_file IN LISTS TEST_FILES)
    get_filename_component(current_folder ${current_file} DIRECTORY)
    get_filename_component(current_name ${current_file} NAME_WE)
    add_executable("${current_folder}_${current_name}_unit_test" ${current_file})
    target_link_libraries("${current_folder}_${current_name}_unit_test" bopt GTest::gtest_main glog gflags)
endforeach()
