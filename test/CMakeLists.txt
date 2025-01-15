find_package(GTest REQUIRED)
find_package(gflags REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})
link_directories(${GTEST_LIB_DIRS})

enable_testing()

# Create list of files
SET(TEST_FILES
unit_test_expression.cpp
unit_test_costs.cpp
unit_test_constraints.cpp
unit_test_binding.cpp
unit_test_variable.cpp
unit_test_program.cpp
unit_test_qpoases.cpp
)

include(GoogleTest)

foreach(current_file IN LISTS TEST_FILES)
    get_filename_component(current_folder ${current_file} DIRECTORY)
    get_filename_component(current_name ${current_file} NAME_WE)
    add_executable("${current_name}" ${current_file})
    target_link_libraries("${current_name}" bopt GTest::gtest_main glog gflags)
endforeach()
