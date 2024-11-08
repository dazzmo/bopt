#include "bopt/profiler.hpp"
#include "bopt/logging.hpp"

#include <gflags/gflags.h>
#include <gtest/gtest.h>


TEST(Control, Task) {

  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
