
#include "bopt/profiler.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <boost/mpl/accumulate.hpp>

#include "bopt/logging.h"

TEST(Profiler, Basic) {
  for (int i = 0; i < 100; ++i) {
    bopt::core::Profiler profile1("loop1");

    for (int j = 0; j < 50; ++j) {
      bopt::core::Profiler profile1("loop2");
      bopt::core::Profiler profile2("loop3");
    }
  }

  bopt::core::Profiler profiler_summary;

  EXPECT_TRUE(true);
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
