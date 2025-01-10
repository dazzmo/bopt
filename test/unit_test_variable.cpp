#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/variable.hpp"

TEST(Variable, SingleVariable) {
    bopt::variable x("x");

    EXPECT_EQ(x.id(), 0);
    EXPECT_EQ(x.name(), "x");

    bopt::variable empty;

    EXPECT_EQ(empty.id(), 1);
    EXPECT_EQ(empty.name(), "");
}

TEST(Variable, VariableVector) {
    Eigen::Index n;

    n = 10;
    bopt::variable_vector x = bopt::create_variable_vector("x", n);
    EXPECT_EQ(x.size(), n);
    for (Eigen::Index i = 0; i < n; ++i) {
        EXPECT_EQ(x[i].name(), "x_" + std::to_string(i));
    }

    n = 1000;
    x = bopt::create_variable_vector("x", n);
    EXPECT_EQ(x.size(), n);
    for (Eigen::Index i = 0; i < n; ++i) {
        EXPECT_EQ(x[i].name(), "x_" + std::to_string(i));
    }

    n = -1;
    ASSERT_DEATH({ x = bopt::create_variable_vector("x", n); }, "sz >= 0");
}

TEST(Variable, VariableIndices) {
    Eigen::Index n;

    std::vector<Eigen::Index> indices;
    indices = {0, 1, 2, 3, 4, 5};

    bopt::variable_indices vi(indices);

    EXPECT_EQ(vi.indices().size(), indices.size());
    EXPECT_TRUE(vi.is_block());

    indices = {10, 6, 1, 0, 4, 7, 8, 9};
    vi.set_indices(indices);
    EXPECT_EQ(vi.indices().size(), indices.size());
    EXPECT_FALSE(vi.is_block());
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::profiler summary;
    return status;
}