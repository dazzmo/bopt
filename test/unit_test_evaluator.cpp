#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

class DenseEvaluatorTest : public bopt::DenseEvaluatorTpl<double, 2> {
    using Base = bopt::DenseEvaluatorTpl<double, 2>;

   public:
    using Data = typename Base::Data;

    DenseEvaluatorTest() : Base(2, 1, "Dense evaluator") {}

    void setDataSparsityImpl(Data &data) const override {
        data.Jx.resize(2, 2);
    }

    void eval(const typename Base::InputVectorConstRef &x, Data &data) const {
        data.y << 1.0;
    }
};

class SparseEvaluator : public bopt::SparseEvaluatorTpl<double, 2> {
    using Base = bopt::SparseEvaluatorTpl<double, 2>;
    using Data = typename Base::Data;

   public:
    SparseEvaluator() : Base(2, 1, "Sparse evaluator") {}

    void setDataSparsityImpl(Data &data) const override {
        data.Jx.resize(2, 2);
    }

    void eval(const typename Base::InputVectorConstRef &x, Data &data) const {
        data.y << 1.0;
    }
};

TEST(DenseEvaluator, Constructor) {
    DenseEvaluatorTest e;
    EXPECT_EQ(e.numInputs(), 2);

    std::cout << e << std::endl;

    DenseEvaluatorTest::Data data(e);

    Eigen::VectorXd x(2);
    x.setRandom();
    {
        bopt::Profiler profiler("DenseEvaluatorTest");
        e.eval(x, data);
    }
}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}