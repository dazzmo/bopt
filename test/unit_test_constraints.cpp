#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Constraints.hpp"
#include "bopt/Logging.hpp"
#include "bopt/Profiler.hpp"

class DenseEvaluatorTest : public bopt::DenseEvaluatorTpl<double, 2> {
    using Base = bopt::DenseEvaluatorTpl<double, 2>;

   public:
    using Data = typename Base::Data;

    DenseEvaluatorTest() : Base(2, 2, "Dense evaluator") {}

    void setDataSparsityImpl(Data &data) const override {
        data.Jx.resize(2, 2);
    }

    void evalImpl(const typename Base::InputVectorConstRef &x,
                  Data &data) const {
        data.y << x[0] * x[1], x[0];
    }

    void evalJacobiansImpl(const typename Base::InputVectorConstRef &x,
                           Data &data, bool compute_x,
                           bool compute_p) const override {
        data.Jx << x[1], x[0], 1.0, 0.0;
    }
};

TEST(Constraint, Constraint) {
    auto e = std::make_shared<DenseEvaluatorTest>();

    bopt::DenseConstraint<2> c(e, bopt::ConstraintBoundType::POSITIVE);

    auto data = c.createData();
    {
        for (int i = 0; i < 100; ++i) {
            bopt::Profiler profiler("bounds");
            c.evalBounds(data);
        }
    }

    bopt::Logger::info() << data.lb.transpose();
    bopt::Logger::info() << data.ub.transpose();
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    bopt::Profiler summary;
    return status;
}