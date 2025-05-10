#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/Evaluator.hpp"
#include "bopt/Logging.hpp"
// #include "bopt/profiler.hpp"

class DenseEvaluator : public bopt::DenseEvaluatorTpl<double> {
    using Base = bopt::DenseEvaluatorTpl<double>;
    using Data = typename Base::Data;

   public:
    DenseEvaluator() : Base(2, 1, "Dense evaluator") {}

    std::shared_ptr<Data> createData() override {
        auto res = std::make_shared<Data>(*this);
        res->Jx.resize(2, 2);
        return res;
    }

    void eval(const typename Base::InputVectorConstRef &x, Data &data) const {
        data.y << 1.0;
    }
};

class SparseEvaluator : public bopt::SparseEvaluatorTpl<double> {
    using Base = bopt::SparseEvaluatorTpl<double>;
    using Data = typename Base::Data;

   public:
    SparseEvaluator() : Base(2, 1, "Sparse evaluator") {}

    std::shared_ptr<Data> createData() override {
        auto res = std::make_shared<Data>(*this);
        res->Jx.resize(2, 2);
        return res;
    }
};

TEST(DenseEvaluator, Constructor) {
    DenseEvaluator e;
    EXPECT_EQ(e.getInputDimension(), 2);

    std::cout << e << std::endl;

    auto data = e.createData();

    Eigen::VectorXd x(2);
    x.setRandom();
    e.eval(x, *data);

}

int main(int argc, char **argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    // bopt::profiler summary;
    return status;
}