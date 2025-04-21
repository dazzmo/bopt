#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/costs.hpp"
#include "bopt/logging.hpp"
// #include "bopt/profiler.hpp"

class BasicCost : public bopt::DenseCostTpl<double> {
   public:
    BasicCost() : bopt::DenseCostTpl<double>(2) {}

    using Base = bopt::DenseCostTpl<double>;
    using InputVectorConstRef = typename Base::InputVectorConstRef;
    using Data = typename Base::Data;

    std::shared_ptr<Data> createData() override {
        auto data = std::make_shared<Data>(*this);
        return data;
    }

   protected:
    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        data.y = 1.0;
    }

    void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        data.gx << 1.0;
        data.gp << 0.0;
    }
};

class LinearCost : public bopt::SparseLinearCostTpl<double> {
   public:
    LinearCost() : bopt::SparseLinearCostTpl<double>(1000) {}
    
    using LinearData =  typename bopt::SparseLinearCostTpl<double>::Data;
    using Data =  typename bopt::SparseLinearCostTpl<double>::Data;

   protected:
    void evalImpl(const InputVectorConstRef &x, Data &data) const override {
        data.y = 1.0;
    }

    void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
                           bool compute_x, bool compute_p) const override {
        data.gx.valuePtr()[0] = -1.0;
        data.gx.valuePtr()[1] = 1.0;
    }
};

TEST(BasicCost, Construction) {
    BasicCost cost;
    auto data = cost.createData();
}

TEST(LinearCost, Construction) {
    std::shared_ptr<LinearCost> cost = std::make_shared<LinearCost>();
    auto data = cost->createData();

    auto x = Eigen::VectorXd::Zero(cost->getInputDimension());
    cost->evalGradients(x, *data, true, true);

    std::shared_ptr<bopt::SparseCostTpl<double>> ptr = cost;

    auto data2 = ptr->createData();

    std::cout << data->gx.transpose() << std::endl;
}

int main(int argc, char **argv) {
    FLAGS_logtostderr = true;
    FLAGS_v = 10;

    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    testing::InitGoogleTest(&argc, argv);
    int status = RUN_ALL_TESTS();
    // bopt::profiler summary;
    return status;
}