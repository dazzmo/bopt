#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/logging.hpp"
#include "bopt/profiler.hpp"
#include "bopt/program.hpp"

class BasicCost : public bopt::DenseCostTpl<double> {
   public:
    BasicCost() : bopt::DenseCostTpl<double>(1) {}

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

class BasicSparseCost : public bopt::SparseCostTpl<double> {
   public:
    BasicSparseCost() : bopt::SparseCostTpl<double>(1) {}

    using Base = bopt::SparseCostTpl<double>;
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
};

// class LinearCost : public bopt::SparseLinearCostTpl<double> {
//    public:
//     LinearCost() : bopt::SparseLinearCostTpl<double>(1) {}

//     using LinearData = typename bopt::SparseLinearCostTpl<double>::Data;
//     using Data = typename bopt::SparseLinearCostTpl<double>::Data;

//    protected:
//     void evalImpl(const InputVectorConstRef &x, Data &data) const override {
//         data.y = 1.0;
//     }

//     void evalGradientsImpl(const InputVectorConstRef &x, Data &data,
//                            bool compute_x, bool compute_p) const override {
//         data.gx.valuePtr()[0] = -1.0;
//         data.gx.valuePtr()[1] = 1.0;
//     }
// };

TEST(Program, AddCosts) {
    bopt::MathematicalProgram p("program");
    bopt::VariableVector x(1);
    x << p.addVariable("x");

    auto c0 = std::make_shared<BasicCost>();
    auto d0 = c0->createData();
    p.addCost(c0, d0, x);

    auto c1 = std::make_shared<BasicSparseCost>();
    auto d1 = c1->createData();
    p.addCost(c1, d1, x);

    auto c = p.getDenseCosts();

    EXPECT_EQ(c.size(), 1);

    std::cout << *c[0].get() << std::endl;
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