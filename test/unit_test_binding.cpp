#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include "bopt/binding.hpp"
#include "bopt/constraints.hpp"
#include "bopt/Logging.hpp"
#include "bopt/profiler.hpp"

class DummyConstraint : public bopt::constraint_tpl<double> {
   public:
    DummyConstraint() : bopt::constraint_tpl<double>(2, 2) {
        this->set_name("dummy_constraint");
    }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[0];
        out[0] = x[0] - x[1];
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 0) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }
};

class DummyLinearConstraint : public bopt::linear_constraint_tpl<double> {
   public:
    DummyLinearConstraint() : bopt::linear_constraint_tpl<double>(2, 2) {
        this->set_name("dummy_linear_constraint");
    }

   protected:
    bopt::evaluator::return_status eval_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_vector_t> out) override {
        out[0] = x[0];
        out[0] = x[0] - x[1];
        return bopt::evaluator::return_status::Success;
    }

    bopt::evaluator::return_status eval_jacobian_impl(
        const Eigen::Ref<const dense_vector_t> &x,
        Eigen::Ref<dense_matrix_t> out) override {
        out(0, 0) = 1.0;
        out(1, 0) = 1.0;
        out(1, 1) = -1.0;
        return bopt::evaluator::return_status::Success;
    }
};

TEST(Variable, EmptyBinding) {
    bopt::binding<bopt::constraint_tpl<double>> b;

    EXPECT_DEATH({ b.get(); }, "");
    EXPECT_DEATH({ b.indices(); }, "");
}

TEST(Variable, ConstraintBinding) {
    std::vector<Eigen::Index> indices = {0, 1};
    auto constraint = std::make_shared<DummyConstraint>();
    bopt::binding<bopt::constraint_tpl<double>> b(constraint, indices);

    EXPECT_EQ(b.indices().indices().size(), 2);
    EXPECT_EQ(b.get()->sz_out(), 2);

    // Incorrect indice vector size
    indices = {0, 1, 2};
    EXPECT_DEATH(
        {
            bopt::binding<bopt::constraint_tpl<double>> binding_incorrect(
                constraint, indices);
        },
        "");
}

TEST(Variable, ConstraintBindingConversion) {
    std::vector<Eigen::Index> indices = {0, 1};
    auto lc = std::make_shared<DummyLinearConstraint>();
    bopt::binding<bopt::linear_constraint_tpl<double>> bl(lc, indices);

    bopt::binding<bopt::constraint_tpl<double>> b(bl);
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