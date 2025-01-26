
#pragma once

#include "bopt/ad/casadi/evaluator/quadratic.hpp"

namespace bopt {
namespace casadi {
namespace evaluator {
namespace quadratic {

scalar::scalar(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool densify, bool codegen)
    : base_t(x.size1(), p.size1()) {
    // Compute coefficients
    sym_t A, b, c;
    sym_t::quadratic_coeff(expression, x, A, b, c, true);

    std::vector<sym_vector_t> in = {};
    in.push_back(p);

    f_ = create_function("f", in, {expression}, true, codegen);
    A_ = create_function("A", in, {A}, densify, codegen);
    b_ = create_function("b", in, {b}, densify, codegen);
    c_ = create_function("c", in, {c}, densify, codegen);
}

return_status scalar::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                value_t &out) {
    f_({x.data(), this->parameters().data()}, {&out});
    return return_status::Success;
}

void scalar::get_A_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, A_.sparsity_out(0));
}

return_status scalar::eval_A_impl(Eigen::Ref<dense_matrix_t> out) {
    A_({this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status scalar::eval_A_impl(sparse_matrix_t &out) {
    A_({this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

return_status scalar::eval_b_impl(Eigen::Ref<dense_vector_t> out) {
    b_({this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status scalar::eval_b_impl(sparse_vector_t &out) {
    b_({this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

void scalar::get_b_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, b_.sparsity_out(0));
}

return_status scalar::eval_c_impl(value_t &out) {
    c_({this->parameters().data()}, {&out});
    return return_status::Success;
}

}  // namespace quadratic
}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt