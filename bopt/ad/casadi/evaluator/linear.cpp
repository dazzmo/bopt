
#pragma once

#include "bopt/ad/casadi/evaluator/linear.hpp"

namespace bopt {
namespace casadi {
namespace evaluator {
namespace linear {

vector::vector(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool densify, bool codegen)
    : base_t(x.size1(), expression.rows(), p.size1()) {
    // Compute coefficients
    sym_t A, b;
    sym_t::linear_coeff(expression, x, A, b, true);

    f_ = create_function("f", {x, p}, {expression}, true, codegen);
    A_ = create_function("A", {p}, {A}, densify, codegen);
    b_ = create_function("b", {p}, {b}, densify, codegen);
}

return_status vector::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_vector_t> out) {
    f_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

void vector::get_A_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, A_.sparsity_out(0));
}

return_status vector::eval_A_impl(Eigen::Ref<dense_matrix_t> out) {
    A_({this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status vector::eval_A_impl(sparse_matrix_t &out) {
    A_({this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

return_status vector::eval_b_impl(Eigen::Ref<dense_vector_t> out) {
    b_({this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status vector::eval_b_impl(sparse_vector_t &out) {
    b_({this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

void vector::get_b_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, b_.sparsity_out(0));
}

scalar::scalar(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool densify, bool codegen)
    : base_t(x.size1(), p.size1()) {
    // Compute coefficients
    sym_t a, b;
    sym_t::linear_coeff(expression, x, a, b, true);

    f_ = create_function("f", {x, p}, {expression}, true, codegen);
    a_ = create_function("A", {p}, {a}, densify, codegen);
    b_ = create_function("b", {p}, {b}, true, codegen);
}

return_status scalar::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                value_t &out) {
    f_({x.data(), this->parameters().data()}, {&out});
    return return_status::Success;
}

void scalar::get_a_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, a_.sparsity_out(0));
}

return_status scalar::eval_a_impl(Eigen::Ref<dense_vector_t> out) {
    a_({this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status scalar::eval_a_impl(sparse_vector_t &out) {
    a_({this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

return_status scalar::eval_b_impl(value_t &out) {
    b_({this->parameters().data()}, {&out});
    return return_status::Success;
}

}  // namespace linear
}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt