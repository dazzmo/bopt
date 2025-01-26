#include "bopt/ad/casadi/evaluator.hpp"

namespace bopt {
namespace casadi {

scalar_evaluator::scalar_evaluator(const sym_t &expression,
                                   const sym_vector_t &x, const sym_vector_t &p,
                                   bool codegen)
    : base_t(x.size1(), p.size1()) {
    DBGASSERT(expression.size1() == 1 && expression.size2() == 1 &&
              "Expression is not scalar!");
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
}

evaluator::return_status scalar_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
    fun_({x.data(), this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

vector_evaluator::vector_evaluator(const sym_t &expression,
                                   const sym_vector_t &x, const sym_vector_t &p,
                                   bool codegen)
    : bopt::vector_evaluator_tpl<double>(x.size1(), expression.size1(),
                                         p.size1()) {
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
}

evaluator::return_status vector_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_vector_t> out) {
    fun_({x.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

linear_vector_evaluator::linear_vector_evaluator(const sym_t &expression,
                                                 const sym_vector_t &x,
                                                 const sym_vector_t &p,
                                                 bool densify, bool codegen)
    : bopt::linear_vector_evaluator_tpl<double>(x.size1(), expression.rows(),
                                                p.size1()) {
    // Compute coefficients
    sym_t A, b;
    sym_t::linear_coeff(expression, x, A, b, true);

    f_ = create_function("f", {x, p}, {expression}, true, codegen);
    A_ = create_function("A", {p}, {A}, densify, codegen);
    b_ = create_function("b", {p}, {b}, densify, codegen);
}

evaluator::return_status linear_vector_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_vector_t> out) {
    f_({x.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

void linear_vector_evaluator::get_A_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, A_.sparsity_out(0));
}

evaluator::return_status linear_vector_evaluator::eval_A_impl(
    Eigen::Ref<dense_matrix_t> out) {
    A_({this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status linear_vector_evaluator::eval_A_impl(
    sparse_matrix_t &out) {
    A_({this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

evaluator::return_status linear_vector_evaluator::eval_b_impl(
    Eigen::Ref<dense_vector_t> out) {
    b_({this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status linear_vector_evaluator::eval_b_impl(
    sparse_vector_t &out) {
    b_({this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

void linear_vector_evaluator::get_b_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, b_.sparsity_out(0));
}

linear_scalar_evaluator::linear_scalar_evaluator(const sym_t &expression,
                                                 const sym_vector_t &x,
                                                 const sym_vector_t &p,
                                                 bool densify, bool codegen)
    : bopt::linear_scalar_evaluator_tpl<double>(x.size1(), p.size1()) {
    // Compute coefficients
    sym_t a, b;
    sym_t::linear_coeff(expression, x, a, b, true);

    f_ = create_function("f", {x, p}, {expression}, true, codegen);
    a_ = create_function("A", {p}, {a}, densify, codegen);
    b_ = create_function("b", {p}, {b}, true, codegen);
}

evaluator::return_status linear_scalar_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
    f_({x.data(), this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

void linear_scalar_evaluator::get_a_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, a_.sparsity_out(0));
}

evaluator::return_status linear_scalar_evaluator::eval_a_impl(
    Eigen::Ref<dense_vector_t> out) {
    a_({this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status linear_scalar_evaluator::eval_a_impl(
    sparse_vector_t &out) {
    a_({this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

evaluator::return_status linear_scalar_evaluator::eval_b_impl(value_t &out) {
    b_({this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

quadratic_scalar_evaluator::quadratic_scalar_evaluator(const sym_t &expression,
                                                       const sym_vector_t &x,
                                                       const sym_vector_t &p,
                                                       bool densify,
                                                       bool codegen)
    : bopt::quadratic_scalar_evaluator_tpl<double>(x.size1(), p.size1()) {
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

evaluator::return_status quadratic_scalar_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
    f_({x.data(), this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

void quadratic_scalar_evaluator::get_A_sparsity_impl(
    sparse_matrix_t &out) const {
    set_eigen_sparsity(out, A_.sparsity_out(0));
}

evaluator::return_status quadratic_scalar_evaluator::eval_A_impl(
    Eigen::Ref<dense_matrix_t> out) {
    A_({this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status quadratic_scalar_evaluator::eval_A_impl(
    sparse_matrix_t &out) {
    A_({this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

evaluator::return_status quadratic_scalar_evaluator::eval_b_impl(
    Eigen::Ref<dense_vector_t> out) {
    b_({this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status quadratic_scalar_evaluator::eval_b_impl(
    sparse_vector_t &out) {
    b_({this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

void quadratic_scalar_evaluator::get_b_sparsity_impl(
    sparse_vector_t &out) const {
    set_eigen_sparsity(out, b_.sparsity_out(0));
}

evaluator::return_status quadratic_scalar_evaluator::eval_c_impl(value_t &out) {
    c_({this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

differentiable_scalar_evaluator::differentiable_scalar_evaluator(
    const sym_t &expression, const sym_vector_t &x, const sym_vector_t &p,
    bool densify, bool codegen)
    : base_t(x.size1(), p.size1()) {
    DBGASSERT(expression.is_scalar() && "Expression is not scalar!");
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
    grd_ = create_function("grd", in, {sym_t::gradient(expression, x)}, densify,
                           codegen);

    // Create hessian
    sym_vector_t l = sym_vector_t::sym("l", expression.size1());
    // Create input list
    in = {};
    in.push_back(x);
    in.push_back(l);
    in.push_back(p);

    hes_ = create_function(
        "hes", in, {sym_t::tril(sym_t::hessian(sym_t::dot(l, expression), x))},
        densify, codegen);
}

evaluator::return_status differentiable_scalar_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, value_t &out) {
    fun_({x.data(), this->parameters().data()}, {&out});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_scalar_evaluator::eval_gradient_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_vector_t> out) {
    grd_({x.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_scalar_evaluator::eval_gradient_impl(
    const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
    grd_({x.data(), this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

void differentiable_scalar_evaluator::get_gradient_sparsity_impl(
    sparse_vector_t &out) const {
    set_eigen_sparsity(out, grd_.sparsity_out(0));
}

evaluator::return_status differentiable_scalar_evaluator::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda,
    Eigen::Ref<dense_matrix_t> out) {
    hes_({x.data(), lambda.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_scalar_evaluator::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
    hes_({x.data(), lambda.data(), this->parameters().data()},
         {out.valuePtr()});
    return evaluator::return_status::Success;
}

void differentiable_scalar_evaluator::get_hessian_sparsity_impl(
    sparse_matrix_t &out) const {
    set_eigen_sparsity(out, hes_.sparsity_out(0));
}

differentiable_vector_evaluator::differentiable_vector_evaluator(
    const sym_t &expression, const sym_vector_t &x, const sym_vector_t &p,
    bool densify, bool codegen)
    : base_t(x.size1(), expression.rows(), p.size1()) {
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
    jac_ = create_function("jacobian", in, {sym_t::jacobian(expression, x)},
                           densify, codegen);

    // Create hessian
    sym_vector_t l = sym_vector_t::sym("l", expression.size1());
    // Create input list
    in = {};
    in.push_back(x);
    in.push_back(l);
    in.push_back(p);

    hes_ = create_function(
        "hes", in, {sym_t::tril(sym_t::hessian(sym_t::dot(l, expression), x))},
        densify, codegen);
}

evaluator::return_status differentiable_vector_evaluator::eval_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_vector_t> out) {
    fun_({x.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_vector_evaluator::eval_jacobian_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_matrix_t> out) {
    jac_({x.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_vector_evaluator::eval_jacobian_impl(
    const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
    jac_({x.data(), this->parameters().data()}, {out.valuePtr()});
    return evaluator::return_status::Success;
}

void differentiable_vector_evaluator::get_jacobian_sparsity_impl(
    sparse_matrix_t &out) const {
    set_eigen_sparsity(out, jac_.sparsity_out(0));
}

evaluator::return_status differentiable_vector_evaluator::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda,
    Eigen::Ref<dense_matrix_t> out) {
    hes_({x.data(), lambda.data(), this->parameters().data()}, {out.data()});
    return evaluator::return_status::Success;
}

evaluator::return_status differentiable_vector_evaluator::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
    hes_({x.data(), lambda.data(), this->parameters().data()},
         {out.valuePtr()});
    return evaluator::return_status::Success;
}

void differentiable_vector_evaluator::get_hessian_sparsity_impl(
    sparse_matrix_t &out) const {
    set_eigen_sparsity(out, hes_.sparsity_out(0));
}

}  // namespace casadi
}  // namespace bopt