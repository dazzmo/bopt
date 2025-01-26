#include "bopt/ad/casadi/evaluator/differentiable.hpp"

namespace bopt {
namespace casadi {
namespace evaluator {
namespace differentiable {

scalar::scalar(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool densify, bool codegen)
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

return_status scalar::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                value_t &out) {
    fun_({x.data(), this->parameters().data()}, {&out});
    return return_status::Success;
}

return_status scalar::eval_gradient_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_vector_t> out) {
    grd_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status scalar::eval_gradient_impl(
    const Eigen::Ref<const dense_vector_t> &x, sparse_vector_t &out) {
    grd_({x.data(), this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

void scalar::get_gradient_sparsity_impl(sparse_vector_t &out) const {
    set_eigen_sparsity(out, grd_.sparsity_out(0));
}

return_status scalar::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda,
    Eigen::Ref<dense_matrix_t> out) {
    hes_({x.data(), lambda.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status scalar::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
    hes_({x.data(), lambda.data(), this->parameters().data()},
         {out.valuePtr()});
    return return_status::Success;
}

void scalar::get_hessian_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, hes_.sparsity_out(0));
}

vector::vector(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool densify, bool codegen)
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

return_status vector::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_vector_t> out) {
    fun_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status vector::eval_jacobian_impl(
    const Eigen::Ref<const dense_vector_t> &x, Eigen::Ref<dense_matrix_t> out) {
    jac_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status vector::eval_jacobian_impl(
    const Eigen::Ref<const dense_vector_t> &x, sparse_matrix_t &out) {
    jac_({x.data(), this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

void vector::get_jacobian_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, jac_.sparsity_out(0));
}

return_status vector::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda,
    Eigen::Ref<dense_matrix_t> out) {
    hes_({x.data(), lambda.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status vector::eval_hessian_impl(
    const Eigen::Ref<const dense_vector_t> &x,
    const Eigen::Ref<const dense_vector_t> &lambda, sparse_matrix_t &out) {
    hes_({x.data(), lambda.data(), this->parameters().data()},
         {out.valuePtr()});
    return return_status::Success;
}

void vector::get_hessian_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, hes_.sparsity_out(0));
}

}  // namespace differentiable
}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt