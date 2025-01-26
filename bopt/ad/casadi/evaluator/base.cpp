#include "bopt/ad/casadi/evaluator/base.hpp"

namespace bopt {
namespace casadi {
namespace evaluator {

scalar::scalar(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool codegen)
    : base_t(x.size1(), p.size1()) {
    DBGASSERT(expression.size1() == 1 && expression.size2() == 1 &&
              "Expression is not scalar!");
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
}

return_status scalar::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                value_t &out) {
    fun_({x.data(), this->parameters().data()}, {&out});
    return return_status::Success;
}

vector::vector(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool codegen)
    : base_t(x.size1(), expression.size1(), p.size1()) {
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
}

return_status vector::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_vector_t> out) {
    fun_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

matrix::matrix(const sym_t &expression, const sym_vector_t &x,
               const sym_vector_t &p, bool codegen)
    : base_t(x.size1(), expression.size1(), p.size1()) {
    std::vector<sym_vector_t> in = {};
    in.push_back(x);
    in.push_back(p);

    fun_ = create_function("f", in, {expression}, true, codegen);
}

return_status matrix::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                Eigen::Ref<dense_matrix_t> out) {
    fun_({x.data(), this->parameters().data()}, {out.data()});
    return return_status::Success;
}

return_status matrix::eval_impl(const Eigen::Ref<const dense_vector_t> &x,
                                sparse_matrix_t &out) {
    fun_({x.data(), this->parameters().data()}, {out.valuePtr()});
    return return_status::Success;
}

void matrix::get_sparsity_impl(sparse_matrix_t &out) const {
    set_eigen_sparsity(out, fun_.sparsity_out(0));
}

}  // namespace evaluator
}  // namespace casadi
}  // namespace bopt
