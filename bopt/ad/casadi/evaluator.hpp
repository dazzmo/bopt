#include <memory>

#include "bopt/dlib_handler.hpp"
#include "bopt/evaluator.hpp"

namespace bopt {

namespace casadi {

// This evaluator class is based strongly on that of FATROP, using the same
// interfacing strategy

/* Typedefs based on casadi format */
typedef long long int casadi_int;

typedef void (*signal_t)(void);

typedef casadi_int (*getint_t)(void);
typedef int (*work_t)(casadi_int *sz_arg, casadi_int *sz_res, casadi_int *sz_iw,
                      casadi_int *sz_w);

typedef const casadi_int *(*sparsity_t)(casadi_int ind);

typedef int (*eval_t)(const double **arg, double **res, casadi_int *iw,
                      double *w, int mem);

typedef int (*casadi_checkout_t)(void);
typedef void (*casadi_release_t)(int);

template <typename T>
class Evaluator : public EvaluatorBase<T> {
   private:
    /// pointer to casadi codegen evaluation function
    eval_t f;
    /// casadi int work vector
    casadi_int *iw;
    /// casadi double work vector
    double *w;
    /// increase reference counter
    signal_t incref;
    /// decrease reference counter
    signal_t decref;
    /// input size
    int *input_size;
    /// release casadi memory
    casadi_release_t release;
    /// thread local mem id
    int mem;

    std::vector<double> work_vector_d;
    std::vector<double *> res_vec;
    std::vector<const double *> arg_vec;
    /// int work vector
    std::vector<casadi_int> work_vector_i;

   public:
    /**
     * @brief Construct a new Evaluator object from a code-generated casadi
     * function
     *
     * @param handle
     * @param function_name
     */
    Evaluator(const std::shared_ptr<DynamicLibraryHandler> &handle,
              const std::string &function_name)
        : handle_(handle) {
        void *handle_p = handle_->handle;

        /* Memory management -- increase reference counter */
        incref = (signal_t)dlsym(
            handle_p, (function_name + (std::string) "_incref").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Memory management -- decrease reference counter */
        decref = (signal_t)dlsym(
            handle_p, (function_name + (std::string) "_decref").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Thread-local memory management -- checkout memory */
        casadi_checkout_t checkout = (casadi_checkout_t)dlsym(
            handle_p, (function_name + (std::string) "_checkout").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* T memory management -- release memory */
        release = (casadi_release_t)dlsym(
            handle_p, (function_name + (std::string) "_release").c_str());
        if (dlerror()) dlerror();  // No such function, reset error flags

        /* Number of inputs */
        getint_t n_in_fcn = (getint_t)dlsym(
            handle_p, (function_name + (std::string) "_n_in").c_str());
        // if (dlerror()) return 1;
        n_in = n_in_fcn();

        /* Number of outputs */
        getint_t n_out_fcn = (getint_t)dlsym(
            handle_p, (function_name + (std::string) "_n_out").c_str());
        // if (dlerror()) return 1;
        casadi_int n_out = n_out_fcn();
        assert(n_out == 1);

        // Checkout thread-local memory (not thread-safe)
        // Note MAX_NUM_THREADS
        mem = checkout();

        /* Get sizes of the required work vectors */
        casadi_int sz_arg = n_in, sz_res = n_out, sz_iw = 0, sz_w = 0;
        work_t work = (work_t)dlsym(
            handle_p, (function_name + (std::string) "_work").c_str());

        if (dlerror()) dlerror();  // No such function, reset error flags

        assert((work && work(&sz_arg, &sz_res, &sz_iw, &sz_w)) == 0);

        work_vector_d.resize(sz_w);
        work_vector_i.resize(sz_iw);
        res_vec.resize(sz_res);
        arg_vec.resize(sz_arg);
        w = work_vector_d.data();
        iw = work_vector_i.data();

        /* Input sparsities */

        /* Output sparsities */
        sparsity_t sp_out = (sparsity_t)dlsym(
            handle_p, (function_name + (std::string) "_sparsity_out").c_str());
        assert(dlerror() == 0);

        const casadi_int *sparsity_out_ci = sp_out(0);
        out_m = sparsity_out_ci[0];
        out_n = sparsity_out_ci[1];
        out_nnz = sparsity_out_ci[out_n + 2];

        sparsity_out.resize(2 + out_n + 1 + out_nnz, 0);
        sparsity_out.assign(sparsity_out_ci,
                            sparsity_out_ci + 2 + out_n + 1 + out_nnz);

        /* Function for numerical evaluation */
        f = (eval_t)dlsym(handle_p, function_name.c_str());
        if (dlerror()) {
            printf("Failed to retrieve \"f\" function.\n");
        }

        buffer.resize(out_nnz, 0.0);
    }

    index_type operator()(const double **arg) override {
        w = work_vector_d.data();
        iw = work_vector_i.data();
        for (int i = 0; i < n_in; i++) {
            arg_vec[i] = arg[i];
        }
        res_vec[0] = buffer.data();
        if (f(arg_vec.data(), res_vec.data(), iw, w, mem)) return 1;
        return 0;
    }

    bopt_int info(evaluator_out_info<Evaluator> &info) {
        info.out_m = this->out_m;
        info.out_n = this->out_n;
        info.out_nnz = this->out_nnz;
    }

    ~Evaluator() {
        // Release thread-local (not thread-safe)
        release(mem);
        // Free memory (thread-safe)
        decref();
    }

   private:
    std::shared_ptr<DynamicLibraryHandler> handle_;
};

}  // namespace casadi
}  // namespace bopt
