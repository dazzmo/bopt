#pragma once

// #ifdef BOPT_WITH_CASADI

#include <casadi/casadi.hpp>
#include <string>
#include <vector>

struct CasadiExpression {
    using SX = ::casadi::SX;
    std::string name;
    SX expr;
    SX x;
    SX p;

    CasadiExpression(const std::string &name, const SX &expr, const SX &x,
                     const casadi::SX &p)
        : name(name), expr(expr), x(x), p(p) {}
};

inline std::vector<CasadiExpression> getScalarTestExpressions(
    std::size_t n = 4) {
    using SX = ::casadi::SX;

    std::vector<CasadiExpression> cases;

    // Variables
    SX x = SX::sym("x", n);
    // Parameters
    SX p = SX::sym("p", n);

    // Linear expression
    cases.push_back({"linear", dot(p, x), x, p});
    // Trigonometric expression
    cases.push_back({"trigonometric", sin(dot(p, x)), x, p});

    // Quadratic expression
    SX quad = 0;
    for (std::size_t i = 0; i < n; ++i) quad += p(i) * x(i) * x(i);
    cases.push_back({"quadratic", quad, x, p});

    // Mixed expression
    SX mixed = cos(p(0) * x(0)) + exp(p(1) * x(1));
    cases.push_back({"composite", mixed, x, p});

    return cases;
}

inline std::vector<CasadiExpression> getVectorTestExpressions(
    std::size_t n = 4) {
    using SX = ::casadi::SX;

    std::vector<CasadiExpression> cases;

    // Variables
    SX x = SX::sym("x", n);
    // Parameters
    SX p = SX::sym("p", n);

    // Linear expression
    cases.push_back({"linear", SX::mtimes(SX::rand(n, n), x) + p, x, p});

    return cases;
}

// #endif