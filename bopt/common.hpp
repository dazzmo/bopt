#pragma once

#include <cassert>
#if DEBUG
#define DBGASSERT(assertion) assert(assertion);
#else
#define DBGASSERT(assertion)
#endif

// Boost-style setters and getters

// Boost-style setters and getters
template <typename T, typename V>
void set(T p, const V& v) {
    p = v;
}

template <typename T, typename I, typename V>
void set(T p, I i, const V& v) {
    p[i] = v;
}

template <typename T, typename I, typename V>
T& get(const T& p, const I& i) {
    return p[i];
}

template <typename T, typename I, typename V>
const T& get(const T& p, const I& i) {
    return p[i];
}

#define bopt_int int