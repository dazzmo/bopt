#pragma once

#include <cassert>
#if DEBUG
#define DBGASSERT(assertion) assert(assertion);
#else
#define DBGASSERT(assertion)
#endif

#define bopt_int int