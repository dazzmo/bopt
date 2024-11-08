#pragma once

#include <dlfcn.h>

#include <iostream>
#include <string>

namespace bopt {
class DynamicLibraryHandler {
   public:
    DynamicLibraryHandler(const std::string& filename) {
        handle = dlopen(&filename[0], RTLD_LAZY);
        if (handle == 0) {
            std::cout << "Cannot open dynamic library or shared object "
                      << filename << ": " << dlerror() << '\n';
        }
    }

    ~DynamicLibraryHandler() { dlclose(handle); }

    void* handle;

   private:
};
}  // namespace bopt
