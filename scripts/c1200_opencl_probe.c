#include <dlfcn.h>
#include <stdio.h>
#include <stdint.h>
typedef int32_t cl_int;
typedef uint32_t cl_uint;
typedef cl_int (*PFN_clGetPlatformIDs)(cl_uint, void **, cl_uint *);
int main(void) {
    void *lib = dlopen("libOpenCL.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!lib) { puts("dlopen_failed"); return 2; }
    PFN_clGetPlatformIDs f = (PFN_clGetPlatformIDs)dlsym(lib, "clGetPlatformIDs");
    cl_uint n = 0; cl_int rv = f ? f(0, 0, &n) : -999;
    printf("clGetPlatformIDs_result=%d count=%u symbol=%s\n", rv, n, f ? "yes" : "no");
    dlclose(lib); return 0;
}
