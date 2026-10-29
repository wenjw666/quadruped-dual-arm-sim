#include <dlfcn.h>
#include <stdint.h>
#include <stdio.h>

typedef int32_t VkResult;
typedef uint32_t VkFlags;
typedef uint32_t VkStructureType;
typedef struct VkApplicationInfo {
    VkStructureType sType; const void *pNext; const char *pApplicationName;
    uint32_t applicationVersion; const char *pEngineName; uint32_t engineVersion;
    uint32_t apiVersion;
} VkApplicationInfo;
typedef struct VkInstanceCreateInfo {
    VkStructureType sType; const void *pNext; VkFlags flags;
    const VkApplicationInfo *pApplicationInfo; uint32_t enabledLayerCount;
    const char *const *ppEnabledLayerNames; uint32_t enabledExtensionCount;
    const char *const *ppEnabledExtensionNames;
} VkInstanceCreateInfo;
typedef VkResult (*PFN_vkEnumerateInstanceVersion)(uint32_t *);
typedef VkResult (*PFN_vkCreateInstance)(const VkInstanceCreateInfo *, const void *, void **);
typedef VkResult (*PFN_vkEnumeratePhysicalDevices)(void *, uint32_t *, void **);
typedef void (*PFN_vkDestroyInstance)(void *, const void *);

int main(void) {
    void *lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!lib) { puts("dlopen_failed"); return 2; }
    PFN_vkEnumerateInstanceVersion ver = (PFN_vkEnumerateInstanceVersion)dlsym(lib, "vkEnumerateInstanceVersion");
    PFN_vkCreateInstance create = (PFN_vkCreateInstance)dlsym(lib, "vkCreateInstance");
    PFN_vkEnumeratePhysicalDevices enumerate = (PFN_vkEnumeratePhysicalDevices)dlsym(lib, "vkEnumeratePhysicalDevices");
    PFN_vkDestroyInstance destroy = (PFN_vkDestroyInstance)dlsym(lib, "vkDestroyInstance");
    uint32_t api = 0; VkResult rv = ver ? ver(&api) : -999;
    printf("enumerate_version_result=%d api=%u.%u.%u create_symbol=%s\n", rv,
           api >> 22, (api >> 12) & 0x3ff, api & 0xfff, create ? "yes" : "no");
    if (create && enumerate) {
        VkApplicationInfo app = {0, 0, "c1200-probe", 1, "probe", 1, api};
        VkInstanceCreateInfo info = {1, 0, 0, &app, 0, 0, 0, 0};
        void *instance = 0;
        VkResult cr = create(&info, 0, &instance);
        printf("create_instance_result=%d instance=%s\n", cr, instance ? "yes" : "no");
        if (cr == 0 && instance) {
            uint32_t count = 0; VkResult er = enumerate(instance, &count, 0);
            printf("enumerate_physical_devices_result=%d count=%u\n", er, count);
            if (destroy) destroy(instance, 0);
        }
    }
    dlclose(lib); return 0;
}
