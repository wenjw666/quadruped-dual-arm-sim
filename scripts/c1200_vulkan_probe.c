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
typedef void (*PFN_vkGetPhysicalDeviceProperties)(void *, void *);

int main(void) {
    void *lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!lib) { puts("dlopen_failed"); return 2; }
    PFN_vkEnumerateInstanceVersion ver = (PFN_vkEnumerateInstanceVersion)dlsym(lib, "vkEnumerateInstanceVersion");
    PFN_vkCreateInstance create = (PFN_vkCreateInstance)dlsym(lib, "vkCreateInstance");
    PFN_vkEnumeratePhysicalDevices enumerate = (PFN_vkEnumeratePhysicalDevices)dlsym(lib, "vkEnumeratePhysicalDevices");
    PFN_vkDestroyInstance destroy = (PFN_vkDestroyInstance)dlsym(lib, "vkDestroyInstance");
    PFN_vkGetPhysicalDeviceProperties props = (PFN_vkGetPhysicalDeviceProperties)dlsym(lib, "vkGetPhysicalDeviceProperties");
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
            if (er == 0 && count && props) {
                void *devices[8] = {0}; uint32_t n = count > 8 ? 8 : count;
                if (enumerate(instance, &n, devices) == 0) {
                    unsigned char raw[1024] = {0}; props(devices[0], raw);
                    const uint32_t *u = (const uint32_t *)raw;
                    const char *name = (const char *)(raw + 20);
                    printf("device_name=%s vendor=0x%x device=0x%x api=%u.%u.%u\n",
                           name, u[2], u[3], u[0] >> 22, (u[0] >> 12) & 0x3ff, u[0] & 0xfff);
                }
            }
            if (destroy) destroy(instance, 0);
        }
    }
    dlclose(lib); return 0;
}
