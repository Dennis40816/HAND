Import('env')
from os.path import join, realpath

def set_src_filter(env, platform):
    if platform == "espidf":
        src_filter = ["+<*>", "+<platform/espidf/*>", "-<platform/dummy/*>"]
    elif platform == "dummy":
        src_filter = ["+<*>", "+<platform/dummy/*>", "-<platform/espidf/*>"]
    # TODO: other platforms
    else:
        raise ValueError("Unsupported platform: %s" % platform)
    
    env.Replace(SRC_FILTER=src_filter)

# 獲取構建標誌中的 LIB_PLATFORM
lib_platform = None
for item in env.get("CPPDEFINES", []):
    if isinstance(item, tuple) and item[0] == "LIB_PLATFORM":
        lib_platform = item[1]
        break

if lib_platform:
    set_src_filter(env, lib_platform)
else:
    print("Warning: LIB_PLATFORM is not defined in build flags.")
