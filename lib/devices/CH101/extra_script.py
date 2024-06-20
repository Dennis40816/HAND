Import('env')

# 定義支援的平臺列表
SUPPORTED_PLATFORMS = ["espidf", "dummy"]

def set_src_filter(env, platform):
    # 檢查平臺是否被支援
    if platform not in SUPPORTED_PLATFORMS:
        raise ValueError(f"Unsupported platform: {platform}. Supported platforms are: {SUPPORTED_PLATFORMS}")
    
    # 排除所有 platform 資料夾中的文件
    exclude_filter = "-<platform/>"
    
    # 包含指定平台的文件
    include_filter = f"+<platform/{platform}/*>"

    # 固定添加其他目錄
    fixed_includes = [
        "+<application/inc>",
        "+<drivers/chirpmicro/inc>",
        "+<ultrasound/inc>",
        "+<board/config>",
        "+<drivers/chirpmicro/src>"
    ]

    # 合併過濾規則
    src_filter = [exclude_filter, include_filter] + fixed_includes

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
