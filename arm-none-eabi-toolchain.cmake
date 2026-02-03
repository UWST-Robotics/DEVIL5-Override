# Indicate cross-compilation
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Compilers
set(CMAKE_C_COMPILER "C:/Users/kuglera2948/pros-toolchain/usr/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "C:/Users/kuglera2948/pros-toolchain/usr/bin/arm-none-eabi-g++.exe")

# Disable trying to compile test programs for the host
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

# Paths for finding includes and libraries
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)

# Additional flags for ARM Cortex V5 (VEX)
set(CMAKE_C_FLAGS "--specs=nosys.specs -mcpu=cortex-m7 -mthumb")
set(CMAKE_CXX_FLAGS "--specs=nosys.specs -mcpu=cortex-m7 -mthumb")
