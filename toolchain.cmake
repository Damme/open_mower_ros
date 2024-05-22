set(CMAKE_SYSTEM_NAME Linux)
#set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# Specify the cross compiler
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -static")

# Specify the target system root
set(CMAKE_SYSROOT /opt/sysroot_arm64)

set(SYSROOT_PATH ${CMAKE_SYSROOT})

unset(CMAKE_SYSTEM_PREFIX_PATH)
list(APPEND CMAKE_SYSTEM_PREFIX_PATH ${CMAKE_SYSROOT}/opt/ros/noetic)

set(CMAKE_PREFIX_PATH ${CMAKE_SYSROOT}/opt/ros/noetic ${CMAKE_SYSROOT}/opt/ros/noetic/share)
set(CMAKE_LIBRARY_PATH ${CMAKE_SYSROOT}/opt/ros/noetic/lib)
set(CMAKE_INCLUDE_PATH ${CMAKE_SYSROOT}/opt/ros/noetic/include)
set(CMAKE_FIND_ROOT_PATH ${CMAKE_SYSROOT})

# Search for programs in the build host directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# For libraries and headers in the target directories
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(CMAKE_MODULE_PATH ${CMAKE_SYSROOT}/usr/share/cmake-3.13/Modules/)
set(ENV{PKG_CONFIG_PATH} ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/pkgconfig/)


