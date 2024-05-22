#!/bin/bash

cd src/lib/slic3r_coverage_planner

patch CMakeLists.txt <<'EOF'
@@ -33,7 +33,9 @@ ExternalProject_Add(

         CMAKE_CACHE_ARGS
         SOURCE_SUBDIR src
-        CMAKE_ARGS -DSLIC3R_BUILD_TESTS=OFF
+        CMAKE_ARGS -DSLIC3R_BUILD_TESTS=OFF -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
+               -DCMAKE_EXE_LINKER_FLAGS:STRING=--sysroot=${CMAKE_SYSROOT}\;-L${CMAKE_SYSROOT}/opt/ros/noetic/lib\;-L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
+               -DCMAKE_SHARED_LINKER_FLAGS:STRING=--sysroot=${CMAKE_SYSROOT}\;-L${CMAKE_SYSROOT}/opt/ros/noetic/lib\;-L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
         BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release
 )
EOF

cd ../../../

cd src/lib/xbot_driver_gps

patch CMakeLists.txt <<'EOF'
@@ -32,6 +32,9 @@ ExternalProject_Add(
         INSTALL_COMMAND cmake -E echo "Skipping install step."

         CMAKE_CACHE_ARGS
+        CMAKE_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE}
+               -DCMAKE_EXE_LINKER_FLAGS:STRING=--sysroot=${CMAKE_SYSROOT}\;-L${CMAKE_SYSROOT}/opt/ros/noetic/lib\;-L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
+               -DCMAKE_SHARED_LINKER_FLAGS:STRING=--sysroot=${CMAKE_SYSROOT}\;-L${CMAKE_SYSROOT}/opt/ros/noetic/lib\;-L${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu
         BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> --config Release
 )
EOF

cd ../../../
