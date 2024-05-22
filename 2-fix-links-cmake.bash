SYSROOT=/opt/sysroot_arm64

echo "Linking some missing library links, broken links /hardcoded to /lib/..."

ln -sfv $SYSROOT/usr/lib/aarch64-linux-gnu/libPocoFoundation.so.62 $SYSROOT/usr/lib/aarch64-linux-gnu/libPocoFoundation.so
ln -sfv $SYSROOT/usr/lib/aarch64-linux-gnu/blas/libblas.so.3 $SYSROOT/usr/lib/aarch64-linux-gnu/libblas.so.3
ln -sfv $SYSROOT/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3 $SYSROOT/usr/lib/aarch64-linux-gnu/liblapack.so.3

ln -sfv $SYSROOT/lib/aarch64-linux-gnu/librt.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/librt.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libpthread.so.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libpthread.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libBrokenLocale.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libBrokenLocale.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libanl.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libanl.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libblkid.so.1.1.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libblkid.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libbz2.so.1.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libbz2.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libexpat.so.1.6.11 $SYSROOT/usr/lib/aarch64-linux-gnu/libexpat.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libgpg-error.so.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libgpg-error.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libkeyutils.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libkeyutils.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/liblzma.so.5.2.4 $SYSROOT/usr/lib/aarch64-linux-gnu/liblzma.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libm.so.6 $SYSROOT/usr/lib/aarch64-linux-gnu/libm.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libmount.so.1.1.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libmount.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnsl.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libnsl.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_compat.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_compat.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_dns.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_dns.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_files.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_files.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_hesiod.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_hesiod.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_nis.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_nis.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libnss_nisplus.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libnss_nisplus.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libpcre.so.3 $SYSROOT/usr/lib/aarch64-linux-gnu/libpcre.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libresolv.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libresolv.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libthread_db.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libthread_db.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libutil.so.1 $SYSROOT/usr/lib/aarch64-linux-gnu/libutil.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libuuid.so.1.3.0 $SYSROOT/usr/lib/aarch64-linux-gnu/libuuid.so
ln -sfv $SYSROOT/lib/aarch64-linux-gnu/libz.so.1.2.11 $SYSROOT/usr/lib/aarch64-linux-gnu/libz.so
ln -svf $SYSROOT/lib/aarch64-linux-gnu/libdl.so.2 $SYSROOT/usr/lib/aarch64-linux-gnu/libdl.so

echo "Fix cmake files with hardcoded paths.."
find $SYSROOT/opt/ros/noetic/ -name "*.cmake" | xargs -i sh -c 'echo {}; sed -i s@/opt/ros/noetic/lib@\${SYSROOT_PATH}/opt/ros/noetic/lib@g {}'
find $SYSROOT/opt/ros/noetic/ -name "*.cmake" | xargs -i sh -c 'echo {}; sed -i s@PREFIX\ /opt/ros/noetic@PREFIX\ \${SYSROOT_PATH}/opt/ros/noetic@g {}'
find $SYSROOT/opt/ros/noetic/ -name "*.cmake" | xargs -i sh -c 'echo {}; sed -i s@\;/usr/@\;\$\{SYSROOT_PATH\}/usr/@g {}'
find $SYSROOT/opt/ros/noetic/ -name "*.cmake" | xargs -i sh -c 'echo {}; sed -i s@\"/usr/@\"\$\{SYSROOT_PATH\}/usr/@g {}'




#sed -i s@\;/usr/@\;\$\{SYSROOT_PATH\}/usr/@g $SYSROOT/opt/ros/noetic/share/class_loader/cmake/class_loaderConfig.cmake
#sed -i s@\;/usr/@\;\$\{SYSROOT_PATH\}/usr/@g $SYSROOT/opt/ros/noetic/share/roscpp/cmake/roscppConfig.cmake
#sed -i s@\;/usr/@\;\$\{SYSROOT_PATH\}/usr/@g $SYSROOT/opt/ros/noetic/share/mbf_costmap_nav/cmake/mbf_costmap_navConfig.cmake



#/opt/sysroot_arm64/opt/ros/noetic/share/pluginlib/cmake/pluginlibConfig.cmake:
#set(libraries "/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libtinyxml2.so")
#/opt/sysroot_arm64/opt/ros/noetic/share/rospack/cmake/rospackConfig.cmake:
#set(libraries "rospack;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpython3.8.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_program_options.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libtinyxml2.so")
#/opt/sysroot_arm64/opt/ros/noetic/share/roscpp/cmake/roscppConfig.cmake:
#set(libraries "roscpp;pthread;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_chrono.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_system.so.1.71.0")
#/opt/sysroot_arm64/opt/ros/noetic/share/rosbag_storage/cmake/rosbag_storageConfig.cmake:
#set(libraries "rosbag_storage;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_filesystem.so.1.71.0")
#/opt/sysroot_arm64/opt/ros/noetic/share/pcl_conversions/cmake/pcl_conversionsConfig.cmake:
#set(libraries "/usr/lib/aarch64-linux-gnu/libpcl_common.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpcl_octree.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpcl_io.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_system.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_filesystem.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_date_time.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_iostreams.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libboost_regex.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkChartsCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonColor-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtksys-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonDataModel-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonMath-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonMisc-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonSystem-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonTransforms-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonExecutionModel-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersGeneral-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkCommonComputationalGeometry-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkInfovisCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersExtraction-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersStatistics-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingFourier-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkalglib-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingContext2D-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersGeometry-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersSources-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingFreeType-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libfreetype.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libz.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersModeling-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingSources-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkInteractionStyle-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkInteractionWidgets-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkFiltersHybrid-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingColor-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingGeneral-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkImagingHybrid-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOImage-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkDICOMParser-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkmetaio-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libjpeg.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libpng.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libtiff.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingAnnotation-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingVolume-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOXML-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOXMLParser-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libexpat.so;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOGeometry-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOLegacy-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkIOPLY-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingLOD-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkViewsContext2D-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkViewsCore-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingContextOpenGL2-7.1.so.7.1p.1;${SYSROOT_PATH}/usr/lib/aarch64-linux-gnu/libvtkRenderingOpenGL2-7.1.so.7.1p.1")
