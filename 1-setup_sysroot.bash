#!/bin/bash

set -e

SYSROOT=/opt/sysroot_arm64
DOWNLOADDIR=/opt/sysroot_downloads
PARALLELDOWNLOAD=10

#########################

rm -rf $SYSROOT
mkdir -p $SYSROOT
mkdir -p $DOWNLOADDIR

# Link might not be needed? Cant remember :D
ln -s $SYSROOT/usr/lib $SYSROOT/lib

install_package() {
#  for PKG in "${PACKAGES[@]}"; do
    PKG=$1
    URL=$(apt-get download --print-uris $PKG | cut -d\' -f2)
    if [ -n "$URL" ]; then
      echo "Downloading $PKG ($URL)"
      wget -q -nc $URL -P $DOWNLOADDIR && dpkg -x $DOWNLOADDIR/$(basename $URL) $SYSROOT
# --show-progress
#      dpkg -x $DOWNLOADDIR/$(basename $URL) $SYSROOT
      echo "Done installing $PKG"
    else
      echo "$PKG NOT FOUND!"
    fi
#  done
}

export -f install_package
export DOWNLOADDIR
export SYSROOT

# read packages.txt into PACKAGES
while IFS= read -r line; do
    PACKAGES+=("$line")
done < "packages.txt"


parallel -j $PARALLELDOWNLOAD install_package ::: "${PACKAGES[@]}"


echo "Done!"
