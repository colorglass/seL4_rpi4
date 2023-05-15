#!/usr/bin/bash

if [ -e "$PWD/scripts/build-kernel.sh" ]; then
    TOP_DIR=$PWD
elif [ -e "$PWD/build-kernel.sh" ] || [ -d "../kernel" ]; then
    TOP_DIR=`dirname $PWD`
else
    echo "Please run this script in top directory or \`kernel\' directory"
    exit 1
fi

APP="$1"

if [ "$APP" == "" ]; then
    echo "Please enter app name!"
    echo "Usage: ./build-app.sh <app>"
    echo "List of available apps: comm;comm-smaccm;empty;hello;hello-camkes-1;hello-camkes-2"
    exit 1
fi

BUILD_DIR="$TOP_DIR/build"
APP_DIR="$TOP_DIR/apps/$APP"
BUILD_APP_DIR="$BUILD_DIR/apps/$APP"

echo "TOP_DIR: $TOP_DIR"
echo "App build directory: $BUILD_KERNEL_DIR"

rm -rf $BUILD_APP_DIR
mkdir -p $BUILD_APP_DIR
cd $BUILD_APP_DIR

echo "Configuration start"

# plat="qemu-arm-virt"
# plat="ls1046a"
plat="rpi4"

cmake "$APP_DIR" \
    -GNinja \
    -DSEL4_CACHE_DIR="$TOP_DIR/.sel4_cache" \
    -DPLATFORM=rpi4 \
    -DVM_LINUX=1 \
    -DVM_LINUX_APP=normal \

echo "Configuration complete"

##########################

echo "##########################"
echo "Build start"

ninja -C $BUILD_APP_DIR

echo "Build complete"

##########################

if [ $plat == "qemu-arm-virt" ]; then
    exit 0
fi

echo "##########################"

DEST_DIR="/mnt/d/Resource/tftp"

echo "Copy to shared folder: $DEST_DIR"

cp $BUILD_APP_DIR/images/* $DEST_DIR

if test $? -eq 0; then
    echo "Copy complete"
else
    echo "Copy failed"
fi
