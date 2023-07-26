#!/bin/bash

set -e

GREEN='\033[0;32m'
NC='\033[0m'

script_dir="$(cd "$(dirname "$0")" >/dev/null 2>&1 && pwd )"

if [ ! -d $script_dir/app ]
then
    printf "Please run this script in the root of project\n"
    exit 1
fi


root_dir=$script_dir
build_dir=$root_dir/build
app_dir=$root_dir/app

if [ -e $build_dir ]
then
    read -p "Build directory already exists. Do you want to delete it? [y/n]"$'\n' -n 1 -r REPLY
    if [[ $REPLY =~ ^[Yy]$ ]]
    then
        rm -rf $build_dir
        mkdir $build_dir
    fi
fi

cmake -S "$app_dir" \
    -B "$build_dir" \
    -GNinja \
    -DSEL4_CACHE_DIR="$root_dir/.sel4_cache" \
    -DPLATFORM=rpi4 \
    -DVM_LINUX=1 \
    -DVM_LINUX_APP=normal \

echo -e "${GREEN}CMake Configuration complete\n${NC}"

ninja -C $build_dir

echo -e "${GREEN}Build complete${NC}"
echo -e "${GREEN}Image included path:${NC} $build_dir/images/"
