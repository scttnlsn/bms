#!/usr/bin/env bash

export ZEPHYR_BASE=./zephyr
source $ZEPHYR_BASE/zephyr-env.sh
mkdir -p $ZEPHYR_BASE/build && cd $ZEPHYR_BASE/build
cmake $ZEPHYR_BASE/scripts
make
export PATH=$ZEPHYR_BASE/build/kconfig:$PATH
source $ZEPHYR_BASE/zephyr-env.sh
cd $ZEPHYR_BASE
cd ..

export ZEPHYR_GCC_VARIANT=gccarmemb
export GCCARMEMB_TOOLCHAIN_PATH=/usr
mkdir -p build && cd build
cmake ..
cd ..
