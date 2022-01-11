#!/bin/bash

mkdir -p build

pushd lib/tco_libd
./build.sh
mv -f build/tco_libd.a ../../build
popd

pushd build
clang \
    -Wall \
    -std=c11 \
    -D _DEFAULT_SOURCE \
    -I ../code \
    -I ../lib/tco_libd/include \
    -I ../lib/tco_shmem \
    ../code/*.c \
    -l rt \
    -l pthread \
	-lm \
    tco_libd.a \
    -o tco_controld.bin
popd
