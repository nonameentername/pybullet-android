#!/usr/bin/env sh

export ROOTDIR=$(dirname $(readlink -f $0))

export NDK="$HOME/source/android-ndk"
export SDK="$HOME/source/android-sdk/"
export NDKPLATFORM="$NDK/platforms/android-9/arch-arm"

export PATH="$NDK/toolchains/arm-linux-androideabi-4.4.3/prebuilt/linux-x86/bin/:$NDK:$SDK/tools:$PATH"

ndk-build

rm -r $ROOTDIR/bullet/bin/*
cp $ROOTDIR/libs/armeabi/*.so $ROOTDIR/bullet/bin/
