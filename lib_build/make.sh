#!/usr/bin/env bash

here=`dirname $0`
system=`uname`
arch=`python -c "from sys import maxint; print 'x86' if maxint == 0x7fffffff else 'x64'"`

bullet=$here/../bullet-2.75-rc7/src
dynamics=$bullet/BulletDynamics/libBulletDynamics.a
collision=$bullet/BulletCollision/libBulletCollision.a
math=$bullet/LinearMath/libLinearMath.a

rm -f $here/bullet-*.so $here/bullet-*.dylib $here/bullet.o $here/bullet-*.dll
g++ -I$bullet -c -fPIC $here/bullet.cpp -o $here/bullet.o

if [ $system == "Linux" ]; then
    soname="bullet-$arch.so"
    echo "Building: $soname"
    g++ -shared -Wl,-soname,$soname -o $here/$soname $here/bullet.o $dynamics $collision $math
fi
if [ $system == "Darwin" ]; then
    soname="bullet-$arch.dylib"
    echo "Building: $soname"
    g++ -dynamiclib -o $here/$soname $here/bullet.o $dynamics $collision $math
fi
cp $here/$soname $here/../bullet/bin/$soname
