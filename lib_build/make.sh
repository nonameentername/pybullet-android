here=`dirname $0`
bullet=$here/../bullet-2.75-rc7/src
dynamics=$bullet/BulletDynamics/libBulletDynamics.a
collision=$bullet/BulletCollision/libBulletCollision.a
math=$bullet/LinearMath/libLinearMath.a

rm -f $here/_bullet.o $here/_bullet.so
g++ -I$bullet -c -fPIC $here/bullet.cpp -o $here/bullet.o
g++ -shared -Wl,-soname,_bullet.so -o $here/_bullet.so $here/bullet.o $dynamics $collision $math
cp $here/_bullet.so $here/../bullet
