
# Reqired Dependencies

## install Boost 1.63 in ubuntu
download boost 1.63
$ ./bootstrap.sh --prefix=${PWD}/.local --with-libraries=all --with-toolset=gcc
$ ./b2 toolset=gcc -q -d2 -j4 --debug-configuration --disable-filesystem2 --layout=tagged --build-dir=${PWD}/obj cxxflags="-v -std=c++11 -stdlib=libstdc++" linkflags="-stdlib=libstdc++" link=shared threading=multi 
$ ./b2 install --prefix=/usr

c++11 compatible solution:
https://ask.helplib.com/c/11499455

## install Eigen 3
$ mkdir build 
$ cd build 
$ cmake ../ 
$ sudo make install

## install opencv 3 in ubuntu
$ cd ~/opencv-3.1.0  
$ mkdir release  
$ cd release  
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr cmake -DENABLE_PRECOMPILED_HEADERS=OFF..  
$ make  
$ sudo make install  

a good reference is [here][1]


## install PCL  1.8.0
$ cd /pcl 
$ mkdir release 
$ cd release 
$ cmake -D CMAKE_BUILD_TYPE=RELEASE -DBUILD_GPU=ON -DBUILD_apps=ON -D CMAKE_INSTALL_PREFIX=/usr  .. 
$ make 
$ sudo make install 

## install cgal 4.11
$ sudo apt-get install libcgal-dev  
$ sudo apt-get install libcgal-demo 


## install GDAL 2.2.2
please refer to [here][2]
if you want to use kmz, GDAL should build with libkml supports [here][3].
GEOS is disable for GDAL/OGR by default. If you want to use topologic functions, you should build GEOS and enable GEOS in makefile.opt. a detailed reference can be found [here][4] 



[1]: https://www.learnopencv.com/install-opencv3-on-ubuntu/
[2]: http://blog.csdn.net/zdc_nj/article/details/51373338
[3]: https://trac.osgeo.org/gdal/wiki/LibKML
[4]: http://www.cnblogs.com/denny402/p/4968685.html