# PCL Utils 
This some helpful PointClouds functions to be used with PCL Library.

## Build:
* `mkdir build && cd build`
* `cmake ..`
* `sudo make install`

## How to use:
 The shared library is installed in `/usr/lib` with name `libpcl_utils.so`

 in your CMakeLists.txt link the library with your app like this

 `target_link_libraries(my_app libpcl_utils.so)`