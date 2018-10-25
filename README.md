# ROS_COLLISION_CHECKING
A package to access FCL through a simple ros interface. 
This package requires 

 * fcl > 6.0 [FCL](http://www.ros.org/wiki/fcl) 
 * [libccd](https://github.com/danfis/libccd) 
 
## Installing FCL & CCD
The binaries FCL 5.0 should be already installed on ros kinetic, but we can't use them, likewise libccd-dev may already be installed but we need to reinstall from source.  Ensure this option is enabled, when compiling: 
>> -DENABLE_DOUBLE_PRECISION=ON

### LIBCCD
1. git clone clone https://github.com/danfis/libccd.git
2. mkdir build && cd build
3. cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
4. make
5. sudo make install

## FCL
1. git clone https://github.com/flexible-collision-library/fcl.git
2. mkdir build
3. cd build
4. cmake ..
5. make
6. sudo make install

### finding
The FindFCL.cmake should find the installed FCL (which overrides version 5.0).
If there are errors suchs as constants not found probably it's still using the 
older version of FCL. Lastly, none of the examples from FCL homepage will work as they now require a templating argument. 

### To do
1. Add interface for mesh type objects
2. Add interface for octree
3. Add server client