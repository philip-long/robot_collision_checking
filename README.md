# ROS_COLLISION_CHECKING
A lightweight package to use FCL with a ros message (heavily inspired by moveit's but with FCL 5.0). 
1. This package is for finding distances and collision between shape primitives described by ros_messages and eigen poses
2. Collision checking & distance checking can be done by static functions or creating a class and maintaining a world


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

### Installation
1. This package compiles with catkin build

### To do
1. Add interface for mesh type objects
2. Add interface for octree
3. Add server client

### Example using ros shape_msgs
```
    shape_msgs::SolidPrimitive sphere1,box1;
    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.3;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;

    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;

    Eigen::Affine3d e_wTs1,e_wTs2;
    Eigen::Quaterniond q ( 0.5,0.5,0.23,0.43 );    q.normalize();
    Eigen::Quaterniond q2 ( -0.5,0.5,-1.23,0.43 );     q2.normalize();
    Eigen::Vector3d e_wps1 ( 0.0,0.0,0.0 ),e_wps2 ( -1.3,2.0,0.3 );

    e_wTs1.linear() =q.toRotationMatrix();;
    e_wTs1.translation() =e_wps1;
    e_wTs2.linear() =q2.toRotationMatrix();;
    e_wTs2.translation() =e_wps2;

    Eigen::Vector3d p1,p2;
    double distance;
    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,box1,e_wTs2,p1,p2 );
    bool in_collision=FCLInterface::checkCollisionObjects( sphere1,e_wTs1,box1,e_wTs2);
    std::cout<<" sphere1 & box1  in collision = "<<in_collision<<std::endl;
    std::cout<<" sphere1 & box1  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points on sphere p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                on box    p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;
```














