# Robot Collision Checking

A lightweight package to use FCL with ROS messages (heavily inspired by [MoveIt's version](https://moveit.ros.org/documentation/concepts/developer_concepts/) but with FCL 5.0). 

The package can be utilised for a couple of purposes:
1. To find distances and collisions between shape primitives, as described by ROS messages and Eigen poses.
2. To perform distance and collision checking via static functions, or by creating a class and maintaining a world. The world can be represented using shape primitives, [Octomaps](http://docs.ros.org/en/noetic/api/octomap_msgs/html/msg/Octomap.html) and [VoxelGrids](http://docs.ros.org/en/noetic/api/costmap_2d/html/msg/VoxelGrid.html).

This package requires:
 * [FCL](http://www.ros.org/wiki/fcl) > 6.0
 * [libccd](https://github.com/danfis/libccd) 
 * [geometric_shapes](https://github.com/ros-planning/geometric_shapes/tree/591b7a0708c9cc1e42b5cdbbc306e99913ecffa8) at commit **#591b7a0**; 
 
## Installation

The binaries FCL 5.0 and `geometric_shapes` should be already installed on your ROS distro, but we cannot use them directly. Likewise, `libccd-dev` may already be installed but we need to reinstall from source. The following instructions will enable you to build the `robot_collision_checking` package within a catkin workspace using `catkin_make` or `catkin build`.

### libccd

Please ensure this option is enabled, when compiling: 
>> -DENABLE_DOUBLE_PRECISION=ON

1. git clone https://github.com/danfis/libccd.git
2. mkdir build && cd build
3. cmake -G "Unix Makefiles" -DENABLE_DOUBLE_PRECISION=ON ..
4. make
5. sudo make install

### FCL
1. git clone https://github.com/flexible-collision-library/fcl.git
2. mkdir build
3. cd build
4. cmake ..
5. make
6. sudo make install

The FindFCL.cmake should find the installed FCL (which overrides version 5.0).
If there are errors suchs as constants not found probably it's still using the 
older version of FCL. Lastly, none of the examples from FCL homepage will work as they now require a templating argument. 

### geometric_shapes

Please note that the reason for reverting to the #591b7a0 version of [geometric_shapes](https://github.com/ros-planning/geometric_shapes/tree/591b7a0708c9cc1e42b5cdbbc306e99913ecffa8) is due to the QHull error documented in [this issue thread](https://github.com/ros-planning/moveit_task_constructor/issues/241#issuecomment-793539263). Unfortunately, this may incur segfault errors, however our tests did not encounter these.

To correctly run `robot_collision_checking` with the capability of checking distances and collisions against Octomap representations, please include [geometric_shapes](https://github.com/ros-planning/geometric_shapes/tree/591b7a0708c9cc1e42b5cdbbc306e99913ecffa8) locally in your catkin workspace and build from source.

## Example using ROS shape_msgs
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