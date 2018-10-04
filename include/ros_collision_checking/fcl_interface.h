#include <ros/ros.h>
#include <ros/package.h>
#include <Eigen/Core>
#include <Eigen/Eigen>/*
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>*/
#include <fcl/fcl.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shape_to_marker.h>

#ifndef FCL_INTERFACE_HPP
#define FCL_INTERFACE_HPP


typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;

int PLANE=10;
struct FCLInterfaceCollisionObject {
    FCLCollisionObjectPtr collision_object;
    unsigned int collision_id;
    unsigned int object_type;
    shape_msgs::Plane plane;
    shape_msgs::Mesh mesh;
    shape_msgs::SolidPrimitive solid;
    ~FCLInterfaceCollisionObject() {
        std::cout<<"Destroying object  "<<collision_id<<std::endl;
    };
};


class FCLInterface
{
private:
    ros::NodeHandle nh_;
    std::vector<std::unique_ptr<FCLInterfaceCollisionObject>> fcl_collision_world;
    // Their respective ids
    unsigned int obj_counter;
    ros::Publisher mkr_pub; //  rviz visualization

public:
    FCLInterface ( ros::NodeHandle nh );
    ~FCLInterface();
    bool addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
    bool addCollisionObject ( const shape_msgs::Plane  & s1,
                              const  Eigen::Affine3d  & wT1, unsigned int object_id );
    bool addCollisionObject ( const shape_msgs::Mesh  & s1 ,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
    bool removeCollisionObject ( unsigned int object_id );

    bool displayMarker ( shape_msgs::SolidPrimitive s1, const Eigen::Affine3d & T,
                         unsigned int obj_id,Eigen::Vector4d color );
    bool displayObjects ( std::vector<unsigned int> object_ids );
    bool displayObjects();
    void publishPoint ( Eigen::Vector3d pose,
                        std::string mkr_namespace,
                        unsigned int id,
                        std::string frame,
                        std::vector<double> color,
                        std::vector<double> scale
                      );
    // Check the collision between a primitives and the known world
    // Return true if in collision
    bool checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                     const  Eigen::Affine3d  & wT1
                                   );
    // Check the collision between a primitives and the known world
    // Return true if in collision, also returns a list of colliding objects
    bool checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                     const  Eigen::Affine3d  & wT1,
                                     std::vector<unsigned int> & id_collision_objects
                                   );
    // Gets the distance from the object( shape s1 location wT1) and the collision world
    // Returns:
    //    1. objs_distance a vector of distances (len = nbr of objects)
    //    2. wP1 a vector of points (len = nbr of objects) closest pt on primitives to objects
    //    3. wPobjs a vector of points (len = nbr of objects) closest pt on objects to primitives
    bool checkDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                    const  Eigen::Affine3d  & wT1,
                                    std::vector<int> & id_collision_objects,
                                    std::vector<double> & objs_distance,
                                    std::vector<Eigen::Vector3d> & wP1,
                                    std::vector<Eigen::Vector3d> & wPobjs
                                  );


    // Static functions
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::SolidPrimitive & s1 );
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::Plane  & s1 );
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::Mesh  & s1 );

    // Returns the distance between two solid primitives
    static double checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::SolidPrimitive  &  s2,
                                         const Eigen::Affine3d  & wT2
                                       );
    // Returns the distance between two solid primitives
    // Also returns the position w.r.t to world frame of the closest points
    static double checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::SolidPrimitive  &  s2,
                                         const Eigen::Affine3d  & wT2,
                                         Eigen::Vector3d & wP1,
                                         Eigen::Vector3d & wP2
                                       );
    // Returns true if two solid primitives are in collision
    static bool checkCollisionObjects ( const shape_msgs::SolidPrimitive  & s1,
                                        const  Eigen::Affine3d  & wT1,
                                        const shape_msgs::SolidPrimitive  & s2,
                                        const Eigen::Affine3d  & wT2
                                      );


    static void convertGeometryPoseEigenTransform ( const geometry_msgs::Pose & geo_pose,
            Eigen::Affine3d& wTt );
    static void convertEigenTransformGeometryPose ( const  Eigen::Affine3d & wTt,
            geometry_msgs::Pose& geo_pose );

    static void convertGeometryPointEigenVector ( const geometry_msgs::Point & geo_pose,
            Eigen::Vector3d& wTt );
    static void convertEigenVectorGeometryPoint ( const  Eigen::Vector3d& wTt,
            geometry_msgs::Point& geo_pose );

    // These function is taken from moveit core
    static void transform2fcl ( const Eigen::Affine3d& b, fcl::Transform3d& f );
    /** \brief Construct the marker that corresponds to the shape. Return false on failure. */

};
#endif
