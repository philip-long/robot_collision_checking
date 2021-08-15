///  @file fcl_interface.h
/// \class FCLInterface
/// \brief A simple ros interface for fcl
///
/// A simple class that allows collision checking using FCL in ros. Solid primitives
/// and mesh objects are supported. Collision worlds can be constructed, can return
/// collission as a boolean but also minimum distance and points of minimum distance
/// on both objects.
///
///
/// \author Philip Long <philip.long01@gmail.com>, RiVER Lab Northeastern University
/// \date Mar, 2019


#include <ros/ros.h>
#include <Eigen/Eigen>
#include <fcl/fcl.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shape_to_marker.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>

#ifndef FCL_INTERFACE_HPP
#define FCL_INTERFACE_HPP


typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;



#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

const int PLANE=10;
const int OCTOMAP_INT = 20;

#endif

/// A collision object to interface with ros
struct FCLInterfaceCollisionObject {
    FCLCollisionObjectPtr collision_object;
    unsigned int collision_id;
    unsigned int object_type;
    shape_msgs::Plane plane;
    shape_msgs::Mesh mesh;
    shape_msgs::SolidPrimitive solid;
    ~FCLInterfaceCollisionObject() {
        // std::cout<<"Destroying object  "<<collision_id<<std::endl;
    };
};

/// FCL Object is simply a container for the object information
struct FCLObject {
    FCLObject ( const shape_msgs::SolidPrimitive & shape,
                const Eigen::Affine3d  & transform
              ) : object_shape ( shape ),object_transform ( transform ) {};
    FCLObject () {};
    shape_msgs::SolidPrimitive  object_shape;
    Eigen::Affine3d  object_transform;
};

/// Vector of FCL Objects
typedef std::vector<FCLObject> FCLObjectSet;

class FCLInterface
{
private:
    ros::NodeHandle nh_;
    std::vector<std::unique_ptr<FCLInterfaceCollisionObject>> fcl_collision_world;
    // Their respective ids
    unsigned int obj_counter;
    ros::Publisher mkr_pub; //  rviz visualization

    /// Check the minimum distance between current world and ptr
    double checkMinimumDistanceObjectWorld(FCLCollisionObjectPtr o1);
    
    /// Static function that checks the minimum distance between two collision objects
    static double checkDistanceObjects ( FCLCollisionObjectPtr o1,
        FCLCollisionObjectPtr o2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2 );
    
    /// Check the distances between object and all world objects
    bool checkDistanceObjectWorld ( FCLCollisionObjectPtr o1,
          std::vector<int> & id_collision_objects,
        std::vector<double> & objs_distance,
        std::vector<Eigen::Vector3d> & wP1,
        std::vector<Eigen::Vector3d> & wPobjs);
        
public:
    FCLInterface ( ros::NodeHandle nh );
    ~FCLInterface();

    /// Add a set of collision objects to the world
    bool addCollisionObject ( FCLObjectSet & objects );
    /// Add a FCL collision objects with a defined id to the world
    bool addCollisionObject ( FCLObject & object,
                              unsigned int object_id );
    /// Add a collision objects defined by a ROS SolidPrimitive msgs with a defined id to the world
    bool addCollisionObject ( const shape_msgs::SolidPrimitive & s1,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
    /// Add a collision objects defined by a ROS plane msgs with a defined id to the world (Untested)
    bool addCollisionObject ( const shape_msgs::Plane  & s1,
                              const  Eigen::Affine3d  & wT1, unsigned int object_id );
    /// Add a collision objects defined by a ROS mesh msgs with a defined id to the world
    bool addCollisionObject ( const shape_msgs::Mesh  & s1 ,
                              const  Eigen::Affine3d  & wT1,unsigned int object_id );
    // Add an octomap to the world
    bool addCollisionObject(const octomap_msgs::Octomap &map,
                            const  Eigen::Affine3d  & wT1,
                            unsigned int object_id);
    /// Delete a collision object with object id
    
    
    
    

    bool removeCollisionObject ( unsigned int object_id );
    /// Display a marker defined by its ROS msgs in rviz
    bool displayMarker ( shape_msgs::SolidPrimitive s1, const Eigen::Affine3d & T,
                         unsigned int obj_id,Eigen::Vector4d color );
    /// Display all  objects 
    bool displayObjects(std::string frame_name="world");
    /// Publish a point in RVIZ
    void publishPoint ( Eigen::Vector3d pose,
                        std::string mkr_namespace,
                        unsigned int id,
                        std::string frame,
                        std::vector<double> color,
                        std::vector<double> scale
                      );

    /// Check the collision between a primitives and the known world. Return true if in collision
    bool checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                     const  Eigen::Affine3d  & wT1
                                   );
    /// Check the collision between a mesh and the known world. Return true if in collision
    bool checkCollisionObjectWorld ( const shape_msgs::Mesh  & s1,
                                     const  Eigen::Affine3d  & wT1
                                   );
    // Check the collision between a primitives and the known world. Return true if in collision, also returns a list of colliding objects
    bool checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                     const  Eigen::Affine3d  & wT1,
                                     std::vector<unsigned int> & id_collision_objects
                                   );
    /** Gets the distance from the object( shape s1 location wT1) and the collision world
    * Returns:
    *    1. objs_distance a vector of distances (len = nbr of objects)
    *    2. id_collision_objects an ordered vector of collision object ids
    *    3. wP1 a vector of points (len = nbr of objects) closest pt on primitives to objects
    *    4. wPobjs a vector of points (len = nbr of objects) closest pt on objects to primitives*/
    bool checkDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
                                    const  Eigen::Affine3d  & wT1,
                                    std::vector<int> & id_collision_objects,
                                    std::vector<double> & objs_distance,
                                    std::vector<Eigen::Vector3d> & wP1,
                                    std::vector<Eigen::Vector3d> & wPobjs
                                  );
    
    /** Gets the distance from the object( shape s1 location wT1) and the collision world
    * Returns:
    *    1. objs_distance a vector of distances (len = nbr of objects)
    *    2. id_collision_objects an ordered vector of collision object ids
    *    3. wP1 a vector of points (len = nbr of objects) closest pt on primitives to objects
    *    4. wPobjs a vector of points (len = nbr of objects) closest pt on objects to primitives*/
    bool checkDistanceObjectWorld ( const shape_msgs::Mesh  & s1,
                                    const  Eigen::Affine3d  & wT1,
                                    std::vector<int> & id_collision_objects,
                                    std::vector<double> & objs_distance,
                                    std::vector<Eigen::Vector3d> & wP1,
                                    std::vector<Eigen::Vector3d> & wPobjs
                                  );
    
    
    /// Get the minimum distance between a solid primitive and the world
    double checkMinimumDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1 );
    double checkMinimumDistanceObjectWorld ( const shape_msgs::Mesh  & s1,
        const  Eigen::Affine3d  & wT1 );
    
    
     
    
    
    
    /// Create a collision fcl geometry from a ros msgs
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::SolidPrimitive & s1 );
    /// Create a collision fcl geometry from a ros msgs
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::Plane  & s1 );
    /// Create a collision fcl geometry from a ros msgs
    static  FCLCollisionGeometryPtr createCollisionGeometry ( const shape_msgs::Mesh  & s1 );
    /// Create a collision fcl geometry from a octree
    static FCLCollisionGeometryPtr createCollisionGeometry(const octomap_msgs::Octomap &map);
    /// Create a collision fcl geometry from a octree
    static FCLCollisionGeometryPtr createCollisionGeometry( const std::shared_ptr<const octomap::OcTree>& tree);
    
    
    
    // Returns the distance between two solid primitives
    static double checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::SolidPrimitive  &  s2,
                                         const Eigen::Affine3d  & wT2
                                       );
    /** Returns the distance between two solid primitives
    * Also returns the position w.r.t to world frame of the closest points */
    static double checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::SolidPrimitive  &  s2,
                                         const Eigen::Affine3d  & wT2,
                                         Eigen::Vector3d & wP1,
                                         Eigen::Vector3d & wP2
                                       );
    
        /** Returns the distance between two solid primitives
    * Also returns the position w.r.t to world frame of the closest points */
    static double checkDistanceObjects ( const shape_msgs::Mesh & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::Mesh  &  s2,
                                         const Eigen::Affine3d  & wT2,
                                         Eigen::Vector3d & wP1,
                                         Eigen::Vector3d & wP2
                                       );
        /** Returns the distance between two solid primitives
    * Also returns the position w.r.t to world frame of the closest points */
    static double checkDistanceObjects ( const shape_msgs::Mesh & s1,
                                         const  Eigen::Affine3d  & wT1,
                                         const shape_msgs::SolidPrimitive  &  s2,
                                         const Eigen::Affine3d  & wT2,
                                         Eigen::Vector3d & wP1,
                                         Eigen::Vector3d & wP2
                                       );
    
    
    
    /** Returns the distance between two solid primitives
    * Also returns the position w.r.t to world frame of the closest points */
    static double checkDistanceObjects ( const  FCLObject & object1,
                                         const  FCLObject & object2,
                                         Eigen::Vector3d & closest_pt_object1,
                                         Eigen::Vector3d & closest_pt_object2
                                       );    
    
    /** Gets the distance from the FCLObject and the FCLObjectSet
    * Returns:
    *    1. objs_distance a vector of distances (len = nbr of objects)
    *    2. closest_pt_robot a vector of points (len = nbr of objects) closest pt on FCLObject to FCLObjectSet
    *    3. closest_pt_objects a vector of points (len = nbr of objects) closest pt on FCLObjectSet to FCLObject */
    static   double checkDistanceObjectWorld ( FCLObject link,
            FCLObjectSet object_world,
            std::vector<double> & objs_distance,
            std::vector<Eigen::Vector3d> & closest_pt_robot,
            std::vector<Eigen::Vector3d> & closest_pt_objects
                                             );
    
    /** Gets the distance from the object( shape s1 location wT1) and the FCLObjectSet
    * Returns:
    *    1. objs_distance a vector of distances (len = nbr of objects)
    *    2. closest_pt_robot a vector of points (len = nbr of objects) closest pt on FCLObject to FCLObjectSet
    *    3. closest_pt_objects a vector of points (len = nbr of objects) closest pt on FCLObjectSet to FCLObject */
    static   double checkDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & s1,
            const  Eigen::Affine3d  & wT1,
            FCLObjectSet object_world,
            std::vector<double> & objs_distance,
            std::vector<Eigen::Vector3d> & closest_pt_robot,
            std::vector<Eigen::Vector3d> & closest_pt_objects
                                             );
 /** Gets the distance from the object( mesh s1 location wT1) and the FCLObjectSet
    * Returns:
    *    1. objs_distance a vector of distances (len = nbr of objects)
    *    2. closest_pt_robot a vector of points (len = nbr of objects) closest pt on FCLObject to FCLObjectSet
    *    3. closest_pt_objects a vector of points (len = nbr of objects) closest pt on FCLObjectSet to FCLObject */
   static   double checkDistanceObjectWorld ( const shape_msgs::Mesh  & s1,
            const  Eigen::Affine3d  & wT1,
            FCLObjectSet object_world,
            std::vector<double> & objs_distance,
            std::vector<Eigen::Vector3d> & closest_pt_robot,
            std::vector<Eigen::Vector3d> & closest_pt_objects
                                             );
    
    /// Check collision between a ros shape msgs with transform and FCLObjectSet. Returns true if there is a collision
    static bool checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & shape,
                                            const  Eigen::Affine3d  & transform, FCLObjectSet object_world );

    /// Returns true if two solid primitives are in collision
    static bool checkCollisionObjects ( const shape_msgs::SolidPrimitive  & s1,
                                        const  Eigen::Affine3d  & wT1,
                                        const shape_msgs::SolidPrimitive  & s2,
                                        const Eigen::Affine3d  & wT2
                                      );
    /// Returns true if two FCLObjects are in collision
    static bool checkCollisionObjects ( const FCLObject & object1,
                                        const FCLObject & object2
                                      );

    static void convertGeometryPoseEigenTransform ( const geometry_msgs::Pose & geo_pose,
            Eigen::Affine3d& wTt );
    static void convertEigenTransformGeometryPose ( const  Eigen::Affine3d & wTt,
            geometry_msgs::Pose& geo_pose );

    static void convertGeometryPointEigenVector ( const geometry_msgs::Point & geo_pose,
            Eigen::Vector3d& wTt );
    static void convertEigenVectorGeometryPoint ( const  Eigen::Vector3d& wTt,
            geometry_msgs::Point& geo_pose );

    // This function is taken from moveit core
    static void transform2fcl ( const Eigen::Affine3d& b, fcl::Transform3d& f );
};
#endif
