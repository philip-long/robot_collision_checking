#include <robot_collision_checking/fcl_interface.h>

// This file includes all static functions for fcl_interface


double FCLInterface::checkDistanceObjects ( FCLCollisionObjectPtr o1,
        FCLCollisionObjectPtr o2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2 ) {

    fcl::DistanceRequestd dist_req;
    dist_req.enable_nearest_points=true;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    fcl::DistanceResultd dist_result;

    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();
    fcl::distance ( o1.get(), o2.get(), dist_req, dist_result );
    for ( int i=0; i<3; i++ ) {
        wP1 ( i ) =dist_result.nearest_points[0][i];
        wP2 ( i ) =dist_result.nearest_points[1][i];
    }
    return dist_result.min_distance;

}

double FCLInterface::checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  &  s2,
        const Eigen::Affine3d  & wT2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    FCLCollisionObjectPtr o1=std::make_shared<fcl::CollisionObjectd> ( cg_1,wTf1 );
    FCLCollisionObjectPtr o2=std::make_shared<fcl::CollisionObjectd> ( cg_2,wTf2 );
    
     Eigen::Vector3d  wP1,wP2;
    
    double dist=checkDistanceObjects ( o1,o2,wP1,wP2 );
    return dist;
}
double FCLInterface::checkDistanceObjects ( const shape_msgs::SolidPrimitive & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  &  s2,
        const Eigen::Affine3d  & wT2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    FCLCollisionObjectPtr o1=std::make_shared<fcl::CollisionObjectd> ( cg_1,wTf1 );
    FCLCollisionObjectPtr o2=std::make_shared<fcl::CollisionObjectd> ( cg_2,wTf2 );
    double dist=checkDistanceObjects ( o1,o2,wP1,wP2 );
    return dist;
}

double FCLInterface::checkDistanceObjects ( const shape_msgs::Mesh & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::Mesh  &  s2,
        const Eigen::Affine3d  & wT2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    FCLCollisionObjectPtr o1=std::make_shared<fcl::CollisionObjectd> ( cg_1,wTf1 );
    FCLCollisionObjectPtr o2=std::make_shared<fcl::CollisionObjectd> ( cg_2,wTf2 );
    double dist=checkDistanceObjects ( o1,o2,wP1,wP2 );
    return dist;
}


double FCLInterface::checkDistanceObjects ( const shape_msgs::Mesh & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  &  s2,
        const Eigen::Affine3d  & wT2,
        Eigen::Vector3d & wP1,
        Eigen::Vector3d & wP2
                                          ) {
    fcl::Transform3d wTf1,wTf2;
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    FCLCollisionObjectPtr o1=std::make_shared<fcl::CollisionObjectd> ( cg_1,wTf1 );
    FCLCollisionObjectPtr o2=std::make_shared<fcl::CollisionObjectd> ( cg_2,wTf2 );
    double dist=checkDistanceObjects ( o1,o2,wP1,wP2 );
    return dist;
}

double FCLInterface::checkDistanceObjectWorld ( FCLObject link,
        FCLObjectSet object_world,
        std::vector<double> & objs_distance,
        std::vector<Eigen::Vector3d> & closest_pt_robot,
        std::vector<Eigen::Vector3d> & closest_pt_objects
                                              ) {
    closest_pt_robot.clear();
    closest_pt_robot.resize ( object_world.size() );
    closest_pt_objects.clear();
    closest_pt_objects.resize ( object_world.size() );
    objs_distance.clear();
    objs_distance.resize ( object_world.size() );


    for ( unsigned int i=0; i<object_world.size(); i++ ) {
        objs_distance[i]=checkDistanceObjects ( link.object_shape,
                                                link.object_transform,
                                                object_world[i].object_shape,
                                                object_world[i].object_transform,
                                                closest_pt_robot[i],
                                                closest_pt_objects[i] );
    }

}



double FCLInterface::checkDistanceObjectWorld ( const shape_msgs::SolidPrimitive  & shape,
        const  Eigen::Affine3d  & transform,
        FCLObjectSet object_world,
        std::vector<double> & objs_distance,
        std::vector<Eigen::Vector3d> & closest_pt_robot,
        std::vector<Eigen::Vector3d> & closest_pt_objects
                                              ) {
    closest_pt_robot.clear();
    closest_pt_robot.resize ( object_world.size() );
    closest_pt_objects.clear();
    closest_pt_objects.resize ( object_world.size() );
    objs_distance.clear();
    objs_distance.resize ( object_world.size() );


    for ( unsigned int i=0; i<object_world.size(); i++ ) {
        objs_distance[i]=checkDistanceObjects ( shape,
                                                transform,
                                                object_world[i].object_shape,
                                                object_world[i].object_transform,
                                                closest_pt_robot[i],
                                                closest_pt_objects[i] );
    }
        return true;

}

double FCLInterface::checkDistanceObjectWorld ( const shape_msgs::Mesh  & shape,
        const  Eigen::Affine3d  & transform,
        FCLObjectSet object_world,
        std::vector<double> & objs_distance,
        std::vector<Eigen::Vector3d> & closest_pt_robot,
        std::vector<Eigen::Vector3d> & closest_pt_objects
                                              ) {
    closest_pt_robot.clear();
    closest_pt_robot.resize ( object_world.size() );
    closest_pt_objects.clear();
    closest_pt_objects.resize ( object_world.size() );
    objs_distance.clear();
    objs_distance.resize ( object_world.size() );


    for ( unsigned int i=0; i<object_world.size(); i++ ) {
        objs_distance[i]=checkDistanceObjects ( shape,
                                                transform,
                                                object_world[i].object_shape,
                                                object_world[i].object_transform,
                                                closest_pt_robot[i],
                                                closest_pt_objects[i] );
    }
    return true;

}

bool FCLInterface::checkCollisionObjectWorld ( const shape_msgs::SolidPrimitive  & shape,
        const  Eigen::Affine3d  & transform,
        FCLObjectSet object_world ) {

    for ( unsigned int i=0; i<object_world.size(); i++ ) {
        if ( checkCollisionObjects ( shape,
                                     transform,
                                     object_world[i].object_shape,
                                     object_world[i].object_transform )
           ) {
            return true; // In collision with ith object
        }
    }
    return false; // is not in collision with any object
}




double FCLInterface::checkDistanceObjects ( const  FCLObject & object1,
        const  FCLObject & object2,
        Eigen::Vector3d & closest_pt_object1,
        Eigen::Vector3d & closest_pt_object2
                                          ) {

    double obj_dist=checkDistanceObjects ( object1.object_shape,
                                           object1.object_transform,
                                           object2.object_shape,
                                           object2.object_transform,
                                           closest_pt_object1,
                                           closest_pt_object2 );
    return obj_dist;
}




bool FCLInterface::checkCollisionObjects ( const shape_msgs::SolidPrimitive  & s1,
        const  Eigen::Affine3d  & wT1,
        const shape_msgs::SolidPrimitive  & s2,
        const Eigen::Affine3d  & wT2
                                         ) {
    fcl::Transform3d wTf1,wTf2;
    FCLCollisionGeometryPtr cg_1=createCollisionGeometry ( s1 );
    FCLCollisionGeometryPtr cg_2=createCollisionGeometry ( s2 );
    transform2fcl ( wT1,wTf1 );
    transform2fcl ( wT2,wTf2 );
    //fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTf1 );
    //fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTf2 );
    FCLCollisionObjectPtr o1=std::make_shared<fcl::CollisionObjectd> ( cg_1,wTf1 );
    FCLCollisionObjectPtr o2=std::make_shared<fcl::CollisionObjectd> ( cg_2,wTf2 );
    fcl::CollisionRequestd col_req;
    fcl::CollisionResultd col_result;
    fcl::collide ( o1.get(), o2.get(), col_req, col_result );
//     delete o1;
//     delete o2;
    return col_result.isCollision();
}

bool FCLInterface::checkCollisionObjects ( const FCLObject & object1,
        const FCLObject & object2
                                         ) {
    return checkCollisionObjects ( object1.object_shape,
                                   object1.object_transform,
                                   object2.object_shape,
                                   object2.object_transform
                                 );
}


void FCLInterface::convertGeometryPoseEigenTransform ( const geometry_msgs::Pose & geo_pose,
        Eigen::Affine3d& wTt ) {
    Eigen::Vector3d t ( geo_pose.position.x, geo_pose.position.y,geo_pose.position.z );
    Eigen::Quaterniond q ( geo_pose.orientation.w,
                           geo_pose.orientation.x,
                           geo_pose.orientation.y,
                           geo_pose.orientation.z );
    wTt.translation() =t;
    wTt.linear() =q.toRotationMatrix();
}

void FCLInterface::convertEigenTransformGeometryPose ( const  Eigen::Affine3d & wTt,
        geometry_msgs::Pose& geo_pose ) {

    Eigen::Quaterniond q ( wTt.linear() );
    q.normalize();
    Eigen::Vector3d t ( wTt.translation() );

    geo_pose.position.x=t ( 0 );
    geo_pose.position.y=t ( 1 );
    geo_pose.position.z=t ( 2 );
    geo_pose.orientation.x=q.x();
    geo_pose.orientation.y=q.y();
    geo_pose.orientation.z=q.z();
    geo_pose.orientation.w=q.w();

}

void FCLInterface::convertGeometryPointEigenVector ( const geometry_msgs::Point & geo_pose,
        Eigen::Vector3d& t ) {
    t.setZero();
    t ( 0 ) =geo_pose.x;
    t ( 1 ) =geo_pose.y;
    t ( 2 ) =geo_pose.z;
}
void FCLInterface::convertEigenVectorGeometryPoint ( const  Eigen::Vector3d& t,
        geometry_msgs::Point& geo_pose ) {
    geo_pose.x=t ( 0 );
    geo_pose.y=t ( 1 );
    geo_pose.z=t ( 2 );
}

void FCLInterface::transform2fcl ( const Eigen::Affine3d& b, fcl::Transform3d& f ) {
    f.linear() =b.linear();
    f.translation() =b.translation();
}
