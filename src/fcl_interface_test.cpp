#include <robot_collision_checking/fcl_interface.h>



int main ( int argc, char **argv ) {
    std::cout<<"hello"<<std::endl;
    ros::init ( argc, argv, "robust" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ROS_INFO ( " RUNNING TEST FCL" );

    ROS_INFO ( "SIMPLE FCL INTERFACE" );
    std::cout<<"---------------------------------------------"<<std::endl;
    Eigen::Vector3d e_wps1 ( 0.0,0.0,0.0 ),e_wps2 ( -1.3,2.0,0.3 ),e_wps3 ( 2.0,0.5,0.0 ),e_wps4 ( 3.0,0.0,-0.25 ),e_wps5 ( 2.21,-1.23,0.21 );
    Eigen::Matrix3d e_wRs1,e_wRs2,e_wRs3,e_wRs4,e_wRs5;
    Eigen::Affine3d e_wTs1,e_wTs2,e_wTs3,e_wTs4,e_wTs5;//(wRs1,wps1);

    e_wRs1.setIdentity();
    e_wRs2.setIdentity();
    e_wRs3.setIdentity();
    e_wRs4.setIdentity();
    e_wRs5.setIdentity();
    Eigen::Quaterniond q ( 0.5,0.5,0.23,0.43 );
    q.normalize();
    Eigen::Quaterniond q2 ( -0.5,0.5,-1.23,0.43 );
    q2.normalize();
    e_wTs1.linear() =e_wRs1;
    e_wTs1.translation() =e_wps1;
    e_wTs2.linear() =e_wRs2;
    e_wTs2.translation() =e_wps2;
    e_wTs3.linear() =e_wRs3;
    e_wTs3.translation() =e_wps3;
    e_wTs4.linear() =q.toRotationMatrix();
    e_wTs4.translation() =e_wps4;
    e_wTs5.linear() =q2.toRotationMatrix();
    e_wTs5.translation() =e_wps5;

    fcl::Transform3d wTs1,wTs2,wTs3,wTs4,wTs5;//(wRs1,wps1);


    FCLInterface::transform2fcl ( e_wTs1,wTs1 );
    FCLInterface::transform2fcl ( e_wTs2,wTs2 );
    FCLInterface::transform2fcl ( e_wTs3,wTs3 );
    FCLInterface::transform2fcl ( e_wTs4,wTs4 );
    FCLInterface::transform2fcl ( e_wTs5,wTs5 );

    std::shared_ptr<fcl::CollisionGeometryd> cg_1= std::make_shared< fcl::Sphered> ( 0.3 ) ;
    std::shared_ptr<fcl::CollisionGeometryd> cg_2= std::make_shared< fcl::Sphered> ( 0.75 ) ;
    std::shared_ptr<fcl::CollisionGeometryd> cg_3= std::make_shared< fcl::Boxd> ( 0.2,0.2,0.2 ) ;
    std::shared_ptr<fcl::CollisionGeometryd> cg_4=  std::make_shared< fcl::Cylinderd> ( 0.1,1.0 ) ;
    std::shared_ptr<fcl::CollisionGeometryd> cg_5=  std::make_shared< fcl::Boxd> ( 0.25,0.5,0.2 ) ;
    std::shared_ptr<fcl::CollisionGeometryd> cg_6=std::make_shared< fcl::Sphered  > ( 0.3 );
    //     std::shared_ptr<fcl::CollisionGeometryd> cg_5= ( new fcl::Boxd ( 0.25,0.5,0.2 ) );


    fcl::CollisionObjectd *o1=new fcl::CollisionObjectd ( cg_1,wTs1 );
    fcl::CollisionObjectd *o2=new fcl::CollisionObjectd ( cg_2,wTs2 );


    fcl::CollisionRequestd request;
    fcl::DistanceRequestd dist_req;
    fcl::detail::GJKSolver_libccd< double> solver1;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    dist_req.enable_nearest_points=true;

//     // result will be returned via the collision result structure
    fcl::CollisionResultd result;
    fcl::DistanceResultd dist_result,dist_result5;
//     // perform distance test
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();

    fcl::distance ( o1, o2, dist_req, dist_result );

    std::cout<<"o1 o2 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;

    delete o2;
    fcl::CollisionObjectd *o3=new fcl::CollisionObjectd ( cg_3,wTs3 );
    fcl::distance ( o1, o3, dist_req, dist_result );
//
    std::cout<<"o1 Box1 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;


    delete o3;
    fcl::CollisionObjectd *o4=new fcl::CollisionObjectd ( cg_4,wTs4 );

    fcl::distance ( o1, o4, dist_req, dist_result );
//
    std::cout<<"o1 Cylinder1 Min distance "<<dist_result.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result.nearest_points[1]<<std::endl;

    delete o4;
    fcl::CollisionObjectd *o5=new fcl::CollisionObjectd ( cg_5,wTs5 );
    fcl::distance ( o1, o5, dist_req, dist_result5 );

    std::cout<<"o1 Box2 Min distance "<<dist_result5.min_distance<<std::endl;
    std::cout<<"Nearest point 1   = "<<dist_result5.nearest_points[0]<<std::endl;
    std::cout<<"Nearest point 2   = "<<dist_result5.nearest_points[1]<<std::endl;

    delete o1,o2,o3,o4,o5;
    ROS_INFO ( "Static functions" );
    std::cout<<"---------------------------------------------"<<std::endl;
    shape_msgs::SolidPrimitive sphere1,sphere2,cylinder1,box1,box2;

    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.3;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;
    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.75;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;
    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;
    box2.type=shape_msgs::SolidPrimitive::BOX;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;
    box2.dimensions.resize ( 3 );
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.25;
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.5;
    box2.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;
    cylinder1.dimensions.resize ( 2 );
    cylinder1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]=1.0;
    cylinder1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS]=0.1;
    cylinder1.type=shape_msgs::SolidPrimitive::CYLINDER;



    Eigen::Vector3d p1,p2;
    double distance;
    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,sphere2,e_wTs2,p1,p2 );
    std::cout<<" sphere1 & sphere2  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,box1,e_wTs3,p1,p2 );
    std::cout<<" sphere1 & box1  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,cylinder1,e_wTs4,p1,p2 );
    std::cout<<" sphere1 & cylinder1  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    distance=FCLInterface::checkDistanceObjects ( sphere1,e_wTs1,box2,e_wTs5,p1,p2 );
    std::cout<<" sphere1 & box2  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;

    ROS_INFO ( "DYNAMIC functions" );
    std::cout<<"---------------------------------------------"<<std::endl;

    FCLInterface test_node ( nh );
    std::cout<<" ===================== "<<std::endl;
    test_node.addCollisionObject ( sphere2,e_wTs2,0 );
    test_node.addCollisionObject ( box1,e_wTs3,1 );
    test_node.addCollisionObject ( cylinder1,e_wTs4,2 );
    test_node.addCollisionObject ( box2,e_wTs5,3 );
    test_node.displayObjects();
    std::vector<double> obj_distances;
    std::vector<int> obj_ids;
    std::vector<Eigen::Vector3d> p1w,p2w;
    test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );

    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
        std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
        std::cout<<"Sphere distance  "<<obj_distances[i]<<std::endl;
        std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
        std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
        test_node.publishPoint ( p1w[i],"closest_point1",i,"world", {1.0,0.0,1.0}, {0.1,0.1,0.1} );
        test_node.publishPoint ( p2w[i],"closest_point2",i,"world", {1.0,0.0,0.0}, {0.1,0.1,0.1} );
    }

    test_node.publishPoint ( e_wTs1.translation(),"object",100,"world", {1.0,0.0,0.0}, {0.6,0.6,0.6} );



    FCLObjectSet objects;
    objects.resize ( 5 );
    objects[0].object_shape=sphere1;
    objects[0].object_transform=e_wTs1;
    objects[1].object_shape=sphere2;
    objects[1].object_transform=e_wTs2;
    objects[2].object_shape=box1;
    objects[2].object_transform=e_wTs3;
    objects[3].object_shape=cylinder1;
    objects[3].object_transform=e_wTs4;
    objects[4].object_shape=box2;
    objects[4].object_transform=e_wTs5;

    double distance_objs;
    distance_objs=FCLInterface::checkDistanceObjects ( objects[0],objects[1],p1,p2 );
    std::cout<<" sphere1 & sphere2  distance = "<<distance<<std::endl;
    std::cout<<" Closest Points p1 = ["<<p1 ( 0 ) <<", "<<p1 ( 1 ) <<", "<<p1 ( 2 ) <<"]"<<std::endl;
    std::cout<<"                p2 = ["<<p2 ( 0 ) <<", "<<p2 ( 1 ) <<", "<<p2 ( 2 ) <<"]"<<std::endl;



    return 1;
}

