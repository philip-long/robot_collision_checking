#include <robot_collision_checking/fcl_interface.h>


octomap_msgs::Octomap octomap_;

bool octomap_received ( false ),lin_callback_received(false);

void octoCallback ( const octomap_msgs::Octomap::ConstPtr& msg ) {
    octomap_=*msg;
    octomap_received=true;
    std::cout<<"received callback"<<std::endl;
}



int main ( int argc, char **argv ) {
    std::cout<<"hello"<<std::endl;
    ros::init ( argc, argv, "octomap" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    
    
    ros::Subscriber  octo_sub= nh.subscribe ( "/octomap_full",
                                1, &octoCallback );
    
    ROS_INFO ( " RUNNING TEST FCL" );

    ROS_INFO ( "SIMPLE FCL INTERFACE" );
    std::cout<<"---------------------------------------------"<<std::endl;
    Eigen::Vector3d e_wps1 ( 0.0,0.0,10.0 ),e_wps2 (1.0,0.0,1.0 ),e_wps3 ( 2.0,0.5,0.0 ),e_wps4 ( 3.0,0.0,-0.25 ),e_wps5 ( 2.21,-1.23,0.21 );
    Eigen::Matrix3d e_wRs1,e_wRs2,e_wRs3,e_wRs4,e_wRs5;
    Eigen::Affine3d e_wTs1,e_wTs2,e_wTs3,e_wTs4,e_wTs5;//(wRs1,wps1);

    
    
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


    ROS_INFO ( " Creating shapes TEST FCL" );
    shape_msgs::SolidPrimitive sphere1,sphere2,cylinder1,box1,box2;

    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.1;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;
    
    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.75;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;
    
    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;    
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;
    

    FCLInterface test_node ( nh );
    std::cout<<" ===================== "<<std::endl;
    ros::Duration(0.2).sleep();
    
    ROS_INFO ( " TEST FCL" );
    test_node.addCollisionObject ( sphere2,e_wTs2,0 );
    test_node.addCollisionObject ( box1,e_wTs3,1 );    
    test_node.displayObjects();
    
        

    std::vector<double> obj_distances;
    std::vector<int> obj_ids;
    std::vector<Eigen::Vector3d> p1w,p2w;
    
    if(octomap_received)
    {
        Eigen::Vector3d e_octo ( 0.0,0.0,0.0);
        Eigen::Matrix3d e_wRsocto;
        Eigen::Affine3d e_wTsocto;
        fcl::Transform3d e_wTsocto_fcl;
        FCLInterface::transform2fcl (e_wTsocto,e_wTsocto_fcl );
        
        std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree*>(octomap_msgs::msgToMap(octomap_)));
        
        std::shared_ptr<fcl::CollisionGeometryd> fcl_octomap(new fcl::OcTreed (om));
        fcl::CollisionObjectd *o1=new fcl::CollisionObjectd(fcl_octomap,e_wTsocto_fcl);
        fcl::CollisionRequestd request;
        fcl::DistanceRequestd dist_req;
        fcl::detail::GJKSolver_libccd< double> solver1;
        dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
        dist_req.enable_nearest_points=true;
    
//     // result will be returned via the collision result structure
        fcl::CollisionResultd result;
        fcl::DistanceResultd dist_result,dist_result5;
     

        
        e_wRsocto.setIdentity();
        e_wTsocto.translation() =e_octo;
        e_wTsocto.linear() =e_wRs1;        
        ros::Duration(0.2).sleep();  
        ROS_INFO ( " TEST OCTOMAP" );
        
        test_node.addCollisionObject ( octomap_,e_wTsocto,99 );
        std::vector<unsigned int> id_collision_objects;id_collision_objects.resize(0);
        test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );
        
    
        for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
        std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
        std::cout<<"Sphere distance  "<<obj_distances[i]<<std::endl;
        std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
        std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
        //test_node.publishPoint ( p1w[i],"closest_point1",i,"world", {1.0,0.0,1.0}, {0.1,0.1,0.1} );
        //test_node.publishPoint ( p2w[i],"closest_point2",i,"world", {1.0,0.0,0.0}, {0.1,0.1,0.1} );
    }

    //test_node.publishPoint ( e_wTs1.translation(),"object",100,"world", {1.0,0.0,0.0}, {0.6,0.6,0.6} );
    test_node.checkCollisionObjectWorld(sphere1,e_wTs1,id_collision_objects);
    
    for ( unsigned int i=0; i<id_collision_objects.size(); i++ ) {
            std::cout<<"Obj id "<<id_collision_objects[i]<<"in collision"<<std::endl;
        }
    }
    

    return 1;
}

