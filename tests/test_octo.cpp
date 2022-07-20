#include <robot_collision_checking/fcl_interface.h>


octomap_msgs::Octomap octomap_;
bool octomap_received ( false ),lin_callback_received(false);

void octoCallback ( const octomap_msgs::Octomap::ConstPtr& msg ) {    
    if(octomap_received==false)
        octomap_=*msg;
    octomap_received=true;
    std::cout<<"received callback"<<std::endl;
}


int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "octomap_test_fcl" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ros::Subscriber  octo_sub= nh.subscribe ( "/octomap_full",
                               1, &octoCallback );


    std::cout<<"---------------------------------------------"<<std::endl;
    ROS_INFO ( "SIMPLE FCL INTERFACE" );
    std::cout<<"---------------------------------------------"<<std::endl;
    Eigen::Vector3d e_wps1 ( 0.5,0.5,0.0 );
    Eigen::Vector3d e_wps2 (0.5,0.0,1.0 );
    Eigen::Vector3d e_wps3 ( 2.0,0.5,0.0 );
    Eigen::Vector3d e_octo ( 0.0,0.0,0.0);
    Eigen::Matrix3d e_wRs1,e_wRs2,e_wRs3,e_wRsocto;
    Eigen::Affine3d e_wTs1,e_wTs2,e_wTs3,e_wTsocto;
    Eigen::Quaterniond quat(0.21,0.5,0.6,0.5);
    quat.normalize();
    e_wRs1.setIdentity();
    e_wRs2=quat.toRotationMatrix();
    //e_wRs2.setIdentity();
    e_wRs3.setIdentity();
    e_wRsocto.setIdentity();
    

    e_wTs1.linear() =e_wRs1;
    e_wTs1.translation() =e_wps1;
    e_wTs2.linear() =e_wRs2;
    e_wTs2.translation() =e_wps2;
    e_wTs3.linear() =e_wRs3;
    e_wTs3.translation() =e_wps3;
    e_wTsocto.linear() =e_wRsocto;
    e_wTsocto.translation() =e_octo;


    ROS_INFO ( " Creating shapes via interface" );
    shape_msgs::SolidPrimitive sphere1,sphere2,box1;

    sphere1.dimensions.resize ( 1 );
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.5;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;

    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.1;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;

    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.1;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.1;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.5;

     
    FCLInterface test_node ( nh );
    ROS_INFO ( "Waiting a few seconds for RVIZ to start up" );
    ros::Duration(2.0).sleep();    
    test_node.addCollisionObject ( sphere1,e_wTs1,0 );
    test_node.addCollisionObject ( box1,e_wTs2,1 );
    test_node.addCollisionObject ( sphere2,e_wTs3,2 );
    test_node.displayObjects();
    test_node.removeCollisionObject(0);
    test_node.removeCollisionObject(1);
    test_node.removeCollisionObject(2);

    ros::Duration(0.2).sleep();
    std::vector<double> obj_distances;
    std::vector<int> obj_ids;
    std::vector<Eigen::Vector3d> p1w,p2w;
    std::vector<unsigned int> id_collision_objects;
    id_collision_objects.resize(0);
    
    while(!octomap_received)
    {
       ROS_INFO ( "Waiting for Octomap" );
       ros::Duration(0.5).sleep(); 
       ros::spinOnce();
    }

    /* =========================================================== //

            Octomap pre-filtering

    // =========================================================== */
    test_node.addCollisionObject ( octomap_,e_wTsocto,99 );

    ROS_INFO ( "Checking Distance & Collision sphere octomap" );    
    std::cout<<"Is sphere in collision"<<test_node.checkCollisionObjectWorld(sphere1,e_wTs1)<<std::endl;
    std::cout<<"Is box in collision"<<test_node.checkCollisionObjectWorld(box1,e_wTs2)<<std::endl;
    std::cout<<"Is sphere2 in collision"<<test_node.checkCollisionObjectWorld(sphere2,e_wTs3)<<std::endl;


    test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }
    test_node.checkDistanceObjectWorld ( box1,e_wTs2,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }

    test_node.checkDistanceObjectWorld ( sphere2,e_wTs3,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }


    /* =========================================================== //

            Octomap filtering

    // =========================================================== */
    geometry_msgs::Pose sphere_pose,box_pose,sphere2_pose;
    tf::poseEigenToMsg(e_wTs1, sphere_pose);
    tf::poseEigenToMsg(e_wTs2, box_pose);
    tf::poseEigenToMsg(e_wTs3, sphere2_pose);
    
    std::vector<shapes::ShapeMsg> shapes;
    
    std::vector<geometry_msgs::Pose> poses;
    shapes.push_back(sphere1);
    shapes.push_back(box1);
    shapes.push_back(sphere2);
    poses.push_back(sphere_pose);
    poses.push_back(box_pose);
    poses.push_back(sphere2_pose);

    FCLCollisionGeometryPtr cg=test_node.FilterObjectFromOctomap(octomap_,shapes,poses);
    ROS_INFO ( "Removing old Octomap from World" );
    test_node.removeCollisionObject(99);
    ros::Duration(1.0).sleep();
    ROS_INFO ( "Adding new Octomap to World" );
    test_node.addCollisionObject ( cg,e_wTsocto,99 );
    
    /* =========================================================== //

            Octomap post-filtering

    // =========================================================== */

    ROS_INFO ( "Checking Distance & Collision sphere post filtering " );    
    std::cout<<"Is sphere in collision"<<test_node.checkCollisionObjectWorld(sphere1,e_wTs1)<<std::endl;
    std::cout<<"Is box in collision"<<test_node.checkCollisionObjectWorld(box1,e_wTs2)<<std::endl;
    std::cout<<"Is sphere2 in collision"<<test_node.checkCollisionObjectWorld(sphere2,e_wTs3)<<std::endl;


    test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }
    test_node.checkDistanceObjectWorld ( box1,e_wTs2,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }


    test_node.checkDistanceObjectWorld ( sphere2,e_wTs3,obj_ids,obj_distances,p1w,p2w );
    for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                test_node.publishPoint ( p1w[i],"closest_point1_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2_box",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
    }
    return 0;
    

}