#include <robot_collision_checking/fcl_interface.h>


octomap_msgs::Octomap octomap_;

bool octomap_received ( false ),lin_callback_received(false);

void octoCallback ( const octomap_msgs::Octomap::ConstPtr& msg ) {
    octomap_=*msg;
    octomap_received=true;
    std::cout<<"received callback"<<std::endl;
}



int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "octomap" ); // ros init
    ros::NodeHandle nh; // Create a node handle and start the node
    ros::Subscriber  octo_sub= nh.subscribe ( "/octomap_full",
                               1, &octoCallback );

    std::cout<<"---------------------------------------------"<<std::endl;
    ROS_INFO ( "SIMPLE FCL INTERFACE" );
    std::cout<<"---------------------------------------------"<<std::endl;
    Eigen::Vector3d e_wps1 ( 0.9,0.3,0.8 );
    Eigen::Vector3d e_wps2 (0.5,0.0,1.0 );
    Eigen::Vector3d e_wps3 ( 2.0,0.5,0.0 );
    Eigen::Vector3d e_octo ( 0.0,0.0,0.0);
    Eigen::Matrix3d e_wRs1,e_wRs2,e_wRs3,e_wRsocto;
    Eigen::Affine3d e_wTs1,e_wTs2,e_wTs3,e_wTsocto;

    e_wRs1.setIdentity();
    e_wRs2.setIdentity();
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
    sphere1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.1;
    sphere1.type=shape_msgs::SolidPrimitive::SPHERE;

    sphere2.dimensions.resize ( 1 );
    sphere2.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]=0.1;
    sphere2.type=shape_msgs::SolidPrimitive::SPHERE;

    box1.dimensions.resize ( 3 );
    box1.type=shape_msgs::SolidPrimitive::BOX;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_X]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Y]=0.2;
    box1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]=0.2;

     
    FCLInterface test_node ( nh );
    ROS_INFO ( "Waiting a few seconds for RVIZ to start up" );
    ros::Duration(5.0).sleep();    
    test_node.addCollisionObject ( sphere1,e_wTs1,0 );
    test_node.displayObjects();
    test_node.removeCollisionObject(0);

    ros::Duration(0.2).sleep();
    std::vector<double> obj_distances;
    std::vector<int> obj_ids;
    std::vector<Eigen::Vector3d> p1w,p2w;
    std::vector<unsigned int> id_collision_objects;
    id_collision_objects.resize(0);
    while(ros::ok())
    {
        if(octomap_received)
        {          
            test_node.removeCollisionObject(99 );
            test_node.addCollisionObject ( octomap_,e_wTsocto,99 );
            bool is_in_collision=test_node.checkCollisionObjectWorld(sphere1,e_wTs1);
            ROS_WARN_COND(is_in_collision,"Octomap in collision");
            test_node.checkDistanceObjectWorld ( sphere1,e_wTs1,obj_ids,obj_distances,p1w,p2w );

            for ( unsigned int i=0; i<obj_distances.size(); i++ ) {
                std::cout<<"Obj id"<<obj_ids[i]<<std::endl;
                std::cout<<"Distance  "<<obj_distances[i]<<std::endl;
                std::cout<<" Closest Points p1w = ["<<p1w[i] ( 0 ) <<", "<<p1w[i] ( 1 ) <<", "<<p1w[i] ( 2 ) <<"]"<<std::endl;
                std::cout<<"                p2w = ["<<p2w[i] ( 0 ) <<", "<<p2w[i] ( 1 ) <<", "<<p2w[i] ( 2 ) <<"]"<<std::endl;
                int n;
                test_node.publishPoint ( p1w[i],"closest_point1",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );
                test_node.publishPoint ( p2w[i],"closest_point2",i,"world", {0.1,0.1,0.1}, {0.05,0.05,0.05} );


            }
            for ( unsigned int i=0; i<id_collision_objects.size(); i++ ) {
                std::cout<<"Obj id "<<id_collision_objects[i]<<"in collision "<<std::endl;
            }

        }
        ros::Duration(0.2).sleep();
        ros::spinOnce();
    }


    return 1;
}

