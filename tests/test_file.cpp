#include <iostream>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/geometric_shape_to_BVH_model.h>
#include <fcl/geometry/octree/octree.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <ccd/vec3.h>
#include <robot_collision_checking/fcl_interface.h>
//#include <fcl/geometry/bvh/BVH_model.h>

int main()
{
    // ccd_real_t a;
    std::vector<fcl::Triangle> t1, t2;
    fcl::Sphered s1(1.0), s2(1.0);
    fcl::Vector3d wps1(0.0, 0.0, 0.0), wps2(10.0, 0.0, 0.0), wps3(5.0, 1.0, 0.0), wps4(5.0, 0.0, 0.0), wps5(4.21, 3.0, 0.21);
    fcl::Matrix3d wRs1, wRs2, wRs3, wRs4, wRs5;
    wRs1.setIdentity();
    wRs2.setIdentity();
    wRs3.setIdentity();
    wRs4.setIdentity();
    wRs5.setIdentity();
    fcl::Transform3d wTs1, wTs4, wTs5; //(wRs1,wps1);

    wTs1.linear() = wRs1;
    wTs1.translation() = wps1;
    fcl::Transform3d wTs2;
    wTs2.linear() = wRs2;
    wTs2.translation() = wps2;
    fcl::Transform3d wTs3;
    wTs3.linear() = wRs3;
    wTs3.translation() = wps3;
    wTs4.linear() = wRs4;
    wTs4.translation() = wps4;
    wTs5.linear() = wRs5;
    wTs5.translation() = wps5;

    std::cout << s1.computeVolume() << std::endl;
    std::shared_ptr<fcl::CollisionGeometryd> cg_1(new fcl::Sphered(1.0));
    std::shared_ptr<fcl::CollisionGeometryd> cg_2(new fcl::Sphered(4.0));
    std::shared_ptr<fcl::CollisionGeometryd> cg_3(new fcl::Boxd(0.2, 0.2, 0.2));
    std::shared_ptr<fcl::CollisionGeometryd> cg_4(new fcl::Cylinderd(0.1, 1.0));
    std::shared_ptr<fcl::CollisionGeometryd> cg_5(new fcl::Boxd(0.25, 0.5, 0.2));

    //
    fcl::CollisionObjectd *o1 = new fcl::CollisionObjectd(cg_1, wTs1);
    fcl::CollisionObjectd *o2 = new fcl::CollisionObjectd(cg_2, wTs2);
    fcl::CollisionObjectd *o3 = new fcl::CollisionObjectd(cg_3, wTs3);
    fcl::CollisionObjectd *o4 = new fcl::CollisionObjectd(cg_4, wTs4);
    fcl::CollisionObjectd *o5 = new fcl::CollisionObjectd(cg_5, wTs5);

    fcl::CollisionRequestd request;
    fcl::DistanceRequestd dist_req;
    fcl::detail::GJKSolver_libccd<double> solver1;
    dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;
    dist_req.enable_nearest_points = true;

    //     // result will be returned via the collision result structure
    fcl::CollisionResultd result;
    fcl::DistanceResultd dist_result, dist_result5;
    //     // perform distance test
    fcl::collide(o1, o2, request, result);
    dist_result.nearest_points[0].setZero();
    dist_result.nearest_points[1].setZero();
    std::cout << "result.isCollision()" << result.numContacts() << std::endl;
    fcl::distance(o1, o2, dist_req, dist_result);
    std::cout << o1->getTranslation() << std::endl;
    std::cout << o2->getTranslation() << std::endl;

    std::cout << "Min distance " << dist_result.min_distance << std::endl;
    std::cout << "Nearest point 1   = " << dist_result.nearest_points[0] << std::endl;
    std::cout << "Nearest point 2   = " << dist_result.nearest_points[1] << std::endl;

    fcl::distance(o1, o3, dist_req, dist_result);
    //
    std::cout << "Min distance " << dist_result.min_distance << std::endl;
    std::cout << "Nearest point 1   = " << dist_result.nearest_points[0] << std::endl;
    std::cout << "Nearest point 2   = " << dist_result.nearest_points[1] << std::endl;

    fcl::distance(o1, o4, dist_req, dist_result);
    //
    std::cout << "Min distance " << dist_result.min_distance << std::endl;
    std::cout << "Nearest point 1   = " << dist_result.nearest_points[0] << std::endl;
    std::cout << "Nearest point 2   = " << dist_result.nearest_points[1] << std::endl;

    fcl::distance(o1, o5, dist_req, dist_result5);
    //
    std::cout << "Min distance " << dist_result5.min_distance << std::endl;
    std::cout << "Nearest point 1   = " << dist_result5.nearest_points[0] << std::endl;

    std::cout << "Nearest point 2   = " << dist_result5.nearest_points[1] << std::endl;
    return 0;
}
