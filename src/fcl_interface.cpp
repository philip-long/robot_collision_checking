#include <voxel_grid/voxel_grid.h>

#include <robot_collision_checking/fcl_interface.h>

namespace robot_collision_checking
{
    FCLInterface::FCLInterface(ros::NodeHandle nh) : nh_(nh)
    {
        mkr_pub = nh_.advertise<visualization_msgs::Marker>("/visualization_marker", 1);
        fcl_collision_world.clear();
    }

    FCLInterface::~FCLInterface()
    {
        ROS_INFO("Destroying world");
    }

    bool FCLInterface::checkCollisionObjectWorld(const shape_msgs::SolidPrimitive &s1,
                                                 const Eigen::Affine3d &wT1)
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);

        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;

        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            fcl::collide(o1.get(),
                         fcl_collision_world[i]->collision_object.get(),
                         col_req,
                         col_result);
            if (col_result.isCollision())
            {
                return true;
            }
        }
        return false;
    }

    bool FCLInterface::checkCollisionObjectWorld(const shape_msgs::Mesh &s1,
                                                 const Eigen::Affine3d &wT1)
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);

        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;

        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            fcl::collide(o1.get(),
                         fcl_collision_world[i]->collision_object.get(),
                         col_req,
                         col_result);
            if (col_result.isCollision())
            {
                return true;
            }
        }
        return false;
    }

    bool FCLInterface::checkCollisionObjectWorld(const shape_msgs::SolidPrimitive &s1,
                                                 const Eigen::Affine3d &wT1,
                                                 std::vector<unsigned int> &id_collision_objects)
    {
        id_collision_objects.clear();
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);

        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        fcl::CollisionRequestd col_req;
        fcl::CollisionResultd col_result;
        bool in_collision(false);
        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            fcl::collide(o1.get(),
                         fcl_collision_world[i]->collision_object.get(),
                         col_req,
                         col_result);
            if (col_result.isCollision())
            {
                id_collision_objects.push_back(fcl_collision_world[i]->collision_id);
                in_collision = true;
            }
        }
        return in_collision;
    }

    bool FCLInterface::checkDistanceObjectWorld(FCLCollisionObjectPtr o1,
                                                std::vector<int> &id_collision_objects,
                                                std::vector<double> &objs_distance,
                                                std::vector<Eigen::Vector3d> &wP1,
                                                std::vector<Eigen::Vector3d> &wPobjs)
    {
        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            id_collision_objects[i] = fcl_collision_world[i]->collision_id;
            fcl::DistanceRequestd dist_req;
            dist_req.enable_nearest_points = true;
            dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

            fcl::DistanceResultd dist_result;
            dist_result.nearest_points[0].setZero();
            dist_result.nearest_points[1].setZero();

            fcl::distance(o1.get(),
                          fcl_collision_world[i]->collision_object.get(),
                          dist_req,
                          dist_result);

            for (int j = 0; j < 3; j++)
            {
                wP1[i](j) = dist_result.nearest_points[0][j];
                wPobjs[i](j) = dist_result.nearest_points[1][j];
            }
            objs_distance[i] = dist_result.min_distance;
        }

        return true;
    }

    bool FCLInterface::checkDistanceObjectWorld(const shape_msgs::SolidPrimitive &s1,
                                                const Eigen::Affine3d &wT1,
                                                std::vector<int> &id_collision_objects,
                                                std::vector<double> &objs_distance,
                                                std::vector<Eigen::Vector3d> &wP1,
                                                std::vector<Eigen::Vector3d> &wPobjs)
    {
        unsigned int world_size = fcl_collision_world.size();
        id_collision_objects.clear();
        id_collision_objects.resize(world_size);
        wP1.clear();
        wP1.resize(world_size);
        wPobjs.clear();
        wPobjs.resize(world_size);
        objs_distance.clear();
        objs_distance.resize(world_size);

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);

        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);
        return checkDistanceObjectWorld(o1, id_collision_objects, objs_distance, wP1, wPobjs);
    }

    bool FCLInterface::checkDistanceObjectWorld(const shape_msgs::Mesh &s1,
                                                const Eigen::Affine3d &wT1,
                                                std::vector<int> &id_collision_objects,
                                                std::vector<double> &objs_distance,
                                                std::vector<Eigen::Vector3d> &wP1,
                                                std::vector<Eigen::Vector3d> &wPobjs)
    {
        unsigned int world_size = fcl_collision_world.size();
        id_collision_objects.clear();
        id_collision_objects.resize(world_size);
        wP1.clear();
        wP1.resize(world_size);
        wPobjs.clear();
        wPobjs.resize(world_size);
        objs_distance.clear();
        objs_distance.resize(world_size);

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);
        return checkDistanceObjectWorld(o1, id_collision_objects, objs_distance, wP1, wPobjs);
    }

    double FCLInterface::checkMinimumDistanceObjectWorld(const shape_msgs::SolidPrimitive &s1,
                                                         const Eigen::Affine3d &wT1)
    {

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);
        double min_distance = checkMinimumDistanceObjectWorld(o1);
        return min_distance;
    }

    double FCLInterface::checkMinimumDistanceObjectWorld(const shape_msgs::Mesh &s1,
                                                         const Eigen::Affine3d &wT1)
    {

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);
        double min_distance = checkMinimumDistanceObjectWorld(o1);
        return min_distance;
    }

    double FCLInterface::checkMinimumDistanceObjectWorld(FCLCollisionObjectPtr o1)
    {
        double min_distance(100);
        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            fcl::DistanceRequestd dist_req;
            dist_req.enable_nearest_points = true;
            fcl::DistanceResultd dist_result;
            FCLCollisionObjectPtr p;
            dist_result.nearest_points[0].setZero();
            dist_result.nearest_points[1].setZero();
            dist_req.enable_nearest_points = true;
            dist_req.gjk_solver_type = fcl::GJKSolverType::GST_LIBCCD;

            fcl::distance(o1.get(),
                          fcl_collision_world[i]->collision_object.get(),
                          dist_req,
                          dist_result);

            if (min_distance > dist_result.min_distance)
            {
                min_distance = dist_result.min_distance;
            }
        }
        return min_distance;
    }

    void FCLInterface::publishPoint(Eigen::Vector3d pose,
                                    std::string mkr_namespace,
                                    unsigned int id,
                                    std::string frame,
                                    std::vector<double> color,
                                    std::vector<double> scale)
    {
        visualization_msgs::Marker mkr;
        mkr.ns = mkr_namespace;
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.type = visualization_msgs::Marker::SPHERE;
        mkr.id = id;
        mkr.pose.position.x = pose(0);
        mkr.pose.position.y = pose(1);
        mkr.pose.position.z = pose(2);
        mkr.pose.orientation.x = 0.0;
        mkr.pose.orientation.y = 0.0;
        mkr.pose.orientation.z = 0.0;
        mkr.pose.orientation.w = 0.0;
        mkr.header.frame_id = frame;
        mkr.lifetime = ros::Duration(0.0);
        mkr.color.r = color[0];
        mkr.color.g = color[1];
        mkr.color.b = color[2];
        mkr.color.a = 0.8;
        mkr.scale.x = scale[0];
        mkr.scale.y = scale[1];
        mkr.scale.z = scale[2];
        while (mkr_pub.getNumSubscribers() < 1 && ros::ok())
        {
            ROS_INFO("Waiting for subs");
            ros::Duration(0.1).sleep();
        }
        mkr_pub.publish(mkr);
    }

    bool FCLInterface::addCollisionObject(FCLObjectSet &objects)
    {
        unsigned int id(0);
        for (auto obj : objects)
        {
            addCollisionObject(obj, id);
            id++;
        }
        return true;
    }

    bool FCLInterface::addCollisionObject(FCLObject &object,
                                          unsigned int object_id)
    {
        addCollisionObject(object.object_shape, object.object_transform, object_id);
        return true;
    }

    bool FCLInterface::addCollisionObject(const shape_msgs::SolidPrimitive &s1,
                                          const Eigen::Affine3d &wT1,
                                          unsigned int object_id)
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject());
        new_col_object->collision_object = o1;
        new_col_object->object_type = s1.type;
        new_col_object->collision_id = object_id;
        new_col_object->solid = s1;
        fcl_collision_world.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
        return true;
    }

    bool FCLInterface::addCollisionObject(const shape_msgs::Plane &s1,
                                          const Eigen::Affine3d &wT1, unsigned int object_id)
    {

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject());
        new_col_object->collision_object = o1;
        new_col_object->object_type = PLANE;
        new_col_object->collision_id = object_id;
        new_col_object->plane = s1;
        fcl_collision_world.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
        return true;
    }

    bool FCLInterface::addCollisionObject(const shape_msgs::Mesh &s1,
                                          const Eigen::Affine3d &wT1, unsigned int object_id)
    {

        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(s1);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject());
        new_col_object->collision_object = o1;
        new_col_object->object_type = visualization_msgs::Marker::TRIANGLE_LIST;
        new_col_object->collision_id = object_id;
        new_col_object->mesh = s1;
        fcl_collision_world.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
        return true;
    }

    // Adding octomap here
    bool FCLInterface::addCollisionObject(const octomap_msgs::Octomap &map,
                                          const Eigen::Affine3d &wT1,
                                          unsigned int object_id)
    {
        FCLCollisionGeometryPtr cg = FCLInterface::createCollisionGeometry(map);
        fcl::Transform3d wTf1;
        FCLInterface::transform2fcl(wT1, wTf1);
        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

        FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject());
        new_col_object->collision_object = o1;
        new_col_object->object_type = OCTOMAP_INT;
        new_col_object->collision_id = object_id;
        fcl_collision_world.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
        return true;
    }

    // Adding voxel grid here
    bool FCLInterface::addCollisionObject(const costmap_2d::VoxelGrid &grid,
                                          const Eigen::Affine3d &wT1,
                                          unsigned int object_id)
    {
        const uint32_t *data = &grid.data.front();
        const double x_origin = grid.origin.x;
        const double y_origin = grid.origin.y;
        const double z_origin = grid.origin.z;
        const double x_res = grid.resolutions.x;
        const double y_res = grid.resolutions.y;
        const double z_res = grid.resolutions.z;
        const uint32_t x_size = grid.size_x;
        const uint32_t y_size = grid.size_y;
        const uint32_t z_size = grid.size_z;

        for (uint32_t y_grid = 0; y_grid < y_size; ++y_grid)
        {
            for (uint32_t x_grid = 0; x_grid < x_size; ++x_grid)
            {
                for (uint32_t z_grid = 0; z_grid < z_size; ++z_grid)
                {
                    voxel_grid::VoxelStatus status = voxel_grid::VoxelGrid::getVoxel(x_grid, y_grid, z_grid,
                                                                                     x_size, y_size, z_size, data);

                    if (status == voxel_grid::MARKED)
                    {
                        // Center point of the cell
                        double x = x_origin + (x_grid + 0.5) * x_res;
                        double y = y_origin + (y_grid + 0.5) * y_res;
                        double z = z_origin + (z_grid + 0.5) * z_res;
                        Eigen::Vector3d center(x, y, z);

                        // Translate to origin of cell
                        Eigen::Vector3d box_trans = wT1 * center;
                        Eigen::Affine3d wTv = wT1;
                        wTv.translate(center);

                        fcl::Transform3d wTf1;
                        FCLInterface::transform2fcl(wTv, wTf1);

                        // Make collision geometry
                        Eigen::Vector3d side(x_res, y_res, z_res);
                        FCLCollisionGeometryPtr cg = std::make_shared<fcl::Boxd>(side);
                        FCLCollisionObjectPtr o1 = std::make_shared<fcl::CollisionObjectd>(cg, wTf1);

                        FCLInterfaceCollisionObject *new_col_object(new FCLInterfaceCollisionObject());
                        new_col_object->collision_object = o1;
                        new_col_object->object_type = VOXEL_GRID_INT;
                        new_col_object->collision_id = object_id;

                        // shape_msgs::SolidPrimitive s1;
                        // s1.type = shape_msgs::SolidPrimitive::BOX;
                        // s1.dimensions.resize(3);
                        // s1.dimensions[shape_msgs::SolidPrimitive::BOX_X] = x_res;
                        // s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = y_res;
                        // s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = z_res;
                        // new_col_object->solid = s1;

                        fcl_collision_world.push_back(std::unique_ptr<FCLInterfaceCollisionObject>(new_col_object));
                    }
                }
            }
        }

        return true;
    }

    bool FCLInterface::removeCollisionObject(unsigned int object_id)
    {
        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            if (object_id == fcl_collision_world[i]->collision_id)
            {
                fcl_collision_world.erase(fcl_collision_world.begin() + i);
                return true;
            }
        }
        return false;
    }

    bool FCLInterface::removeVoxelGridObject(unsigned int object_id)
    {
        unsigned int world_size = fcl_collision_world.size();
        // Assumes contingent cells
        unsigned int j = 0;
        for (auto it = fcl_collision_world.begin(); it != fcl_collision_world.end(); /*it++*/)
        {
            if (object_id == (*it)->collision_id)
            {
                it = fcl_collision_world.erase(it);
                j++;
            }
            else
            {
                ++it;
            }
        }

        if (j > 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool FCLInterface::displayMarker(shape_msgs::SolidPrimitive s1,
                                     const Eigen::Affine3d &T,
                                     unsigned int obj_id, Eigen::Vector4d color)
    {
        visualization_msgs::Marker mkr;
        geometric_shapes::constructMarkerFromShape(s1, mkr);
        while (mkr_pub.getNumSubscribers() < 1)
        {
            ROS_INFO_ONCE("Waiting until marker is displayed in RVIZ");
            // ros::spinOnce();
            ros::Duration(0.05).sleep();
        }
        mkr.action = visualization_msgs::Marker::ADD;
        mkr.header.frame_id = "world";
        mkr.ns = "Objects";
        mkr.lifetime = ros::Duration(0.0);
        mkr.id = obj_id;
        mkr.color.r = color(0);
        mkr.color.g = color(1);
        mkr.color.b = color(2);
        mkr.color.a = color(3);
        Eigen::Quaterniond q(T.linear());
        mkr.pose.position.x = T(0, 3);
        mkr.pose.position.y = T(1, 3);
        mkr.pose.position.z = T(2, 3);
        mkr.pose.orientation.w = q.w();
        mkr.pose.orientation.x = q.x();
        mkr.pose.orientation.y = q.y();
        mkr.pose.orientation.z = q.z();
        mkr_pub.publish(mkr);
        // ros::spinOnce();
        return true;
    }

    bool FCLInterface::displayObjects(std::string frame_name)
    {
        visualization_msgs::Marker mkr;
        while (mkr_pub.getNumSubscribers() < 1)
        {
            ROS_INFO("Waiting for Marker Subs");
            // ros::spinOnce();
            ros::Duration(0.1).sleep();
        }

        unsigned int world_size = fcl_collision_world.size();
        for (unsigned int i = 0; i < world_size; i++)
        {
            fcl::Transform3d wTf2 = fcl_collision_world[i]->collision_object->getTransform();

            mkr.id = fcl_collision_world[i]->collision_id;
            if (fcl_collision_world[i]->object_type == visualization_msgs::Marker::TRIANGLE_LIST)
            {
                geometric_shapes::constructMarkerFromShape(fcl_collision_world[i]->mesh, mkr);
            }
            else if (fcl_collision_world[i]->object_type == PLANE)
            {
                ROS_WARN("Unable to display plane");
            }
            else if (fcl_collision_world[i]->object_type == OCTOMAP_INT)
            {
                ROS_WARN("Unable to display octomap");
            }
            else if (fcl_collision_world[i]->object_type == VOXEL_GRID_INT)
            {
                // geometric_shapes::constructMarkerFromShape(fcl_collision_world[i]->solid, mkr);
            }
            else if (fcl_collision_world[i]->object_type == shape_msgs::SolidPrimitive::SPHERE ||
                     fcl_collision_world[i]->object_type == shape_msgs::SolidPrimitive::BOX ||
                     fcl_collision_world[i]->object_type == shape_msgs::SolidPrimitive::CYLINDER)
            {
                geometric_shapes::constructMarkerFromShape(fcl_collision_world[i]->solid, mkr);
            }

            mkr.action = visualization_msgs::Marker::ADD;
            mkr.header.frame_id = frame_name;
            mkr.ns = "CollisionObjects";
            mkr.lifetime = ros::Duration(0.0);
            mkr.color.a = 1.0;
            mkr.color.g = 1.0;

            Eigen::Quaterniond q(wTf2.linear());
            mkr.pose.position.x = wTf2(0, 3);
            mkr.pose.position.y = wTf2(1, 3);
            mkr.pose.position.z = wTf2(2, 3);
            mkr.pose.orientation.w = q.w();
            mkr.pose.orientation.x = q.x();
            mkr.pose.orientation.y = q.y();
            mkr.pose.orientation.z = q.z();
            mkr_pub.publish(mkr);

            ros::Duration(0.02).sleep();
        }
        return true;
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::Mesh &s1)
    {
        std::shared_ptr<fcl::BVHModel<fcl::OBBRSSd>> g(new fcl::BVHModel<fcl::OBBRSSd>());

        unsigned int vert_size = s1.vertices.size();
        unsigned int tri_size = s1.triangles.size();
        if (vert_size > 0 && tri_size > 0)
        {
            std::vector<fcl::Triangle> tri_indices(tri_size);
            std::vector<fcl::Vector3d> points(vert_size);

            for (unsigned int i = 0; i < tri_size; ++i)
            {
                tri_indices[i] =
                    fcl::Triangle(s1.triangles[i].vertex_indices[0],
                                  s1.triangles[i].vertex_indices[1],
                                  s1.triangles[i].vertex_indices[2]);
            }

            for (unsigned int i = 0; i < vert_size; ++i)
            {
                points[i] = fcl::Vector3d(s1.vertices[i].x, s1.vertices[i].y, s1.vertices[i].z);
            }

            g->beginModel();
            g->addSubModel(points, tri_indices);
            g->endModel();
        }
        return g;
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const octomap_msgs::Octomap &map)
    {
        std::shared_ptr<octomap::OcTree> om(static_cast<octomap::OcTree *>(octomap_msgs::msgToMap(map)));
        return createCollisionGeometry(om);
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const std::shared_ptr<const octomap::OcTree> &tree)
    {
        return FCLCollisionGeometryPtr(new fcl::OcTreed(tree));
    }

    // Plane is defined as ax + by + cz + d = 0;
    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::Plane &s1)
    {
        return FCLCollisionGeometryPtr(
            new fcl::Planed(s1.coef[0],
                            s1.coef[1],
                            s1.coef[2],
                            s1.coef[3]));
    }

    FCLCollisionGeometryPtr FCLInterface::createCollisionGeometry(const shape_msgs::SolidPrimitive &s1)
    {
        if (s1.type == shape_msgs::SolidPrimitive::SPHERE)
        {
            return std::make_shared<fcl::Sphered>(s1.dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS]);
        }
        else if (s1.type == shape_msgs::SolidPrimitive::BOX)
        {

            return std::make_shared<fcl::Boxd>(s1.dimensions[shape_msgs::SolidPrimitive::BOX_X],
                                               s1.dimensions[shape_msgs::SolidPrimitive::BOX_Y],
                                               s1.dimensions[shape_msgs::SolidPrimitive::BOX_Z]);
        }
        else if (s1.type == shape_msgs::SolidPrimitive::CONE)
        {
            return std::make_shared<fcl::Coned>(s1.dimensions[shape_msgs::SolidPrimitive::CONE_RADIUS],
                                                s1.dimensions[shape_msgs::SolidPrimitive::CONE_HEIGHT]);
        }
        else if (s1.type == shape_msgs::SolidPrimitive::CYLINDER)
        {
            return std::make_shared<fcl::Cylinderd>(s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS],
                                                    s1.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT]);
        }
        else
        {
            return nullptr;
        }
    }
}