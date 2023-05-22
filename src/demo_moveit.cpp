#include "ros/ros.h"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Dense>

/*
Help function to add a box to the planning scene
*/
void addBOX(moveit::planning_interface::PlanningSceneInterface &planning_scene, double dim_x, double dim_y, double dim_z, geometry_msgs::PoseStamped pose, const std::string &obj_id)
{
    std::cout << "\n#####\nAdding box [" << dim_x << ", " << dim_y << ", " << dim_z << "]\n pose:\n"
              << pose << "\n#####\n";

    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = pose.header.frame_id;
    collision_object.id = obj_id;

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = dim_x;
    primitive.dimensions[primitive.BOX_Y] = dim_y;
    primitive.dimensions[primitive.BOX_Z] = dim_z;

    pose.pose.position.z += dim_z / 2.0;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(pose.pose);
    collision_object.operation = collision_object.ADD;

    planning_scene.applyCollisionObject(collision_object);
    ros::Duration(1).sleep();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "demo_moveit_ex");
    ros::NodeHandle node_handle;

    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    static const std::string EE_LINK = "panda_hand_tcp";

    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    const moveit::core::JointModelGroup *joint_model_group =
        move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    /* Scelgo il planner da usare */
    // move_group_interface.setPlannerId("RRTstar");
    move_group_interface.setPlannerId("RRTConnect");
    // move_group_interface.setPlannerId("PRMstar");

    // Parametri della scena [HARD CODED!]
    double x1 = 0.5;
    double y1 = 0.60;
    double DX_0 = 0.2;
    double Dy_f = 0.25;
    double Table_DX = 0.5;
    double Table_DY = 1.0;
    double Table_DZ = 0.4;
    double DX = 0.5;
    double DX_2 = 0.25;
    double DZ_2 = 0.5;
    double DZ_3 = 0.25;
    double Dz_grasp = 0.2;

    // PLAN GRASP
    {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

        // Eigen::Quaterniond q;
        // q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitY());

        move_group_interface.setStartStateToCurrentState(); //<-- it is usefull to update the start state to the current state
        geometry_msgs::Pose pose;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = x1 + DX_0;
        pose.position.y = 0.0;
        pose.position.z = Table_DZ + Dz_grasp;

        std::cout << "Plan target:\n"
                  << pose << "\n";
        // return 0;
        move_group_interface.clearPathConstraints();
        move_group_interface.setPlanningTime(10.0);
        move_group_interface.setPoseTarget(pose, EE_LINK);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "plan %s", success ? "" : "FAILED");
        if (success)
        {
            move_group_interface.execute(my_plan);
        }
        else
        {
            return -1;
        }
    }
    char ans;
    std::cin >> ans;

    // obj to attach
    {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.pose.orientation.w = 1.0;
        pose.pose.position.x = x1 + DX_0;
        pose.pose.position.y = 0.0;
        pose.pose.position.z = Table_DZ;
        addBOX(planning_scene_interface, 0.02, 0.02, 0.22, pose, "attach_obj");
        move_group_interface.attachObject("attach_obj", "panda_hand_sc");
    }
    std::cin >> ans;

    // PLAN PLACE
    {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());

        // ADD CONSTRAINTS
        moveit_msgs::OrientationConstraint ocm;
        ocm.link_name = EE_LINK;
        ocm.header.frame_id = "world";
        ocm.orientation.x = q.x();
        ocm.orientation.y = q.y();
        ocm.orientation.z = q.z();
        ocm.orientation.w = q.w();
        ocm.absolute_x_axis_tolerance = 2.0 * M_PI;
        ocm.absolute_y_axis_tolerance = 0.1 * M_PI / 180.0;
        ocm.absolute_z_axis_tolerance = 0.1 * M_PI / 180.0;
        ocm.weight = 100.0;
        moveit_msgs::Constraints test_constraints;
        test_constraints.orientation_constraints.push_back(ocm);
        // move_group_interface.setPathConstraints(test_constraints); // <-- uncomment for orientation constraint

        move_group_interface.setStartStateToCurrentState(); //<-- it is usefull to update the start state to the current state
        geometry_msgs::Pose pose;
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();
        pose.position.x = 0.0;
        pose.position.y = y1 + Dy_f;
        pose.position.z = Table_DZ + Dz_grasp;

        std::cout << "Plan target:\n"
                  << pose << "\n";

        move_group_interface.setPlanningTime(10.0); // <-- change it in case of problems...
        move_group_interface.setPoseTarget(pose, EE_LINK);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("tutorial", "plan %s", success ? "" : "FAILED");
        if (success)
        {
            move_group_interface.execute(my_plan);
        }
        else
        {
            return -1;
        }

        move_group_interface.detachObject("attach_obj");
    }
    std::cin >> ans;
    return 0;
}
