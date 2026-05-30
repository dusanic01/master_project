#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit/trajectory_processing/ruckig_traj_smoothing.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
#include <condition_variable>

class PandaController {
public:
    PandaController(moveit::planning_interface::MoveGroupInterface& mg,
                    moveit::planning_interface::MoveGroupInterface& hg)
        : move_group(mg),
          hand_group(hg),
          ac_move("/franka_gripper/move", false),
          ac_grasp("/franka_gripper/grasp", false),
          pose_received(false), pom_pose_received(false)
    {
        ros::NodeHandle nh;
        pose_sub = nh.subscribe("/pose", 10, &PandaController::poseCallback, this);
        pom_pose_sub = nh.subscribe("/pom_pose", 10, &PandaController::pomPoseCallback, this);

        move_group.setMaxVelocityScalingFactor(0.2);
        move_group.setMaxAccelerationScalingFactor(0.1);

        ROS_INFO("Waiting for gripper servers...");
        ac_move.waitForServer();
        ac_grasp.waitForServer();
        ROS_INFO("Gripper servers connected.");

        Floor();
    }

    void run() {
        ros::Rate rate(10);
        while (ros::ok()) {

            geometry_msgs::Pose target_pose = move_group.getCurrentPose().pose;
            try{
                target_pose.position.x = 0.4;
                target_pose.position.y = 0.4;
                target_pose.position.z = 0.4;
                tf2::fromMsg(target_pose.orientation, q_old);
                q_start = q_old;

                std::vector<geometry_msgs::Pose> waypoints;
                moveToPose(target_pose);
                ros::Duration(0.2).sleep();

                openGripper(0.08, 0.08);

                //  Čekanje nove pozicije 
                ros::Duration(5).sleep();
                waitForPose();

                std::array<double,3> local_pom_position;
                std::array<double,3> local_current_position;
                tf2::Quaternion local_q_now;

                {
                    std::lock_guard<std::mutex> lock(pose_mutex);
                    local_pom_position = pom_position;
                    local_current_position = current_position;
                    local_q_now = q_now;
                }

                if(local_current_position[2]<0.1134){
                    ROS_ERROR("Detektovana pozicija je preniska!");
                    continue;
                }

                //  Pomak na novu poziciju iz callbacka 
                target_pose.position.x = local_pom_position[0];
                target_pose.position.y = local_pom_position[1];
                target_pose.position.z = local_pom_position[2];
                q_new = q_old * local_q_now;
                target_pose.orientation = tf2::toMsg(q_new);
                waypoints = {move_group.getCurrentPose().pose, target_pose};
                executeTrajectory(waypoints);

                target_pose.position.x = local_current_position[0];
                target_pose.position.y = local_current_position[1];
                target_pose.position.z = local_current_position[2];
                waypoints = {move_group.getCurrentPose().pose, target_pose};
                executeTrajectory(waypoints);

                //  Hvatanje objekta 
                graspObject(0.025, 0.05, 80);
                // hand_group.setNamedTarget("close");
                // hand_group.move();

                ros::Duration(2).sleep();

                //  Pomak gore 
                target_pose.position.z += 0.2;
                waypoints = {move_group.getCurrentPose().pose, target_pose};
                executeTrajectory(waypoints);

                //  Odlaganje objekta - kretanje pravolinijskom putanjom
                // target_pose.position.x = 0.4;
                // target_pose.position.y = -0.3;
                // waypoints = {move_group.getCurrentPose().pose, target_pose};
                // executeTrajectory(waypoints);

                // target_pose.orientation = tf2::toMsg(q_start);
                // waypoints = {move_group.getCurrentPose().pose, target_pose};
                // executeTrajectory(waypoints);

                // Odlaganje objekta - kretanje proizvoljnom putanjom
                target_pose.position.x = 0.4;
                target_pose.position.y = -0.3;
                target_pose.orientation = tf2::toMsg(q_start);
                moveToPose(target_pose);
                ros::Duration(0.2).sleep();

                target_pose.position.z -= 0.196;
                waypoints = {move_group.getCurrentPose().pose, target_pose};
                executeTrajectory(waypoints);

                openGripper(0.08, 0.02);

                ros::Duration(2).sleep();

                target_pose.position.z += 0.2;
                waypoints = {move_group.getCurrentPose().pose, target_pose};
                executeTrajectory(waypoints);

                //  Vrati ruku u "home" 
                move_group.setNamedTarget("home");
                move_group.move();

                rate.sleep();
            }

            catch(const std::exception& e) {
                ROS_ERROR_STREAM("ABORT: " << e.what());
                move_group.stop();
                move_group.setNamedTarget("home");
                move_group.move();
                ros::Duration(2).sleep();
                return;
            }
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface& move_group;
    moveit::planning_interface::MoveGroupInterface& hand_group;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp;

    ros::Subscriber pose_sub;
    ros::Subscriber pom_pose_sub;

    std::array<double,3> current_position;
    std::array<double,3> pom_position;
    tf2::Quaternion q_now, q_new, q_old, q_start;
    bool pose_received;
    bool pom_pose_received;

    std::mutex pose_mutex;
    std::condition_variable pose_cv;

    void poseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        current_position[0] = msg->position.x;
        current_position[1] = msg->position.y;
        current_position[2] = msg->position.z;
        tf2::fromMsg(msg->orientation, q_now);
        pose_received = true;
        pose_cv.notify_all();
    }

    void pomPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pose_mutex);
        pom_position[0] = msg->position.x;
        pom_position[1] = msg->position.y;
        pom_position[2] = msg->position.z;
        pom_pose_received = true;
        pose_cv.notify_all();
    }

    void waitForPose() {
        std::unique_lock<std::mutex> lock(pose_mutex);
        pose_received = false;
        pom_pose_received = false;
        ROS_INFO("Waiting for the object's position.");
        pose_cv.wait(lock, [this]{ return pose_received && pom_pose_received; });
        ROS_INFO("Object position received.");
    }

    void executeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints) {
        moveit_msgs::RobotTrajectory trajectory;
        double eef_step = 0.005;

        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction < 0.9) {
            throw std::runtime_error("Cartesian path not fully computed!");
        }

        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        trajectory_processing::RuckigSmoothing ruckig;

        bool success = totg.computeTimeStamps(rt, 0.3, 0.1);

        // Ograničenja za trzaj
        if (success)
        {
            ruckig.applySmoothing(rt);
        }

        rt.getRobotTrajectoryMsg(trajectory);
        if(move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
            throw std::runtime_error("Controller Abort!");
        }
    }

    void moveToPose(const geometry_msgs::Pose& target_pose)
    {
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (!success) {
            move_group.clearPoseTargets();
            throw std::runtime_error("Path not fully computed!");
        }
        
        if (move_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
            move_group.clearPoseTargets();
            throw std::runtime_error("Controller Abort!");
        }
        
        move_group.clearPoseTargets();
    }

    void openGripper(double width, double speed) {
        franka_gripper::MoveGoal goal;
        goal.width = width;
        goal.speed = speed;
        ac_move.sendGoal(goal);
        ac_move.waitForResult();
    }

    void graspObject(double width, double speed, double force) {
        franka_gripper::GraspGoal goal;
        goal.width = width;
        goal.speed = speed;
        goal.force = force;
        goal.epsilon.inner = 0.01;
        goal.epsilon.outer = 0.01;
        ac_grasp.sendGoal(goal);
        ac_grasp.waitForResult();
    }

    void Floor() {
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
        moveit_msgs::CollisionObject collision_object;
        
        collision_object.header.frame_id = move_group.getPlanningFrame(); 
        collision_object.id = "podloga_robota";

        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 4.0;
        primitive.dimensions[primitive.BOX_Y] = 4.0;
        primitive.dimensions[primitive.BOX_Z] = 0.02;

        geometry_msgs::Pose floor_pose;
        floor_pose.orientation.w = 1.0;
        floor_pose.position.x = 0.0; 
        floor_pose.position.y = 0.0;
        floor_pose.position.z = -0.01; 

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(floor_pose);
        collision_object.operation = collision_object.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(collision_object);
        planning_scene_interface.applyCollisionObjects(collision_objects);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fr3_planning_class");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface move_group("fr3_arm");
    moveit::planning_interface::MoveGroupInterface hand_group("fr3_hand");

    PandaController panda(move_group, hand_group);
    panda.run();

    ros::shutdown();
    return 0;
}