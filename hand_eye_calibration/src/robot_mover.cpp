#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <actionlib/client/simple_action_client.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/GraspAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "std_msgs/Bool.h"
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
        pub = nh.advertise<std_msgs::Bool>("/robot_at_pose", 10);
        pub_end = nh.advertise<std_msgs::Bool>("/end_pose", 10);
        msg.data = true;
        end.data = true;

        move_group.setMaxVelocityScalingFactor(0.5);
        move_group.setMaxAccelerationScalingFactor(0.5);

        ROS_INFO("Waiting for gripper servers...");
        ac_move.waitForServer();
        ac_grasp.waitForServer();
        ROS_INFO("Gripper servers connected.");
    }

    void run() {
        ros::Rate rate(10);
        geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;

        // --- Primer cilja ---
        geometry_msgs::Pose target_pose = start_pose;
        std::vector<geometry_msgs::Pose> waypoints;
        std::vector<geometry_msgs::Pose> targets;

        target_pose.position.x = 0.685;
        target_pose.position.y = -0.046;
        target_pose.position.z = 0.376;
        q_new.setRPY(2.767, 0.353, -0.873);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.264;
        target_pose.position.y = 0.124;
        target_pose.position.z = 0.309;
        q_new.setRPY(-2.597, -0.385, -1.584);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.317;
        target_pose.position.y = -0.228;
        target_pose.position.z = 0.463;
        q_new.setRPY(2.810, 0.328, -2.696);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.648;
        target_pose.position.y = 0.248;
        target_pose.position.z = 0.430;
        q_new.setRPY(-2.728, 0.129, 2.189);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.188;
        target_pose.position.y = -0.277;
        target_pose.position.z = 0.453;
        q_new.setRPY(-2.763, -0.222, -0.277);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.683;
        target_pose.position.y = 0.098;
        target_pose.position.z = 0.364;
        q_new.setRPY(-2.902, -0.291, 3.030);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.176;
        target_pose.position.y = 0.314;
        target_pose.position.z = 0.449;
        q_new.setRPY(-3.008, -0.439, -0.915);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.571;
        target_pose.position.y = -0.058;
        target_pose.position.z = 0.318;
        q_new.setRPY(2.813, 0.238, -1.509);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.301;
        target_pose.position.y = -0.060;
        target_pose.position.z = 0.424;
        q_new.setRPY(3.051, -0.452, 0.092);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        target_pose.position.x = 0.613;
        target_pose.position.y = -0.150;
        target_pose.position.z = 0.410;
        q_new.setRPY(-2.892, -0.213, 1.837);
        target_pose.orientation = tf2::toMsg(q_new);
        waypoints.push_back(target_pose);

        // target_pose.position.x = 0.618;
        // target_pose.position.y = 0.083;
        // target_pose.position.z = 0.328;
        // q_new.setRPY(2.730, 0.366, -0.653);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.503;
        // target_pose.position.y = -0.179;
        // target_pose.position.z = 0.381;
        // q_new.setRPY(3.036, 0.465, -1.539);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.400;
        // target_pose.position.y = -0.279;
        // target_pose.position.z = 0.378;
        // q_new.setRPY(-2.593, 0.151, -0.609);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.461;
        // target_pose.position.y = 0.190;
        // target_pose.position.z = 0.255;
        // q_new.setRPY(-2.717, -0.496, -2.096);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.596;
        // target_pose.position.y = 0.275;
        // target_pose.position.z = 0.414;
        // q_new.setRPY(2.582, 0.574, 0.175);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.586;
        // target_pose.position.y = 0.051;
        // target_pose.position.z = 0.280;
        // q_new.setRPY(2.590, -0.127, -1.086);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.351;
        // target_pose.position.y = -0.055;
        // target_pose.position.z = 0.409;
        // q_new.setRPY(-3.051, 0.041, -1.988);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.575;
        // target_pose.position.y = -0.115;
        // target_pose.position.z = 0.461;
        // q_new.setRPY(-3.026, 0.429, -0.087);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.337;
        // target_pose.position.y = 0.184;
        // target_pose.position.z = 0.404;
        // q_new.setRPY(-2.823, -0.453, -1.783);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.473;
        // target_pose.position.y = 0.316;
        // target_pose.position.z = 0.413;
        // q_new.setRPY(-2.396, -0.206, -2.849);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);
        
        // target_pose.position.x = 0.597;
        // target_pose.position.y = 0.047;
        // target_pose.position.z = 0.346;
        // q_new.setRPY(2.848, 0.076, -1.459);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        // target_pose.position.x = 0.446;
        // target_pose.position.y = -0.189;
        // target_pose.position.z = 0.334;
        // q_new.setRPY(-2.731, -0.022, -0.048);
        // target_pose.orientation = tf2::toMsg(q_new);
        // waypoints.push_back(target_pose);

        for(int i=0; i<waypoints.size(); i++)
        {
            // Pravolinijsko kretanje
            // targets = {move_group.getCurrentPose().pose, waypoints[i]};
            // executeTrajectory(targets);

            // Proizvoljno kretanje
            moveToPose(waypoints[i]);

            ros::Duration(0.5).sleep();
            pub.publish(msg);
            ros::Duration(1.5).sleep();
        }
        ros::Duration(2).sleep();
        pub_end.publish(end);
        /*
        move_group.setNamedTarget("home");
        move_group.move();
        */

        rate.sleep();
    }

private:
    moveit::planning_interface::MoveGroupInterface& move_group;
    moveit::planning_interface::MoveGroupInterface& hand_group;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> ac_move;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> ac_grasp;

    ros::Subscriber pose_sub;
    ros::Subscriber pom_pose_sub;
    ros::Publisher pub;
    ros::Publisher pub_end;

    std::array<double,3> current_position;
    std::array<double,3> pom_position;
    tf2::Quaternion q_now, q_new, q_old;
    bool pose_received;
    bool pom_pose_received;
    std_msgs::Bool msg, end;
    

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
        pose_cv.wait(lock, [this]{ return pose_received || pom_pose_received; });
        pose_received = false;
        pom_pose_received = false;
    }

    void executeTrajectory(const std::vector<geometry_msgs::Pose>& waypoints) {
        moveit_msgs::RobotTrajectory trajectory;
        double eef_step = 0.01;

        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction < 0.6) {
            ROS_ERROR("Cartesian path not fully computed!");
            return;
        }

        robot_trajectory::RobotTrajectory rt(move_group.getCurrentState()->getRobotModel(), move_group.getName());
        rt.setRobotTrajectoryMsg(*move_group.getCurrentState(), trajectory);

        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
        totg.computeTimeStamps(rt, 1.0, 1.0);

        rt.getRobotTrajectoryMsg(trajectory);
        move_group.execute(trajectory);
    }

    void moveToPose(const geometry_msgs::Pose& target_pose)
    {
        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success =
            (move_group.plan(plan) ==
            moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            move_group.execute(plan);
        }
        else
        {
            ROS_ERROR("Planning failed!");
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
        goal.epsilon.inner = 0.15;
        goal.epsilon.outer = 0.15;
        ac_grasp.sendGoal(goal);
        ac_grasp.waitForResult();
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