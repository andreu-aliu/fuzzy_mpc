#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include "utils/Config.hpp"
#include "utils/auxiliar.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "cat_msgs/msg/car_state.hpp"
#include "cat_msgs/msg/objective_array_curv.hpp"
#include "cat_msgs/msg/car_velocity_array.hpp"
#include "cat_msgs/msg/car_commands.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"


inline void fill_config(Config& cfg, rclcpp::Node* node)
{
    node->get_parameter("profile", cfg.profile);
    node->get_parameter("ws_path", cfg.ws_path);
    node->get_parameter("debug_path", cfg.debug_path);
    cfg.share_path = ament_index_cpp::get_package_share_directory("fuzzy_mpc") + "/";

    // Car parameters
    node->get_parameter("Car.m",  cfg.car.m);
    node->get_parameter("Car.I",  cfg.car.I);
    node->get_parameter("Car.Lf", cfg.car.Lf);
    node->get_parameter("Car.Lr", cfg.car.Lr);
    node->get_parameter("Car.Bf", cfg.car.Bf);
    node->get_parameter("Car.Br", cfg.car.Br);
    node->get_parameter("Car.Cf", cfg.car.Cf);
    node->get_parameter("Car.Cr", cfg.car.Cr);
    node->get_parameter("Car.Df", cfg.car.Df);
    node->get_parameter("Car.Dr", cfg.car.Dr);
    
    // MPC parameters
    node->get_parameter("MPC.n_horizon",  cfg.mpc.n_horizon);
    node->get_parameter("MPC.n_planning", cfg.mpc.n_planning);
    node->get_parameter("MPC.n_states",   cfg.mpc.n_states);
    node->get_parameter("MPC.n_controls", cfg.mpc.n_controls);
    node->get_parameter("MPC.Ts",         cfg.mpc.Ts);
    node->get_parameter("MPC.Disc",       cfg.mpc.disc);
    node->get_parameter("MPC.latency",    cfg.mpc.latency);

    node->get_parameter("MPC.verbose",    cfg.mpc.verbose);
    node->get_parameter("MPC.save_debug", cfg.mpc.save_debug);

    node->get_parameter("MPC.q_lat",      cfg.mpc.q_lat);
    node->get_parameter("MPC.q_vy",       cfg.mpc.q_vy);
    node->get_parameter("MPC.q_phi",      cfg.mpc.q_phi);
    node->get_parameter("MPC.q_r",        cfg.mpc.q_r);
    node->get_parameter("MPC.q_delta",    cfg.mpc.q_delta);
    node->get_parameter("MPC.q_delta_dot",cfg.mpc.q_delta_dot);

    node->get_parameter("MPC.p_lat",      cfg.mpc.p_lat);
    node->get_parameter("MPC.p_vy",       cfg.mpc.p_vy);
    node->get_parameter("MPC.p_phi",      cfg.mpc.p_phi);
    node->get_parameter("MPC.p_r",        cfg.mpc.p_r);
    node->get_parameter("MPC.p_delta",    cfg.mpc.p_delta);
    node->get_parameter("MPC.p_delta_dot",cfg.mpc.p_delta_dot);

    node->get_parameter("MPC.q_lat_init", cfg.mpc.q_lat_init);
    node->get_parameter("MPC.q_vy_init", cfg.mpc.q_lat_init);
    node->get_parameter("MPC.q_phi_init", cfg.mpc.q_phi_init);
    node->get_parameter("MPC.q_delta_init", cfg.mpc.q_delta_init);
    node->get_parameter("MPC.q_delta_dot_init", cfg.mpc.q_delta_dot_init);

    node->get_parameter("MPC.r_st",       cfg.mpc.r_st);
    node->get_parameter("MPC.r_st_init",  cfg.mpc.r_st);

    node->get_parameter("MPC.vx_to_finish_init", cfg.mpc.vx_to_finish_init);

    // Topics
    node->get_parameter("Topics.InState",     cfg.topics.in_state);
    node->get_parameter("Topics.InPlanner",   cfg.topics.in_planner);
    node->get_parameter("Topics.InVelocities",cfg.topics.in_velocity);
    node->get_parameter("Topics.OutSteering", cfg.topics.out_steering);
    node->get_parameter("Topics.OutDuration", cfg.topics.out_duration);

    node->get_parameter("Topics.Vis.PredictedSteering", cfg.topics.vis.predictedSteering);
    node->get_parameter("Topics.Vis.PredictedPath",     cfg.topics.vis.predictedPath);
    node->get_parameter("Topics.Vis.PredictedHeading",  cfg.topics.vis.predictedHeading);
    node->get_parameter("Topics.Vis.ReferencePath",     cfg.topics.vis.referencePath);
    node->get_parameter("Topics.Debug.OutLateralError", cfg.topics.debug.out_lateral_error);
    node->get_parameter("Topics.Debug.OutHeadingError", cfg.topics.debug.out_heading_error);
}

// From ROS
State stateMsg(const cat_msgs::msg::CarState::SharedPtr& msg){

    State car_state;

    car_state.x = msg->odom.position.x;
    car_state.y = msg->odom.position.y;
    car_state.psi = msg->odom.heading;
    car_state.vx = msg->odom.velocity.x;
    car_state.vy = msg->odom.velocity.y;
    car_state.r = msg->odom.velocity.w;
    car_state.delta = msg->steering;

    return car_state;
}

std::vector<TrajectoryPoint> planMsg(const cat_msgs::msg::ObjectiveArrayCurv::SharedPtr& msg) {
    Config& cfg = Config::getInstance();

    std::vector<TrajectoryPoint> planner;

    for(size_t i = 0; i < msg->objectives.size(); ++i) {
        TrajectoryPoint point;
        point.x = msg->objectives[i].x;
        point.y = msg->objectives[i].y;
        point.s = msg->objectives[i].s;
        point.k = (fabs(msg->objectives[i].k) < 1e-7) ? 1e-7 : msg->objectives[i].k; // avoid absolut zeros
        point.vx = msg->objectives[i].vx;
        point.l = msg->objectives[i].l;
        point.r = msg->objectives[i].r;
        point.ax = msg->objectives[i].ax;
        point.w = msg->objectives[i].w;

        planner.push_back(point);
    }

    return planner;
}

Eigen::VectorXd velsMsg(const cat_msgs::msg::CarVelocityArray::SharedPtr& msg) {
    Config& cfg = Config::getInstance();
    const size_t n = msg->velocities.size();

    if (n < cfg.mpc.n_horizon) {
        if (cfg.mpc.verbose)
            RCLCPP_WARN(rclcpp::get_logger("fuzzy_mpc"), "MPC: Velocity profile too short!");
    }

    Eigen::VectorXd vels(n);
    for (size_t i = 0; i < n; i++)
        vels(i) = msg->velocities[i].x;

    return vels;
}

// To ROS
inline cat_msgs::msg::CarCommands steerMsg(const double& steering_cmd){
    cat_msgs::msg::CarCommands cmd;
    cmd.header.stamp = rclcpp::Clock().now();
    cmd.steering = steering_cmd;
    return cmd;
}

visualization_msgs::msg::MarkerArray headingMsg(const Eigen::MatrixXd &state){
    
    size_t id = 0;

    // Init message:
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markerArray;
    
    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = "global";
    
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    
    tf2::Quaternion q;

    for (unsigned i = 0; i < state.rows(); i++) {
        
        marker.ns = "heading";
        marker.id = id++;
        
        q.setRPY( 0, 0, state(i, 2) );
        
        marker.pose.position.x = state(i, 0);
        marker.pose.position.y = state(i, 1);
        marker.pose.position.z = 0.25;
        
        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        
        marker.scale.x = 0.3;   // shaft length
        marker.scale.y = 0.05;  // head diameter
        marker.scale.z = 0.05;  // head length

        marker.color.r = 0.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}

nav_msgs::msg::Path localPathMsg(
    const std::vector<State> &local_path,
    const State &local_state)
{
    nav_msgs::msg::Path path_msg;

    path_msg.header.stamp = rclcpp::Clock().now();
    path_msg.header.frame_id = "base_link";

    for (const auto& p : local_path)
    {
        // Reuse your transformation
        State p_bl = global_to_local_state(p, local_state);

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;

        pose.pose.position.x = p_bl.x;
        pose.pose.position.y = p_bl.y;
        pose.pose.position.z = 0.0;

        path_msg.poses.push_back(pose);
    }
    std::cout << "Path message created with " << path_msg.poses.size() << " poses." << std::endl;
    return path_msg;
}

visualization_msgs::msg::MarkerArray steeringMsg(const Eigen::MatrixXd &state){
    size_t id = 0;

    // Init message:
    visualization_msgs::msg::Marker marker;
    visualization_msgs::msg::MarkerArray markerArray;

    marker.header.stamp = rclcpp::Clock().now();
    marker.header.frame_id = "global";

    marker.action= visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;

    tf2::Quaternion q;
    
    for (unsigned i = 1; i < state.rows()-1; i++) {

        marker.ns= "steering";
        marker.id = id++;

        q.setRPY(0, 0, state(i, 3)*3 + state(i, 2) );

        marker.pose.position.x = state(i, 0);
        marker.pose.position.y = state(i, 1);
        marker.pose.position.z = 0.50;

        marker.pose.orientation.x = q[0];
        marker.pose.orientation.y = q[1];
        marker.pose.orientation.z = q[2];
        marker.pose.orientation.w = q[3];
        
        marker.scale.x = 0.3;   // shaft length
        marker.scale.y = 0.05;  // head diameter
        marker.scale.z = 0.05;  // head length

        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        markerArray.markers.push_back(marker);
    }
    return markerArray;
}
