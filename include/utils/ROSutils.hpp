#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen3/Eigen/Dense>
#include "utils/Config.hpp"

#include "cat_msgs/msg/car_state.hpp"
#include "cat_msgs/msg/objective_array_curv.hpp"
#include "cat_msgs/msg/car_velocity_array.hpp"
#include "cat_msgs/msg/car_commands.hpp"

#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/path.hpp"


inline void fill_config(Config& cfg, rclcpp::Node* node)
{
    node->get_parameter("profile", cfg.profile);

    // Car parameters
    node->get_parameter_or("Car.m",  cfg.car.m,  1575.0);
    node->get_parameter_or("Car.I",  cfg.car.I,  2875.0);
    node->get_parameter_or("Car.Lf", cfg.car.Lf, 1.2);
    node->get_parameter_or("Car.Lr", cfg.car.Lr, 1.6);
    node->get_parameter_or("Car.Bf", cfg.car.Bf, 0.55);
    node->get_parameter_or("Car.Br", cfg.car.Br, 0.55);
    node->get_parameter_or("Car.Cf", cfg.car.Cf, 19000.0);
    node->get_parameter_or("Car.Cr", cfg.car.Cr, 33000.0);
    node->get_parameter_or("Car.Df", cfg.car.Df, 0.8);
    node->get_parameter_or("Car.Dr", cfg.car.Dr, 0.8);
    
    // MPC parameters
    node->get_parameter_or("MPC.n_horizon",  cfg.mpc.n_horizon,  10);
    node->get_parameter_or("MPC.n_planning", cfg.mpc.n_planning, 20);
    node->get_parameter_or("MPC.n_states",   cfg.mpc.n_states,   5);
    node->get_parameter_or("MPC.n_controls", cfg.mpc.n_controls, 1);
    node->get_parameter_or("MPC.Ts",         cfg.mpc.Ts,         0.1);
    node->get_parameter_or("MPC.Disc",       cfg.mpc.disc,       0.0);
    node->get_parameter_or("MPC.latency",    cfg.mpc.latency,    0.0);

    node->get_parameter_or("MPC.verbose",    cfg.mpc.verbose,    false);
    node->get_parameter("MPC.debug_path",    cfg.mpc.debug_path);
    node->get_parameter_or("MPC.save_debug", cfg.mpc.save_debug, false);

    node->get_parameter_or("MPC.q_lat",      cfg.mpc.q_lat,      1.0);
    node->get_parameter_or("MPC.q_vy",       cfg.mpc.q_vy,       1.0);
    node->get_parameter_or("MPC.q_phi",      cfg.mpc.q_phi,      1.0);
    node->get_parameter_or("MPC.q_r",        cfg.mpc.q_r,        1.0);
    node->get_parameter_or("MPC.q_delta",    cfg.mpc.q_delta,    1.0);
    node->get_parameter_or("MPC.r_delta",    cfg.mpc.r_delta,    1.0);
    node->get_parameter_or("MPC.p_lat",      cfg.mpc.p_lat,      10.0);
    node->get_parameter_or("MPC.p_vy",       cfg.mpc.p_vy,       10.0);
    node->get_parameter_or("MPC.p_phi",      cfg.mpc.p_phi,      10.0);
    node->get_parameter_or("MPC.p_r",        cfg.mpc.p_r,        10.0);
    node->get_parameter_or("MPC.p_delta",    cfg.mpc.p_delta,    10.0);
    node->get_parameter_or("MPC.q_lat_init", cfg.mpc.q_lat_init, 100.0);
    node->get_parameter_or("MPC.q_phi_init", cfg.mpc.q_phi_init, 100.0);
    node->get_parameter_or("MPC.q_delta_init", cfg.mpc.q_delta_init, 100.0);
    node->get_parameter_or("MPC.r_delta_init", cfg.mpc.r_delta_init, 100.0);
    node->get_parameter_or("MPC.vx_to_finish_init", cfg.mpc.vx_to_finish_init, 5.0);

    // Topics
    node->get_parameter_or("Topics.InState",     cfg.topics.in_state,     std::string("/AS/C/state"));
    node->get_parameter_or("Topics.InPlanner",   cfg.topics.in_planner,   std::string("/AS/C/trajectory/locator"));
    node->get_parameter_or("Topics.InVelocities",cfg.topics.in_velocity,  std::string("/AS/C/pid/velocity"));
    node->get_parameter_or("Topics.OutSteering", cfg.topics.out_steering, std::string("/AS/C/steering"));
    node->get_parameter_or("Topics.OutDuration", cfg.topics.out_duration, std::string("/AS/C/ltv_mpc/duration"));

    node->get_parameter_or("Topics.Vis.PredictedSteering", cfg.topics.vis.predictedSteering, std::string("/AS/C/mpc/vis/predicted/steering"));
    node->get_parameter_or("Topics.Vis.PredictedPath",     cfg.topics.vis.predictedPath,     std::string("/AS/C/mpc/vis/predicted/path"));
    node->get_parameter_or("Topics.Vis.PredictedHeading",  cfg.topics.vis.predictedHeading,  std::string("/AS/C/mpc/vis/predicted/heading"));
    node->get_parameter_or("Topics.Vis.ActualPath",        cfg.topics.vis.actualPath,        std::string("/AS/C/mpc/vis/actual/path"));
    node->get_parameter_or("Topics.Debug.OutLateralError", cfg.topics.debug.out_lateral_error, std::string("/AS/C/mpc/debug/lateralError"));
    node->get_parameter_or("Topics.Debug.OutHeadingError", cfg.topics.debug.out_heading_error, std::string("/AS/C/mpc/debug/headingError"));
}

// From ROS
Eigen::VectorXd stateMsg(const cat_msgs::msg::CarState::SharedPtr& msg){
    // car_state: [x, y, vx, vy, phi, r, delta]
    Eigen::VectorXd car_state(7);
    car_state << msg->odom.position.x, msg->odom.position.y, msg->odom.heading,
                 msg->odom.velocity.x, msg->odom.velocity.y, msg->odom.velocity.w, 
                 msg->steering;

    return car_state;
}

Eigen::MatrixXd planMsg(const cat_msgs::msg::ObjectiveArrayCurv::SharedPtr& msg) {
    Config& cfg = Config::getInstance();
    Eigen::MatrixXd planner(msg->objectives.size(), 9);

    for (size_t i = 0; i < msg->objectives.size(); ++i) {
        planner(i, 0) = msg->objectives[i].x;
        planner(i, 1) = msg->objectives[i].y;
        planner(i, 2) = msg->objectives[i].s;
        planner(i, 3) = (fabs(msg->objectives[i].k) < 1e-7) ? 1e-7 : msg->objectives[i].k; // avoid absolut zeros
        planner(i, 4) = msg->objectives[i].vx;
        planner(i, 5) = msg->objectives[i].l;
        planner(i, 6) = msg->objectives[i].r;
        planner(i, 7) = msg->objectives[i].ax;
        planner(i, 8) = msg->objectives[i].w;
    }

    return planner;
}

Eigen::VectorXd velsMsg(const cat_msgs::msg::CarVelocityArray::SharedPtr& msg) {
    Config& cfg = Config::getInstance();
    const size_t n = msg->velocities.size();

    if (n < cfg.mpc.n_horizon) {
        if (cfg.mpc.verbose)
            RCLCPP_WARN(rclcpp::get_logger("ltv_mpc"), "MPC: Velocity profile too short!");
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

nav_msgs::msg::Path pathMsg(const Eigen::MatrixXd &state){

    nav_msgs::msg::Path pathMsg;
	geometry_msgs::msg::PoseStamped pose;

    pathMsg.header.stamp    = rclcpp::Clock().now();
    pathMsg.header.frame_id = "global";

    for (unsigned i = 0; i < state.rows(); i++) {

        pose.pose.position.x = state(i, 0);
        pose.pose.position.y = state(i, 1);

        pathMsg.poses.push_back(pose);
    }
    return pathMsg;
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
