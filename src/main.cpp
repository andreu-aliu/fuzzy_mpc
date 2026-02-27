#define EIGEN_DONT_PARALLELIZE

#include <rclcpp/rclcpp.hpp>
#include <signal.h>

// Modules
#include "utils/Config.hpp"
#include "utils/ROSutils.hpp"
#include "mpc.hpp"

#include <as_lib/utils/Profiler.hpp>



class Manager : public rclcpp::Node{
  private:

    // Objects
    MPC mpc;

    // Output variables
    double steering;
    MatrixXd actual_state;
    MatrixXd predicted_state;

    // Publishers
    rclcpp::Publisher<cat_msgs::msg::CarCommands>::SharedPtr pubSteering;
    // rclcpp::Publisher<cat_msgs::msg::Float32Stamped>::SharedPtr pubLateralError, pubHeadingsssError, pubYawRateRef;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPredictedSteering, pubPredictedHeading;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPredictedPath, pubActualPath;

    // Subscribers
    rclcpp::Subscription<cat_msgs::msg::CarState>::SharedPtr subState;
    rclcpp::Subscription<cat_msgs::msg::ObjectiveArrayCurv>::SharedPtr subPlanner;
    rclcpp::Subscription<cat_msgs::msg::CarVelocityArray>::SharedPtr subVel;

    // Reconfigure handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Flags
    bool planner_recieved = false;
    bool dynamic_recieved = false;
    bool vels_flag = false;
    bool init_state = true; // at the begining we use different initialization weights
  
  public:
    Manager(): Node("fuzzy_mpc", 
        rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)){


        std::cout << "LTV MPC Node Started" << std::endl;
        Config& cfg = Config::getInstance();
        fill_config(cfg, this);
        std::cout << "Configuration Loaded" << std::endl;

        mpc.initialize();
        // Reconfigure parameters
        param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &params)->rcl_interfaces::msg::SetParametersResult{
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            Config& cfg = Config::getInstance();

            for (const auto &param : params) {
                const std::string &name = param.get_name();

                if (name == "MPC.latency")        cfg.mpc.latency = param.as_double();
                else if (name == "MPC.q_lat")     cfg.mpc.q_lat = param.as_double();
                else if (name == "MPC.q_vy")      cfg.mpc.q_vy = param.as_double();
                else if (name == "MPC.q_phi")     cfg.mpc.q_phi = param.as_double();
                else if (name == "MPC.q_r")       cfg.mpc.q_r = param.as_double();
                else if (name == "MPC.q_delta")   cfg.mpc.q_delta = param.as_double();
                else if (name == "MPC.r_delta")   cfg.mpc.r_delta = param.as_double();
                else if (name == "MPC.p_lat")     cfg.mpc.p_lat = param.as_double();
                else if (name == "MPC.p_vy")      cfg.mpc.p_vy = param.as_double();
                else if (name == "MPC.p_phi")     cfg.mpc.p_phi = param.as_double();
                else if (name == "MPC.p_r")       cfg.mpc.p_r = param.as_double();
                else if (name == "MPC.p_delta")   cfg.mpc.p_delta = param.as_double();
                else
                    continue; // ignore unrelated parameters
            }

            if (cfg.mpc.verbose)
                RCLCPP_INFO(this->get_logger(), "Dynamic parameters updated, recalculating weights");

            mpc.createWeights(init_state);
            dynamic_recieved = true;

            return result;
        });

        // Subscribers
        subState = create_subscription<cat_msgs::msg::CarState>(
            cfg.topics.in_state, 1, 
            [this](const cat_msgs::msg::CarState::SharedPtr msg){
            stateCallback(msg);
        });

        subPlanner = create_subscription<cat_msgs::msg::ObjectiveArrayCurv>(
            cfg.topics.in_planner, 1,
            [this](const cat_msgs::msg::ObjectiveArrayCurv::SharedPtr msg){
            plannerCallback(msg);
        });
        
        subVel = create_subscription<cat_msgs::msg::CarVelocityArray>(
            cfg.topics.in_velocity, 1,
            [this](const cat_msgs::msg::CarVelocityArray::SharedPtr msg){
            velsCallback(msg);
        });

        // Main publishers
        pubSteering = this->create_publisher<cat_msgs::msg::CarCommands>(cfg.topics.out_steering, 1);

        // Debug publishers
        // pubLateralError = this->create_publisher<cat_msgs::msg::Float32Stamped>(std::string(this->get_name()) + cfg.topics.debug.out_lateral_error, 1);
        // pubHeadingError = this->create_publisher<cat_msgs::msg::Float32Stamped>(std::string(this->get_name()) + cfg.topics.debug.out_heading_error, 1);
        // pubYawRateRef = this->create_publisher<cat_msgs::msg::Float32Stamped>(std::string(this->get_name()) + cfg.topics.debug.out_yaw_rate_ref, 1);

        // Visualization publishers
        pubPredictedHeading = this->create_publisher<visualization_msgs::msg::MarkerArray>(std::string(this->get_name()) + cfg.topics.vis.predictedHeading, 1);
        pubPredictedPath = this->create_publisher<nav_msgs::msg::Path>(std::string(this->get_name()) + cfg.topics.vis.predictedPath, 1);
        pubPredictedSteering = this->create_publisher<visualization_msgs::msg::MarkerArray>(std::string(this->get_name()) + cfg.topics.vis.predictedSteering, 1);
        pubActualPath = this->create_publisher<nav_msgs::msg::Path>(std::string(this->get_name()) + cfg.topics.vis.actualPath, 1);
    }

    ~Manager(){}

    // Callbacks
    void stateCallback(const cat_msgs::msg::CarState::SharedPtr& msg){
        PROFC_NODE_

        Config& cfg = Config::getInstance();

        if(cfg.mpc.verbose) std::cout << "State callback" << std::endl;
        
        mpc.setState(stateMsg(msg));        // state = [ x, y, theta, vx, vy, w, delta ]

        if(planner_recieved){
            if(cfg.mpc.verbose) std::cout << "Planner received, running MPC" << std::endl;
            
            // Check if init state and recompute weights if so
            if(init_state && msg->odom.velocity.x > cfg.mpc.vx_to_finish_init){
                init_state = false;
                mpc.createWeights(init_state);
            }

            mpc.findReferences();

            mpc.createModelMatrices();

            mpc.solve();

            // Publish steering command
            pubSteering->publish(steerMsg(mpc.getSteeringCmd()));

            // Publish visualization
            predicted_state = mpc.getPredictedStates();
            actual_state = mpc.getActualState();

            pubPredictedHeading->publish(headingMsg(predicted_state));
            pubPredictedPath->publish(pathMsg(predicted_state));
            pubPredictedSteering->publish(steeringMsg(predicted_state));
            pubActualPath->publish(pathMsg(actual_state));

        }else{
            if(cfg.mpc.verbose)
                RCLCPP_WARN(get_logger(), "MPC: No planner received yet");
        }

        // Print or publish computational times
        // PROFC_PRINT()
    }

    void plannerCallback(const cat_msgs::msg::ObjectiveArrayCurv::SharedPtr& msg){
        PROFC_NODE_
        Config& cfg = Config::getInstance();
        Eigen::MatrixXd trajectory = planMsg(msg);

        if(trajectory.size() < cfg.mpc.n_planning){
            RCLCPP_ERROR(get_logger(), "LTV MPC: Too short, Planner");
            return;
        }

        mpc.setPlanner(planMsg(msg));
        planner_recieved = true;
        if(cfg.mpc.verbose)
            RCLCPP_WARN(get_logger(), "MPC: Planner received");
    }

    void velsCallback(const cat_msgs::msg::CarVelocityArray::SharedPtr& msg){
        mpc.setVels(velsMsg(msg));
        vels_flag = true;
    }
};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Manager>();

    rclcpp::on_shutdown([node]() {
        RCLCPP_ERROR(rclcpp::get_logger("ltv_mpc"),"Shutting down node...");
        rclcpp::shutdown();
    });

    if(Config::getInstance().profile)
    {   // Set the parameter profiler to true to enable profiling.
        // You need the profiler viewer (workspace/libs/as-lib/misc/profiler_dashboard.py)
        PROFC_INSTALL(node);
    }
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
