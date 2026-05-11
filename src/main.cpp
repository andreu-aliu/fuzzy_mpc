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
    MPC evaluator;
    Trajectory global_trajectory;
    State base_state;
    States predicted_states;
    States previous_states;
    Controls previous_controls;
    Controls optimal_controls;
    Control applied_control;

    // For evaluator
    double model_error;
    size_t iters = 0;
    std::deque<State> state_history;
    std::deque<Control> control_history;

    // Publishers
    rclcpp::Publisher<cat_msgs::msg::CarCommands>::SharedPtr pubSteering;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubPredictedSteering, pubPredictedHeading;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubReferencePath, pubPredictedPath;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pubModelError;

    // Subscribers
    rclcpp::Subscription<cat_msgs::msg::CarState>::SharedPtr subState;
    rclcpp::Subscription<cat_msgs::msg::ObjectiveArrayCurv>::SharedPtr subPlanner;

    // Reconfigure handle
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Flags
    bool planner_recieved = false;
    bool dynamic_recieved = false;
    bool first_iteration = true;
  
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
        evaluator.initialize();

        // TODO: Dynamic reconfigure

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

        // Publishers
        pubSteering = this->create_publisher<cat_msgs::msg::CarCommands>(cfg.topics.out_steering, 1);
        pubModelError = this->create_publisher<std_msgs::msg::Float64>(cfg.topics.out_model_error, 1);

        pubPredictedHeading = this->create_publisher<visualization_msgs::msg::MarkerArray>(std::string(this->get_name()) + cfg.topics.vis.predictedHeading, 1);
        pubPredictedSteering = this->create_publisher<visualization_msgs::msg::MarkerArray>(std::string(this->get_name()) + cfg.topics.vis.predictedSteering, 1);
        pubPredictedPath = this->create_publisher<nav_msgs::msg::Path>(std::string(this->get_name()) + cfg.topics.vis.predictedPath, 1);
        pubReferencePath = this->create_publisher<nav_msgs::msg::Path>(std::string(this->get_name()) + cfg.topics.vis.referencePath, 1);
    }

    ~Manager(){}

    // Callbacks
    void stateCallback(const cat_msgs::msg::CarState::SharedPtr& msg){
        PROFC_NODE_

        Config& cfg = Config::getInstance();

        if(cfg.verbose) std::cout << "--------  State callback   ------------------------------" << std::endl;
        
        State global_state = stateMsg(msg);     

        if(planner_recieved){
            if(cfg.verbose) std::cout << "Planner received, running MPC" << std::endl;
            iters ++;
            
            // Get reference in global coordinates
            std::vector<State> global_ref = build_reference(global_trajectory, global_state, base_state, cfg.mpc.Ts, cfg.mpc.n_horizon);
            if(!is_valid(global_ref)){
                RCLCPP_ERROR(get_logger(), "MPC: Invalid global reference");
                return;
            }

            // Convert reference to local coordinates (first point of the trajectory)
            State local_state = global_to_local_state(global_state, base_state);
            std::vector<State> local_ref(cfg.mpc.n_horizon);
            for (size_t i = 0; i < global_ref.size(); ++i){
                local_ref[i] = global_to_local_state(global_ref[i], base_state);
            }
            if(!is_valid(local_ref)){
                RCLCPP_ERROR(get_logger(), "MPC: Invalid local state or reference");
                return;
            }

            // Publish reference visualization
            pubReferencePath->publish(localPathMsg(local_ref, local_state));

            // MPC solution
            if(first_iteration){
                predicted_states = local_ref;
                optimal_controls = std::vector<Control>(cfg.mpc.n_horizon, Control{0.0, 0.0});
                applied_control = optimal_controls[0];
                first_iteration = false;
            }
            previous_states = predicted_states;
            previous_controls = optimal_controls;
            mpc.compute_mpc(local_state, applied_control, local_ref, previous_states, previous_controls, predicted_states, optimal_controls);

            if(!is_valid(predicted_states) || !is_valid(optimal_controls)){                
                first_iteration = true;
                RCLCPP_ERROR(get_logger(), "MPC: Invalid solution");
            }

            // Publish commands
            applied_control = optimal_controls[2];  // TODO: Use parameter
            double steering = applied_control.steering;
            pubSteering->publish(steerMsg(steering));

            // Publish prediction visualization
            pubPredictedPath->publish(localPathMsg(predicted_states, local_state));

            // Update history
            state_history.push_back(global_state);
            control_history.push_back(applied_control);

            if(state_history.size() > cfg.mpc.n_evaluation)
                state_history.pop_front();

            if(control_history.size() > cfg.mpc.n_evaluation)
                control_history.pop_front();

            // Compute model error
            if (state_history.size() == cfg.mpc.n_evaluation &&
                control_history.size() == cfg.mpc.n_evaluation){
                PROFC_NODE("Compute model error")

                std::vector<State> state_vect(state_history.begin(), state_history.end());
                std::vector<Control> control_vect(control_history.begin(), control_history.end());

                // Convert to local reference
                std::vector<State> local_state_vect(cfg.mpc.n_horizon);
                for (size_t i = 0; i < state_vect.size(); ++i){
                    local_state_vect[i] = global_to_local_state(state_vect[i], state_vect[0]);
                }

                model_error = evaluator.compute_prediction(local_state_vect[0], control_vect, local_state_vect);

                pubModelError->publish(floatMsg(model_error));
            }
            
        }else{
            if(cfg.verbose)
                RCLCPP_WARN(get_logger(), "MPC: No planner received yet");
        }
    }

    void plannerCallback(const cat_msgs::msg::ObjectiveArrayCurv::SharedPtr& msg){
        PROFC_NODE_
        Config& cfg = Config::getInstance();

        if(cfg.verbose) std::cout << "Planner callback" << std::endl;

        global_trajectory = planMsg(msg);

        if(global_trajectory.size() < 2){
            RCLCPP_ERROR(get_logger(), "LTV MPC: Too short, Planner");
            return;
        }

        planner_recieved = true;
    }

    // TODO: Dynamic reconfigure callback

    // Aux functions
    bool is_valid(const std::vector<State> &states){
        for(const auto& s : states){
            if(!std::isfinite(s.x) || !std::isfinite(s.y) || !std::isfinite(s.psi) || !std::isfinite(s.vx) || !std::isfinite(s.vy) || !std::isfinite(s.r) || !std::isfinite(s.delta) || !std::isfinite(s.delta_dot)){
                return false;
            }
        }
        return true;
    }

    bool is_valid(const std::vector<Control> &controls){
        for(const auto& c : controls){
            if(!std::isfinite(c.steering) || !std::isfinite(c.mz)){
                return false;
            }
        }
        return true;
    }

};


int main(int argc, char **argv){
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Manager>();

    rclcpp::on_shutdown([node]() {
        RCLCPP_ERROR(rclcpp::get_logger("fuzzy_mpc"),"Shutting down node...");
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
