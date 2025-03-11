#include "vehicle_interface/vehicle_interface_class.hpp"



VehicleInterface::VehicleInterface():Node("vehicle_interface_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    //RCLCPP_INFO(this->get_logger(), "NX: %d", static_cast<int>(this->get_parameter("NX").as_int()));

    // this->declare_parameter("NX", 5);
    // this->declare_parameter("NU", 2);
    // this->declare_parameter("integration_deltaT",0.001);
    // this->declare_parameter("sim_deltaT",0.01);
    // this->declare_parameter("state_publish_deltaT",0.05);
    NX = this->get_parameter("NX").as_int();
    NU = this->get_parameter("NU").as_int();
    integration_deltaT = this->get_parameter("integration_deltaT").as_double();
    sim_deltaT = this->get_parameter("sim_deltaT").as_double();
    state_publish_deltaT =  this->get_parameter("state_publish_deltaT").as_double();
    control_ref=Eigen::VectorXd::Zero(NU);


}


void VehicleInterface::state_pub_timer_callback() {
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");

    project_utils::msg::EigenVector msg;
    StateVector state = vehicle_->getState(); 
    msg.data = std::vector<double>(state.data(), state.data() + state.size());
    
    state_publisher_->publish(msg);
}


void VehicleInterface::state_update_timer_callback(){
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    integrator_->simNextState(control_ref,sim_deltaT);

}

void VehicleInterface::control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg){
    control_ref = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
}



void VehicleInterface::on_activate() {
    // Now `shared_from_this()` should work since the node is fully constructed
    vehicle_ = std::make_shared<VehicleClass>(shared_from_this()); // shared pointer to itself , make sure not to create a new reference from (this) pointer
    integrator_ = std::make_shared<IntegratorClass>( 
        [this](const StateVector& state, const InputVector& input) -> StateVector {
        return vehicle_->xdot(state, input);
        },
        [this]() -> const StateVector& {
        return vehicle_->getState();
        },
        [this](const StateVector& state) -> void {
        return vehicle_->setState(state);
        },
        [this](const InputVector& input) -> void {
        return vehicle_->setInput(input);
        },
        integration_deltaT
    );
    
    
    state_publisher_  =  this->create_publisher<project_utils::msg::EigenVector>("/ego_state",10);
    control_subscriber_ = this->create_subscription<project_utils::msg::EigenVector>("/ego_command",10,
                            [this](const project_utils::msg::EigenVector::SharedPtr msg ){this->control_sub_callback(msg);});    
    
    state_update_timer_ = this->create_wall_timer(std::chrono::duration<double>(sim_deltaT),[this](){this->state_update_timer_callback();});
    state_pub_timer_ = this->create_wall_timer(std::chrono::duration<double>(state_publish_deltaT),[this](){this->state_pub_timer_callback();});
    
}