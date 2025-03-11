#include "vehicle_interface_class.hpp"



VehicleInterfaceNode::VehicleInterfaceNode():Node("vehicle_interface_node"){

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    state_update_timer_ = this->create_wall_timer(0.005s,[this](){this->state_update_timer_callback();});
    vehicle_ = std::make_shared<VehicleClass>(this->shared_from_this()); // shared pointer to itself , make sure not to create a new reference from (this) pointer
    integrator_ = std::make_shared<IntegratorClass>(vehicle->xdot,vehicle->getState,vehicle->setState,vehicle->setInput);
    state_publisher_  =  this->create_publisher<project_utils::msg::EigenVector>("/ego_state",10);
    state_pub_timer_ = this->create_wall_timer(0.01s,[this](){this->state_pub_timer_callback();});
    control_subsriber_ = this->create_subscriber<project_utils::msg::EigenVector>("/ego_command",10,
                            [this](const project_utils::msg::EigenVector& msg ){this->control_sub_callback(msg)};);

}


void VehicleInterfaceNode::state_pub_timer_callback() {
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");

    // Example state publishing (optional)
    project_utils::msg::EigenVector msg;
    StateVector state = vehicle->getState(); // Assuming getState() returns StateVector
    // Eigen::Map<const Eigen::VectorXd> map(eigenVec.data(), eigenVec.size());

    // // Convert Eigen::Map to std::vector<double>
    // std::vector<double> stdVec(map.data(), map.data() + map.size());

    msg.data = std::vector<double>(state.data(), state.data() + state.size());
    
    state_publisher_->publish(msg);
}


void VehicleInterfaceNode::state_update_timer_callback(){
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    integrator_->simNextState(this->control_ref,0.005)

}

void VehicleInterfaceNode::control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg){
    auto temp = msg.data;
    control_ref = Eigen::Map<Eigen::VectorXd>(temp.data(),temp.size());
}