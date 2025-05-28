#include "vehicle_interface/vehicle_interface_class.hpp"



VehicleInterface::VehicleInterface():Node("vehicle_interface_node",rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {

    RCLCPP_INFO(this->get_logger(),"vehicle interface node started");
    nx = this->get_parameter("NX").as_int();
    nu = this->get_parameter("NU").as_int();
    m_integration_delta_t = this->get_parameter("integration_deltaT").as_double();
    m_sim_delta_t = this->get_parameter("sim_deltaT").as_double();
    m_state_publish_delta_t =  this->get_parameter("state_publish_deltaT").as_double();
    m_default_acc = this->get_parameter("default_acc").as_double();
    m_default_sv = this->get_parameter("default_sv").as_double();
    m_control_ref = Eigen::VectorXd(nu);  
    m_control_ref<<m_default_acc,m_default_sv; // initialization
}


void VehicleInterface::state_pub_timer_callback() {
    // Timer callback logic here
    RCLCPP_INFO(this->get_logger(), " state Publisher Timer triggered");

    project_utils::msg::EigenVector msg;
    StateVector state = m_vehicle->getState(); 
    msg.data = std::vector<double>(state.data(), state.data() + state.size());
    
    m_state_publisher->publish(msg);
}


void VehicleInterface::state_update_timer_callback(){
    RCLCPP_INFO(this->get_logger(), " State Update Timer triggered");
    m_integrator->simNextState(m_control_ref,m_sim_delta_t);
}

void VehicleInterface::control_sub_callback(const project_utils::msg::EigenVector::SharedPtr& msg){
    m_control_ref = Eigen::Map<Eigen::VectorXd>(msg->data.data(), msg->data.size());
    
}


/**
 * @brief VehicleInterface activation function to intialize 
 * m_vehicle, m_integrator, publisher and subscriber
 */
void VehicleInterface::on_activate() {
    // shared pointer to itself , make sure not to create a new reference from (this) pointer
    m_vehicle = std::make_shared<VehicleClass>(shared_from_this()); 
    m_integrator = std::make_shared<IntegratorClass>( 
        [this](const StateVector& state, const InputVector& input) -> StateVector {
        return m_vehicle->xdot(state, input);
        },
        [this]() -> const StateVector& {
        return m_vehicle->getState();
        },
        [this](const StateVector& state) -> void {
        return m_vehicle->setState(state);
        },
        [this](const InputVector& input) -> void {
        return m_vehicle->setInput(input);
        },
        m_integration_delta_t
    );
    
    
    m_state_publisher  =  this->create_publisher<project_utils::msg::EigenVector>("/ego_state",10);
    m_control_subscriber = this->create_subscription<project_utils::msg::EigenVector>("/ego_command",10,
                            [this](const project_utils::msg::EigenVector::SharedPtr msg ){this->control_sub_callback(msg);});    
    
    m_state_update_timer = this->create_wall_timer(std::chrono::duration<double>(m_sim_delta_t),[this](){this->state_update_timer_callback();});
    m_state_pub_timer = this->create_wall_timer(std::chrono::duration<double>(m_state_publish_delta_t),[this](){this->state_pub_timer_callback();});
    
}