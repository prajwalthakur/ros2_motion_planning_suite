#include "project_utils/vehicle_class.hpp"
VehicleClass::VehicleClass(rclcpp::Node::SharedPtr node){
    // node->declare_parameter("NX", 5);
    // node->declare_parameter("NU", 2);
    // node->declare_parameter("default_x_pos", 0.0);
    // node->declare_parameter("default_y_pos", 0.0);
    // node->declare_parameter("default_yaw", 0.0);
    // node->declare_parameter("default_vx", 0.0);
    // node->declare_parameter("default_sf", 0.0);
    // node->declare_parameter("default_acc",0.0);
    // node->declare_parameter("default_sv",0.0);
    // node->declare_parameter("wheelbase",0.0);

    NX = node->get_parameter("NX").as_int();
    NU = node->get_parameter("NU").as_int();
    default_x_pos = node->get_parameter("default_x_pos").as_double();
    default_y_pos = node->get_parameter("default_y_pos").as_double();
    default_yaw = node->get_parameter("default_yaw").as_double();
    default_vx = node->get_parameter("default_vx").as_double();
    default_sf = node->get_parameter("default_sf").as_double();
    default_acc = node->get_parameter("default_acc").as_double();
    default_sv = node->get_parameter("default_sv").as_double();
    wheelbase = node->get_parameter("wheelbase").as_double();
    reset();
}

void VehicleClass::reset(){

    state.x = default_x_pos;
    state.y = default_y_pos;
    state.yaw = default_yaw;
    state.vx = default_vx;
    state.sf = default_sf;
    statevector.resize(NX);
    statevector(0) = state.x;
    statevector(1) = state.y;
    statevector(2) = state.yaw;
    statevector(3) = state.vx;
    statevector(4) = state.sf;

}

void VehicleClass::setState(const StateVector & statevector){
    this->statevector = statevector; 
    this->state.x = this->statevector(0);
    this->state.y = this->statevector(1);
    this->state.yaw = this->statevector(2);
    this->state.vx = this->statevector(3);
    this->state.sf = this->statevector(4);
}
void VehicleClass::setInput(const InputVector & input_vector){
    this->inputvector = input_vector;
    this->input.acc = this->inputvector(0);
}

const StateVector& VehicleClass::getState() const {
    return this->statevector;
}

const StateVector& VehicleClass::getInput() const {
    return this->inputvector;
}


StateVector VehicleClass::StateToVector(const StateStruct & state_struct) const{
    StateVector state_vector;
    state_vector(0) = state_struct.x;
    state_vector(1) = state_struct.y;
    state_vector(2) = state_struct.yaw;
    state_vector(3) = state_struct.vx;
    state_vector(4) = state_struct.sf;
    return state_vector;
}

InputVector VehicleClass::InputToVector(const InputStruct & input_struct) const{
    InputVector input_vector;
    input_vector(0) = input_struct.sv;
    input_vector(1) = input_struct.acc;
    return input_vector;

}

StateStruct VehicleClass::VectorToState(const StateVector & statevector) const{
    StateStruct st;
    st.x = statevector(0);
    st.y = statevector(1);
    st.yaw = statevector(2);
    st.vx = statevector(3);
    st.sf = statevector(4);
    return st;
}


InputStruct VehicleClass::VectorToInput(const InputVector & inputvector) const{
    InputStruct inpt;
    inpt.acc = inputvector(0);
    inpt.sv = inputvector(1);
    return inpt;
}

StateVector VehicleClass::xdot(const StateVector & statevector, const InputVector & inputvector) const
{
    StateVector statevector_dot;
    statevector_dot.resize(NX);

    auto xk = this->VectorToState(statevector);
    auto uk = this->VectorToInput(inputvector);
    //xdot,ydot,yawdot,vfodt,sfdot
    statevector_dot(0) = xk.vx*std::cos(xk.yaw);
    statevector_dot(1) = xk.vx*std::sin(xk.yaw);
    statevector_dot(2) = xk.vx*std::tan(xk.sf)/wheelbase;
    statevector_dot(3) = uk.acc;
    statevector_dot(4) = uk.sv;
    return statevector_dot;
}



