
#include "project_utils/integrator_class.hpp"
StateVector IntegratorClass::rk4Integrator(const StateVector& x, const InputVector& u,double ts) const
{

    StateVector k1 = dynamics_(x, u);
    StateVector k2 = dynamics_(x + (ts/2.)*k1,u);
    StateVector k3 = dynamics_(x + (ts/2.)*k2,u);
    StateVector k4 = dynamics_(x + (ts/2.)*k3,u);
    StateVector x_next = x + ts*(k1/6.+k2/3.+k3/3.+k4/6.);
    return x_next;
}


StateVector IntegratorClass::efIntegrator(const StateVector& x, const InputVector& u,double ts) const {
    StateVector k1 = dynamics_(x, u);
    StateVector x_next = x + ts*(k1);
    return x_next;
}

void IntegratorClass::simNextState( const InputVector& u,double ts)const{

    StateVector x_next = this->getState_();
    const int integration_steps = (int)(ts/this->fine_time_step_);
    for(int i=0;i<integration_steps;i++){
        x_next=this->rk4Integrator(x_next,u,fine_time_step_);
        }
    //RCLCPP_INFO(this->get_logger(), "rk4 called");
    this->setState_(x_next);
    this->setInput_(u);
}
