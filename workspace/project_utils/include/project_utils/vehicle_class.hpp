
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <cmath>
//typedef Eigen::Matrix<double,NX,1> StateVector;
typedef Eigen::VectorXd InputVector;
typedef Eigen::VectorXd StateVector;
struct StateStruct{
    double x;
    double y;
    double yaw;
    double vx;
    double sf;
};

struct InputStruct{
        double sv;
        double acc;
};


class VehicleClass{
    private:
        int NX;
        int NU;
        float T_fwd;
        double default_x_pos;
        double default_y_pos;
        double default_yaw;
        double default_vx;
        double default_sf;
        double default_sv;
        double default_acc;
        double wheelbase;
        InputStruct input;
        InputVector inputvector;
        StateStruct state;
        StateVector statevector;
    public:
        explicit VehicleClass(rclcpp::Node::SharedPtr node);
        void reset();
        void setState(const StateVector &);
        void setInput(const InputVector &);
        const StateVector& getState() const;
        const StateVector& getInput() const;
        StateVector StateToVector(const StateStruct & ) const;
        StateStruct VectorToState(const StateVector &) const;
        InputVector InputToVector(const InputStruct &) const;
        InputStruct VectorToInput(const InputVector &) const;
        StateVector xdot(const StateVector & , const InputVector &) const;
};






