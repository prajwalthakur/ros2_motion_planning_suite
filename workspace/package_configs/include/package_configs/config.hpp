#ifndef PACKAGE_CONFIGS_CONFIG_HPP_
#define PACKAGE_CONFIGS_CONFIG_HPP_
    #include <rclcpp/rclcpp.hpp>
    #include <Eigen/Dense>
    #include <cmath>
    namespace vehicle_config{
        //typedef Eigen::Matrix<double,NX,1> StateVector;
        typedef Eigen::VectorXd InputVector;
        typedef Eigen::VectorXd StateVector;
        struct StateStruct{
            double x;
            double y;
            double yaw;
            double vx;
            double sf;
            
            // double yaw_dt;
            // double beta;
        };

        struct InputStruct{
                double sv;
                double acc;
        };


        class vehicleClass{
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
                explicit vehicleClass(rclcpp::Node::SharedPtr node);
                void reset();
                void setState(const StateVector &);
                void setInput(const InputVector &);
                StateVector StateToVector(const StateStruct & ) const;
                StateStruct VectorToState(const StateVector &) const;
                InputVector InputToVector(const InputStruct &) const;
                InputStruct VectorToInput(const InputVector &)const;
                StateVector xdot(const StateVector & , const InputVector &);
        };



        // class Input{
        //     private:
        //         InputStruct input;
        //     public:
        //         Input();
        //         void reset();
        //         void set_input(const & InputVector );
        //         InputVector InputToVector();
        //         InputStruct VectorToInput();
        // };
    }

#endif //CONFIG_SIM_HPP_
