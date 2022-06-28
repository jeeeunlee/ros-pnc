#pragma once

#include <stdio.h>
#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/robot_system.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <magneto_pnc/magneto_command_api.hpp>

class Constraint {
   public:
    Constraint(RobotSystem* _robot, int _link_idx);
    ~Constraint() {}


    bool isUpdated() {return b_updated_; }

    int getLinkIdx() { return link_idx_; }
    int getDim() { return dim_constraint_; }
    int getPositionDim() { return dim_position_constraint_; }
    int getJointDim() { return dim_joint_constraint_; }

    void getJacobian(Eigen::MatrixXd& Jcs) { Jcs = Jcs_; }
    void getPositionJacobian(Eigen::MatrixXd& Jcs) { 
        Jcs = Jcs_.block(dim_constraint_-dim_position_constraint_, 0,
                        dim_position_constraint_, dim_joint_constraint_); }
    void getPosition(Eigen::VectorXd& Pcs) { Pcs = Pcs_; }
    void getPositionError(Eigen::VectorXd& Pcs_err) { Pcs_err = Pcs_err_; }
    
    void update();
    void setDesired(const POSE_DATA& pos_del);
      
    void printInfos() {
        pnc_utils::pretty_print(pos_des_, std::cout, "pos err");
        pnc_utils::pretty_print(Jcs_, std::cout, "task jacobian");
    }

   protected:
    void _updatePositionError();
    void _updatePosition();
    void _updateJacobian();

   protected:
    RobotSystem* robot_;

    bool b_updated_;
    int link_idx_;  

    int dim_constraint_;
    int dim_position_constraint_;
    int dim_joint_constraint_;


    Eigen::MatrixXd Jcs_;
    Eigen::VectorXd Pcs_; // position
    Eigen::VectorXd Pcs_err_; // des - act
    Eigen::VectorXd Jdotqdot_;

    Eigen::VectorXd pos_ini_;
    Eigen::Quaternion<double> ori_ini_; // R_wb

    Eigen::VectorXd pos_des_;
    Eigen::Quaternion<double> ori_des_;

};