#pragma once
#include <utility>

#include <../my_utils/Configuration.h>
#include <my_utils/General/Clock.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
//#include <RobotSystem/include/CentroidModel.hpp>



class RobotSystem;
class MotionCommand;

class MagnetoStateProvider {
   public:
    static MagnetoStateProvider* getStateProvider(RobotSystem* _robot);
    ~MagnetoStateProvider() {}

    void saveCurrentData();
    void divideJoints2AnV(const Eigen::VectorXd& q_full, 
                        Eigen::VectorXd& q_a, Eigen::VectorXd& q_v);    
   
    Eigen::VectorXd getActiveJointValue();
    Eigen::VectorXd getVirtualJointValue();
    Eigen::VectorXd getActiveJointValue(const Eigen::VectorXd& q_full);
    Eigen::VectorXd getVirtualJointValue(const Eigen::VectorXd& q_full);
    Eigen::VectorXd getFullJointValue(const Eigen::VectorXd& q_a);
    Eigen::VectorXd getFullJointValue(const Eigen::VectorXd& q_a, const Eigen::VectorXd& q_v);

    void setContactPlan(const int& movingFootIdx);

    Clock clock;

    double curr_time;
    double prev_state_machine_time;
    double planning_moment;

    bool b_do_planning;

    int stance_foot;
    Eigen::Isometry3d stance_foot_iso;
    Eigen::Isometry3d moving_foot_target_iso;

    Eigen::VectorXd q_des;
    Eigen::VectorXd q;
    Eigen::VectorXd qdot;

    Eigen::VectorXd jpos_ini;

    // real : determined by sensor / simulation
    // int b_arfoot_contact;
    // int b_brfoot_contact;
    // int b_alfoot_contact;
    // int b_blfoot_contact;
    std::array<int, Magneto::n_leg> b_foot_contact_list;

    // planning : determined by planner
    // bool b_arfoot_contact_plan;
    // bool b_brfoot_contact_plan;
    // bool b_alfoot_contact_plan;
    // bool b_blfoot_contact_plan;
    // std::map<int, bool> b_contact_plan_map;
    std::array<bool, Magneto::n_leg> b_contact_plan_list;


    int num_step_copy;
    int phase_copy;
 
    // save planned result for the plot
    std::vector<Eigen::Isometry3d> foot_target_list;
    std::vector<Eigen::VectorXd> com_des_list;  

    /* -------------- Magneto by JE ---------------*/

    // save feasibile com for the plot
    std::vector<std::pair<double, Eigen::Vector3d>> feasible_com_list;
    Eigen::Vector3d com_pos_ini_step;
    Eigen::Vector3d com_pos_des_step;
    Eigen::Vector3d com_pos_init;
    Eigen::Vector3d com_pos_target;
    int check_com_planner_updated;
    // foot desired position
    Eigen::VectorXd foot_pos_init;
    Eigen::VectorXd foot_pos_target;
    int check_foot_planner_updated;
    // magentic force
    // Eigen::VectorXd arf_magenetic_wrench;
    // Eigen::VectorXd alf_magenetic_wrench;
    // Eigen::VectorXd brf_magenetic_wrench;
    // Eigen::VectorXd blf_magenetic_wrench;

    /*-------------- Magneto by JE ---------------*/

    // data manager
    Eigen::VectorXd com_pos;
    Eigen::VectorXd com_vel;
    Eigen::VectorXd mom;
    Eigen::VectorXd est_com_vel;

    Eigen::VectorXd com_pos_des;
    Eigen::VectorXd com_vel_des;
    Eigen::VectorXd mom_des;    

    std::array<Eigen::VectorXd, Magneto::n_leg> foot_pos;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_vel;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_pos_des;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_vel_des;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_ang_vel;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_ang_vel_des;
    std::array<Eigen::Quaternion<double>, Magneto::n_leg> foot_ori_quat;
    std::array<Eigen::Quaternion<double>, Magneto::n_leg> foot_ori_quat_des;

    Eigen::Quaternion<double> base_ori;
    Eigen::VectorXd base_ang_vel;

    Eigen::Quaternion<double> base_ori_des;
    Eigen::VectorXd base_ang_vel_des;

    std::array<Eigen::VectorXd, Magneto::n_leg> foot_grf_des;
    std::array<Eigen::VectorXd, Magneto::n_leg> foot_grf;

    // Eigen::VectorXd al_rf_des;
    // Eigen::VectorXd bl_rf_des;
    // Eigen::VectorXd ar_rf_des;
    // Eigen::VectorXd br_rf_des;

    // Eigen::VectorXd al_rf;        
    // Eigen::VectorXd bl_rf;
    // Eigen::VectorXd ar_rf;
    // Eigen::VectorXd br_rf;

    Eigen::VectorXd des_jacc_cmd;

   private:
    MagnetoStateProvider(RobotSystem* _robot);
    RobotSystem* robot_;
};
