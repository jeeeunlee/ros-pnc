#pragma once

#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>
#include <my_pnc/TaskAndForceContainer.hpp>
#include <my_wbc/Contact/BasicContactSpec.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_pnc/MagnetoPnC/MagnetoTask/TaskSet.hpp>

namespace MAGNETO_TASK {
// in priority
constexpr int COM = 0;
constexpr int BASE_ORI = 1;

constexpr int AL_POS = 2;
constexpr int AR_POS = 3;
constexpr int BL_POS = 4;
constexpr int BR_POS = 5;
constexpr int CL_POS = 6;
constexpr int CR_POS = 7;

constexpr int AL_ORI = 8;
constexpr int AR_ORI = 9;
constexpr int BL_ORI = 10;
constexpr int BR_ORI = 11;
constexpr int CL_ORI = 12;
constexpr int CR_ORI = 13;

constexpr int JOINT_TASK = 14;
constexpr int n_task = 15;
}; //namespace MAGNETO_TASK 

// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoTaskAndForceContainer : public TaskAndForceContainer {
 public:
  MagnetoTaskAndForceContainer(RobotSystem* _robot);
  ~MagnetoTaskAndForceContainer();
  void paramInitialization(const YAML::Node& node);
  
  void setContactFriction();
  void setContactFriction(double _mu);

  // -------------------------------------------------------
  //    set functions
  // -------------------------------------------------------
  // magnetism
  void set_magnetism(int moving_foot_idx);
  void set_contact_magnetic_force(int moving_foot_idx);
  void set_residual_magnetic_force(int moving_foot_idx, double contact_distance=0.0);
  // contact
  void set_contact_list(int moving_foot_idx);
  // contact spec
  void set_maxfz_contact(int moving_foot_idx);
  void set_maxfz_contact(int moving_foot_idx,
                        double max_rfz_cntct,
                        double max_rfz_nocntct);
  void compute_weight_param(int moving_foot_idx,
                            const Eigen::VectorXd &W_contact,
                            const Eigen::VectorXd &W_nocontact,
                            Eigen::VectorXd &W_result);


 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _InitializeMagnetisms();
  void _DeleteTasks();
  void _DeleteContacts();

 public:
  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  std::array<bool, MAGNETO_TASK::n_task> b_task_list_;
  std::array<Task*, MAGNETO_TASK::n_task> task_container_;

  // task
  void clear_task_list();
  void delete_task_list(int task_id){ b_task_list_[task_id] = false; }
  void add_task_list(int task_id){ b_task_list_[task_id] = true; }
  void check_task_list();
  Task* get_foot_pos_task(int foot_idx);
  Task* get_foot_ori_task(int foot_idx);

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  std::array<bool, Magneto::n_leg> b_feet_contact_list_;
  std::array<ContactSpec*, Magneto::n_leg> contact_container_;
  int dim_contact_;
  int full_dim_contact_;

  double max_fz_;

  // -------------------------------------------------------
  // Magnetic
  // -------------------------------------------------------
  std::array<bool, Magneto::n_leg> b_magnetism_list_;
  Eigen::VectorXd F_magnetic_;
  
  Eigen::VectorXd F_residual_;
  Eigen::MatrixXd J_residual_;
  double w_res_;

  double friction_coeff_;
  double magnetic_force_; //[N]
  double residual_ratio_; //[%]
  double residual_force_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------
  // Max rf_z for contactSpec in contact_list
  double max_rf_z_contact_;
  double max_rf_z_nocontact_;

  double max_rf_z_trans_; // be updated from TrajManager

  // QP weights init & target
  double w_qddot_;
  double w_xddot_contact_;
  double w_xddot_nocontact_;
  double w_rf_;
  double w_rf_z_contact_;
  double w_rf_z_nocontact_;

  // 
  Eigen::VectorXd W_xddot_contact_; // contact dim for 1 foot 
  Eigen::VectorXd W_rf_contact_;

  Eigen::VectorXd W_xddot_nocontact_;
  Eigen::VectorXd W_rf_nocontact_;

  // QP weights // be updated from TrajManager
  Eigen::VectorXd W_qddot_; 
  Eigen::VectorXd W_xddot_; // contact dim for all feet in contact
  Eigen::VectorXd W_rf_;

};
