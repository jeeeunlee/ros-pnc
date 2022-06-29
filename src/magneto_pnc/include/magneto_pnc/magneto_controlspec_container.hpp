#pragma once

#include <vector>
#include <array>

#include <magneto_pnc/magneto_definition.hpp>
#include <pnc_utils/io_utilities.hpp>
#include <pnc_core/control_spec_container.hpp>

typedef int FootIdx;
typedef int FootLinkIdx;

class RobotSystem;

#include <pnc_core/wbc/Contact/ContactSpec.hpp>


#include <pnc_core/wbc/Task/task_set.hpp>
#include <magneto_pnc/magneto_specs/magnet_spec.hpp>
#include <magneto_pnc/magneto_specs/contact_weight.hpp>

// Object which publicly contains all the tasks, contacts and reaction forces
class MagnetoControlSpecContainer : public ControlSpecContainer{
 public:
  MagnetoControlSpecContainer(RobotSystem* _robot);
  ~MagnetoControlSpecContainer();

  // -------------------------------------------------------
  //    initialization
  // -------------------------------------------------------
  void weightParamInitialization(const YAML::Node& node);
  void contactParamInitialization(const YAML::Node& node);
  void magneticParamInitialization(const YAML::Node& node);
  void setContactFriction();
  void setContactFriction(const Eigen::VectorXd& _mu_vec);
  void setContactFriction(int foot_idx, double mu);
  void setMagneticForce();
  void setMagneticForce(const Eigen::VectorXd& magnetic_force,
                        const Eigen::VectorXd& residual_ratio);
  void setMagneticForce(int foot_idx, double fm, double rr);
  
  // -------------------------------------------------------
  //    set functions
  // -------------------------------------------------------
  // magnetism
  void get_magnetism_onoff(std::array<bool, Magneto::n_leg>& onoff);
  void update_magnet_forces();
  void set_foot_magnet_off(int moving_cop);
  void set_contact_magnetic_force(int moving_cop);
  void set_magnet_distance(int moving_cop, double contact_distance=0.0);
  // contact
  void set_contact_list(int moving_cop);
  // contact spec
  void set_contact_maxfz(int moving_cop=-1);

  void set_contact_weight_param(int trans_cop=-1);
  void reshape_weight_param(double alpha,
                            int slip_cop,
                            int moving_cop=-1);
  // task
  void clear_task_list();
  void add_task_list(Task* task);
  Task* get_foot_pos_task(int moving_cop);
  Task* get_foot_ori_task(int moving_cop);

  //  
  int footLink2FootIdx(int moving_cop);

 protected:
  void _InitializeTasks();
  void _InitializeContacts();
  void _InitializeMagnetisms();
  void _InitializeWeightParams();
  void _DeleteTasks();
  void _DeleteContacts();
  void _DeleteMagnetisms();
  void _DeleteOthers();

 public:

  // -------------------------------------------------------
  // Task Member variables
  // -------------------------------------------------------
  Task* com_task_;
  Task* joint_task_;
  Task* base_ori_task_;
  std::array<Task*, Magneto::n_leg> feet_pos_tasks_;
  std::array<Task*, Magneto::n_leg> feet_ori_tasks_;

  // -------------------------------------------------------
  // Contact Member variables
  // -------------------------------------------------------
  std::array<bool, Magneto::n_leg> b_feet_contact_list_;
  std::array<ContactSpec*, Magneto::n_leg> feet_contacts_;
  int full_contact_dim_;

  // -------------------------------------------------------
  // Magnetic
  // -------------------------------------------------------
  // MagnetSpec* feet_magnets_[Magneto::n_leg];
  std::array<MagnetSpec*, Magneto::n_leg> feet_magnets_;

  Eigen::VectorXd F_magnetic_;
  
  Eigen::VectorXd F_residual_;
  Eigen::MatrixXd J_residual_;

  Eigen::VectorXd friction_coeff_;
  Eigen::VectorXd magnetic_force_; //[N]
  Eigen::VectorXd residual_ratio_; //[%]
  Eigen::VectorXd residual_force_;

  // -------------------------------------------------------
  // Weight Parameters
  // -------------------------------------------------------
  std::array<ContactWeight*, Magneto::n_leg> feet_weights_;
  
  // Max rf_z for contactSpec in contact_list
  double max_rf_z_contact_;
  double max_rf_z_nocontact_;

  double max_rf_z_trans_; // be updated from TrajManager

  // QP weights init & target
  double w_qddot_;
  double w_xddot_;
  double w_xddot_z_contact_; // =w_xddot_
  double w_xddot_z_nocontact_;
  double w_rf_;
  double w_rf_z_contact_;
  double w_rf_z_nocontact_;

  double w_xddot_z_trans_;
  double w_rf_z_trans_;

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
