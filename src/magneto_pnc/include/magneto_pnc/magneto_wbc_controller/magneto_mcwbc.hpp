#pragma once

#include <magneto_pnc/magneto_definition.hpp>
#include <magneto_pnc/magneto_interface.hpp>
#include <magneto_pnc/magneto_state_provider.hpp>
#include <magneto_pnc/magneto_controlspec_container.hpp>

#include <pnc_core/wbc/joint_integrator.hpp>
#include <pnc_core/wbc/kin_wbc.hpp>
#include <magneto_pnc/magneto_wbc_controller/mf_mcwbc.hpp>
#include <magneto_pnc/magneto_wbc_controller/mr_mcwbc.hpp>

// MCWBC (Magnetic Contact Whole Body Control)

namespace MCWBC_TYPES {
constexpr int MRWBCC = 0;
constexpr int MFWBCC = 1;
}; // namespace MCWBC_TYPES


class MagnetoMCWBC {
 public:
  MagnetoMCWBC(MagnetoControlSpecContainer* _ws_container,
                            RobotSystem* _robot,
                            int _controller_type);
  virtual ~MagnetoMCWBC();

  virtual void getCommand(void* _cmd);
  virtual void ctrlInitialization(const YAML::Node& node);

 protected:
  //  Processing Step for first visit
  virtual void firstVisit();  

  // Redefine PreProcessing Command
  virtual void _PreProcessing_Command();

  void set_grf_des();

 protected:
  RobotSystem* robot_;
  MagnetoControlSpecContainer* ws_container_;
  MagnetoStateProvider* sp_;  

  // -------------------------------------------------------
  // Controller Objects
  // -------------------------------------------------------
  std::vector<bool> act_list_;
  KinWBC* kin_wbc_;

  Eigen::VectorXd Fd_des_;
  Eigen::VectorXd tau_cmd_;
  Eigen::VectorXd qddot_cmd_;

  Eigen::VectorXd jpos_des_;
  Eigen::VectorXd jvel_des_;
  Eigen::VectorXd jacc_des_;
  Eigen::VectorXd jtrq_des_;

  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::MatrixXd grav_;
  Eigen::MatrixXd coriolis_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;
  std::vector<MagnetSpec*> magnet_list_;

  // -------------------------------------------------------
  // Parameters
  // -------------------------------------------------------  
  Eigen::VectorXd Kp_, Kd_;

  // Joint Integrator parameters
  double wbc_dt_;
  double vel_freq_cutoff_;  // Hz
  double pos_freq_cutoff_;  // Hz
  double max_pos_error_;    // radians. After position integrator, deviation
                            // from current position

  // 
  bool b_first_visit_;
  bool b_enable_torque_limits_;  // Enable IHWBC torque limits
  double torque_limit_;
  Eigen::VectorXd tau_min_;
  Eigen::VectorXd tau_max_;

 private:
 // Controller Objects
  // WBRMC* mcwbc_;
  // WBRMC_ExtraData* mcwbc_param_;

  MCWBC* mcwbc_;
  MCWBC_ExtraData* mcwbc_param_;
  
  
  
};
