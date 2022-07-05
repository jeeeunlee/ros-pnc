#pragma once

#include <vector>
#include <mutex> 

#include <pnc_core/control_architecture.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <magneto_pnc/magneto_command.hpp>

#include <magneto_pnc/magneto_wbc_controller/state_machines/state_machine_set.hpp>
// #include <magneto_pnc/magneto_wbc_controller/magneto_wbmc.hpp>
#include <magneto_pnc/magneto_wbc_controller/magneto_mcwbc.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_controlspec_container.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_planner_container.hpp>


#include <magneto_pnc/magneto_estimator/slip_observer.hpp>

class MagnetoStateProvider;

class MagnetoMpcControlArchitecture : public ControlArchitecture {
 public:
  MagnetoMpcControlArchitecture(RobotSystem* _robot);
  virtual ~MagnetoMpcControlArchitecture();
  virtual void ControlArchitectureInitialization();
  virtual void getCommand(void* _command);
  virtual void addState(void* _user_state_command);

  void saveData();
  void getIVDCommand(void* _command);
  void estimateAndReplan();
  
  StateSequence<MotionCommand>* states_sequence_;
  MotionCommand user_cmd_;
  
  // initialize parameters
  void _ReadParameters();
  void _InitializeParameters();
  bool b_state_first_visit_;
  bool b_env_param_updated_;

 protected:
  MagnetoStateProvider* sp_;  
  
  YAML::Node cfg_;

  int controller_type_;

 public:
  // Task and Force Containers
  MagnetoControlSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  // Controller Object
  MagnetoMCWBC* wbc_controller;

  // Observers
  SlipObserver* slip_ob_;
  SlipObserverData* slip_ob_data_;

  private:
    Eigen::VectorXd tau_min_;
    Eigen::VectorXd tau_max_;
};


