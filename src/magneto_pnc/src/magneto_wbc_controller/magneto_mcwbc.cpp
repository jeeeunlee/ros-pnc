#include <magneto_pnc/magneto_wbc_controller/magneto_mcwbc.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_control_architecture_set.hpp>

MagnetoMCWBC::MagnetoMCWBC( MagnetoControlSpecContainer* _ws_container, 
                            RobotSystem* _robot, int _controller_type ){
  pnc_utils::pretty_constructor(2, "Magnetic Contact Whole Body Controller");
  // Initialize Flag
  b_first_visit_ = true;

  // Initialize Pointer to the Task and Force Container
  ws_container_ = _ws_container;
  robot_ = _robot;

  // Initialize State Provider
  sp_ = MagnetoStateProvider::getStateProvider(robot_);

  // Create joint_integrator_
  joint_integrator_ = new JointIntegrator(Magneto::n_adof, MagnetoAux::servo_rate);

  // Initialize Actuator selection list  
  act_list_.resize(Magneto::n_dof, true);
  for (int i(0); i < Magneto::n_vdof; ++i) 
      act_list_[Magneto::idx_vdof[i]] = false;

  // Initialize WBC 
  if(_controller_type == MCWBC_TYPES::MFWBCC)
    mcwbc_ = new MFWBCC(act_list_);
  else if(_controller_type == MCWBC_TYPES::MRWBCC)
    mcwbc_ = new MRWBCC(act_list_);
  else{
    std::cout<<"error @ MagnetoMCWBC : invalid controller_type"<<_controller_type<<std::endl;
    exit(0);
  }
  mcwbc_param_ = new MCWBC_ExtraData();
  kin_wbc_ = new KinWBC(act_list_);
  
  tau_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  qddot_cmd_ = Eigen::VectorXd::Zero(Magneto::n_adof);

  // Initialize desired pos, vel, acc containers
  jpos_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jvel_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);
  jacc_des_ = Eigen::VectorXd::Zero(Magneto::n_dof);

  jvel_des_integrated_ = Eigen::VectorXd::Zero(Magneto::n_adof);
  jpos_des_integrated_ = Eigen::VectorXd::Zero(Magneto::n_adof);
}

MagnetoMCWBC::~MagnetoMCWBC() {
  delete mcwbc_;
  delete mcwbc_param_;
}


void MagnetoMCWBC::ctrlInitialization(const YAML::Node& node) {
  // WBC Defaults
  b_enable_torque_limits_ = true;  // Enable WBC torque limits

  // Joint Integrator parameters
  double vel_freq_cutoff = 2.; // Hz
  double pos_freq_cutoff = 1.; // Hz
  double max_pos_error = 0.2; // radians. 
                            // After position integrator, 
                            // deviation from current position

  // Load Custom Parmams ----------------------------------
  try {
    // Load Integration Parameters
    pnc_utils::readParameter(node, "enable_torque_limits", b_enable_torque_limits_);
    pnc_utils::readParameter(node, "torque_limit", torque_limit_); 
    pnc_utils::readParameter(node, "acc_limit", acc_limit_);

    pnc_utils::readParameter(node, "velocity_freq_cutoff", vel_freq_cutoff);
    pnc_utils::readParameter(node, "position_freq_cutoff", pos_freq_cutoff);
    pnc_utils::readParameter(node, "max_position_error", max_pos_error);

    pnc_utils::readParameter(node, "kp", Kp_);
    pnc_utils::readParameter(node, "kd", Kd_);   


  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
  // ----------------------------------

  // Set WBC Parameters
  // Enable Torque Limits
  tau_min_ =
      // sp_->getActiveJointValue(robot_->GetTorqueLowerLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, -torque_limit_); //-2500.
  tau_max_ =
      // sp_->getActiveJointValue(robot_->GetTorqueUpperLimits());
      Eigen::VectorXd::Constant(Magneto::n_adof, torque_limit_); //-2500.
  mcwbc_->setTorqueLimits(tau_min_, tau_max_);

  acc_min_ = Eigen::VectorXd::Constant(Magneto::n_adof, -acc_limit_);
  acc_max_ = Eigen::VectorXd::Constant(Magneto::n_adof, +acc_limit_);

  // Set Joint Integrator Parameters  
  // pnc_utils::pretty_print(robot_->GetVelocityLowerLimits(), std::cout, "vel limit");
  // pnc_utils::pretty_print(robot_->GetVelocityUpperLimits(), std::cout, "vel limit");
  // pnc_utils::pretty_print(robot_->GetPositionLowerLimits(), std::cout, "pos limit");
  // pnc_utils::pretty_print(robot_->GetPositionUpperLimits(), std::cout, "pos limit");

  joint_integrator_->setVelocityFrequencyCutOff(vel_freq_cutoff);
  joint_integrator_->setPositionFrequencyCutOff(pos_freq_cutoff);
  joint_integrator_->setMaxPositionError(max_pos_error);
  joint_integrator_->setVelocityBounds( 
    robot_->getActiveJointValue(robot_->GetVelocityLowerLimits()),
    robot_->getActiveJointValue(robot_->GetVelocityUpperLimits()) );
  joint_integrator_->setPositionBounds( 
    robot_->getActiveJointValue(robot_->GetPositionLowerLimits()),
    robot_->getActiveJointValue(robot_->GetPositionUpperLimits()) );
}

void MagnetoMCWBC::_PreProcessing_Command() {
  // Update Dynamic Terms
  A_ = robot_->getMassMatrix();
  Ainv_ = robot_->getInvMassMatrix();
  grav_ = robot_->getGravity();
  coriolis_ = robot_->getCoriolis();

  // Grab Variables from the container.
  mcwbc_param_->W_qddot_ = ws_container_->W_qddot_;
  mcwbc_param_->W_xddot_ = ws_container_->W_xddot_;
  mcwbc_param_->W_rf_ = ws_container_->W_rf_;

  // Clear out local pointers
  task_list_.clear();
  contact_list_.clear();
  magnet_list_.clear();

  // Update task and contact list pointers from container object
  for (int i = 0; i < ws_container_->task_list_.size(); i++) {
    task_list_.push_back(ws_container_->task_list_[i]);
  }
  for (int i = 0; i < ws_container_->contact_list_.size(); i++) {
    contact_list_.push_back(ws_container_->contact_list_[i]);
  }

  for (int i=0; i<ws_container_->feet_magnets_.size(); i++) {
    magnet_list_.push_back(ws_container_->feet_magnets_[i]);
  }

  // Update Contact Spec
  for (int i = 0; i < contact_list_.size(); i++) {
    contact_list_[i]->updateContactSpec();
  }
}

void MagnetoMCWBC::getInitialCommand(const Eigen::VectorXd& _jnt_pos_des,
                                    void* _cmd) {
  // _jnt_pos_des << 0.0, 0.0, -1.5707963268, 
  //               0.0, 0.0, -1.5707963268, 
  //               0.0, 0.0, -1.5707963268, 
  //               0.0, 0.0, -1.5707963268;  

  if (b_first_visit_) {
    firstVisit();
    b_first_visit_ = false;
  }  

  jvel_des_.setZero();
  jtrq_des_.setZero();
  jpos_des_.setZero();
  for(int i(0); i<Magneto::n_adof; ++i){
    jpos_des_[Magneto::idx_adof[i]] = _jnt_pos_des[i];
  }
    
  Eigen::VectorXd jacc_des_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);
  for(int i(0); i<Magneto::n_adof; ++i) {
    jacc_des_cmd[i] +=
          Kp_[i]*(jpos_des_[Magneto::idx_adof[i]] - sp_->q[Magneto::idx_adof[i]])
          + Kd_[i]*(jvel_des_[Magneto::idx_adof[i]] - sp_->qdot[Magneto::idx_adof[i]]);
  }

  pnc_utils::CropVector(jacc_des_cmd, acc_min_, acc_max_);
  joint_integrator_->integrate(jacc_des_cmd, 
                                robot_->getActiveQdot(),
                                robot_->getActiveQ(), 
                                jvel_des_integrated_,
                                jpos_des_integrated_);

  // SET COMMAND
  ((MagnetoCommand*)_cmd)->jtrq = jtrq_des_;
  ((MagnetoCommand*)_cmd)->q = jpos_des_integrated_; // sp_->getActiveJointValue(jpos_des_);
  ((MagnetoCommand*)_cmd)->qdot = jvel_des_integrated_; // sp_->getActiveJointValue(jvel_des_);
  
  ((MagnetoCommand*)_cmd)->magnetism_onoff[MagnetoFoot::AL] = 1;
  ((MagnetoCommand*)_cmd)->magnetism_onoff[MagnetoFoot::AR] = 1;
  ((MagnetoCommand*)_cmd)->magnetism_onoff[MagnetoFoot::BL] = 1;
  ((MagnetoCommand*)_cmd)->magnetism_onoff[MagnetoFoot::BR] = 1; 
  
  // SAVE DATA  
  Eigen::VectorXd current_q = robot_->getActiveQ();
  pnc_utils::pretty_print(current_q, std::cout, "current_q");
  pnc_utils::pretty_print(_jnt_pos_des, std::cout, "_jnt_pos_des");
  pnc_utils::pretty_print(jpos_des_integrated_, std::cout, "jpos_des_integrated_");
  pnc_utils::pretty_print(jacc_des_cmd, std::cout, "jacc_des_cmd");
  std::cout<<"----------"<<std::endl;
  
}

void MagnetoMCWBC::getCommand(void* _cmd) {

  if (b_first_visit_) {
    firstVisit();
    b_first_visit_ = false;
  }
  jtrq_des_.setZero();

  // GET jacc_des_cmd  
  Eigen::VectorXd jacc_des_cmd = Eigen::VectorXd::Zero(Magneto::n_dof);
  
  // grab & update task_list and contact_list & QP weights
  _PreProcessing_Command();

  // KIN-WBC 
  kin_wbc_->FindFullConfiguration(sp_->q, task_list_, contact_list_, 
                                    jpos_des_, jvel_des_, jacc_des_); 

  jacc_des_cmd = jacc_des_;
  for(int i(0); i<Magneto::n_adof; ++i) {
    jacc_des_cmd[Magneto::idx_adof[i]] +=
          Kp_[i]*(jpos_des_[Magneto::idx_adof[i]] - sp_->q[Magneto::idx_adof[i]])
          + Kd_[i]*(jvel_des_[Magneto::idx_adof[i]] - sp_->qdot[Magneto::idx_adof[i]]);
  }
                                
  // DYN-WBC
  mcwbc_->updateSetting(A_, Ainv_, coriolis_, grav_);
  mcwbc_->makeTorqueGivenRef(jacc_des_cmd, contact_list_, magnet_list_, jtrq_des_, mcwbc_param_);
  set_grf_des(); // sp_->al_rf_des FROM mcwbc_param_->Fr_     


  // JOINT INTEGRATOR
  joint_integrator_->integrate(robot_->getActiveJointValue(mcwbc_param_->qddot_), 
                                robot_->getActiveQdot(),
                                robot_->getActiveQ(), 
                                jvel_des_integrated_,
                                jpos_des_integrated_);

  // SET COMMAND
  ((MagnetoCommand*)_cmd)->jtrq = jtrq_des_;
  ((MagnetoCommand*)_cmd)->q = jpos_des_integrated_; // sp_->getActiveJointValue(jpos_des_);
  ((MagnetoCommand*)_cmd)->qdot = jvel_des_integrated_; // sp_->getActiveJointValue(jvel_des_);
  ws_container_->get_magnetism_onoff(((MagnetoCommand*)_cmd)->magnetism_onoff);
  
  // SAVE DATA  
  // std::cout<<"----------"<<std::endl;
  Eigen::VectorXd current_q = robot_->getActiveQ();
  Eigen::VectorXd jpos_des_kin = robot_->getActiveJointValue(jpos_des_);
  // pnc_utils::pretty_print(current_q, std::cout, "current_q");  
  // pnc_utils::pretty_print(jpos_des_integrated_, std::cout, "jpos_des_integrated_");
  // pnc_utils::pretty_print(jpos_des_kin, std::cout, "jpos_des_kin");
  pnc_utils::saveVector(current_q,"current_q");
  pnc_utils::saveVector(jpos_des_integrated_,"jpos_des_integrated_");
  pnc_utils::saveVector(jpos_des_kin,"jpos_des_kin");
  // std::cout<<"----------"<<std::endl;
  Eigen::VectorXd jacc_des_cmd_kin = robot_->getActiveJointValue(jacc_des_cmd);
  Eigen::VectorXd jacc_des_cmd_dyn = robot_->getActiveJointValue(mcwbc_param_->qddot_);
  
  // pnc_utils::pretty_print(jacc_des_cmd_kin, std::cout, "jacc_des_cmd_kin");
  // pnc_utils::pretty_print(jacc_des_cmd_dyn, std::cout, "jacc_des_cmd_dyn");  
  pnc_utils::saveVector(jacc_des_cmd_kin,"jacc_des_cmd_kin");
  pnc_utils::saveVector(jacc_des_cmd_dyn,"jacc_des_cmd_dyn");  
  // std::cout<<"----------"<<std::endl;
  
  // pnc_utils::pretty_print(jvel_des_integrated_, std::cout, "jvel_des");
  // pnc_utils::pretty_print(((MagnetoCommand*)_cmd)->jtrq, std::cout, "jtrq");
  // pnc_utils::pretty_print(jpos_des_, std::cout, "jpos_des_");
  // pnc_utils::pretty_print(((MagnetoCommand*)_cmd)->q, std::cout, "q");
  // pnc_utils::pretty_print(((MagnetoCommand*)_cmd)->qdot, std::cout, "qdot");

  Eigen::VectorXd magnet_state = Eigen::VectorXd::Zero(4);
  magnet_state<<((MagnetoCommand*)_cmd)->magnetism_onoff[0],
                ((MagnetoCommand*)_cmd)->magnetism_onoff[1],
                ((MagnetoCommand*)_cmd)->magnetism_onoff[2],
                ((MagnetoCommand*)_cmd)->magnetism_onoff[3];
  pnc_utils::saveVector(magnet_state,"magnet_state");  
}


void MagnetoMCWBC::firstVisit() { 
  Eigen::VectorXd jvel_ini = robot_->getActiveQdot();
  Eigen::VectorXd jpos_ini = robot_->getActiveQ();
  joint_integrator_->initializeStates(jvel_ini, jpos_ini);  
}

void MagnetoMCWBC::set_grf_des(){
  // initialize
  std::array<Eigen::VectorXd, Magneto::n_leg> grf_des_list;  
  for(int foot_idx(0); foot_idx<Magneto::n_leg; ++foot_idx) 
    grf_des_list[foot_idx] = Eigen::VectorXd::Zero(6);  

  int dim_grf_stacked(0), dim_grf(0), foot_idx;
  for ( auto &contact : contact_list_) {
    foot_idx = ws_container_->footLink2FootIdx(contact->getLinkIdx());
    if(foot_idx>-1 && foot_idx< Magneto::n_leg){
      dim_grf = contact->getDim();
      grf_des_list[foot_idx]  = mcwbc_param_->Fr_.segment(dim_grf_stacked, dim_grf);
      dim_grf_stacked += dim_grf;    
    }else{
      std::cout<<"set_grf_des??? foot_idx = "<< foot_idx << std::endl;
      std::cout<<"set_grf_des???? link idx = "<< contact->getLinkIdx() << std::endl;
    }    
  }

  sp_->al_rf_des = grf_des_list[MagnetoFoot::AL];
  sp_->ar_rf_des = grf_des_list[MagnetoFoot::AR];
  sp_->bl_rf_des = grf_des_list[MagnetoFoot::BL];
  sp_->br_rf_des = grf_des_list[MagnetoFoot::BR];
  
}

