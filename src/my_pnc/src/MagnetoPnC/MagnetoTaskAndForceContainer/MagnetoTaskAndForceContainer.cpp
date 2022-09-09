#include <my_pnc/MagnetoPnC/MagnetoTaskAndForceContainer/MagnetoTaskAndForceContainer.hpp>

MagnetoTaskAndForceContainer::MagnetoTaskAndForceContainer(RobotSystem* _robot)
    : TaskAndForceContainer(_robot) {
  _InitializeTasks();
  _InitializeContacts();
  _InitializeMagnetisms();
}

MagnetoTaskAndForceContainer::~MagnetoTaskAndForceContainer() {
  for( auto &task : task_container_)
    delete task;
  for( auto &contact : contact_container_)
    delete contact;
}

void MagnetoTaskAndForceContainer::_InitializeTasks() {
  my_utils::pretty_constructor(2, "Magneto Task And Force Container");

  // CoM and Pelvis Tasks
  task_container_[MAGNETO_TASK::COM] = 
      new CoMTask(robot_);
  task_container_[MAGNETO_TASK::BASE_ORI]  = 
      new BasicTask(robot_, BasicTaskType::LINKORI, 3, MagnetoBodyNode::base_link); 
  task_container_[MAGNETO_TASK::JOINT_TASK]  = 
      new BasicTask(robot_, BasicTaskType::JOINT, Magneto::n_adof);

  // Set Foot Motion Tasks
  for(int i(0); i<Magneto::n_leg; ++i){
    task_container_[MAGNETO_TASK::A1_POS+i] = 
      new BasicTask(robot_,BasicTaskType::LINKXYZ, 3, MagnetoFoot::LinkIdx[i]);
    task_container_[MAGNETO_TASK::A1_ORI+i]  = 
      new BasicTask(robot_,BasicTaskType::LINKORI, 3, MagnetoFoot::LinkIdx[i]);
  } 

  // initialize bool task
  for(int i(0);i<MAGNETO_TASK::n_task;++i)
    b_task_list_[i] = false;
}

void MagnetoTaskAndForceContainer::_InitializeContacts() {

  friction_coeff_ = 0.7; // updated later in setContactFriction 
  
  double foot_x = 0.02; // 0.05; 
  double foot_y = 0.02; 

full_dim_contact_ = 0;
for(int i(0); i<Magneto::n_leg; ++i) {
    contact_container_[i] = new BodyFrameSurfaceContactSpec(
                              robot_, 
                              MagnetoFoot::LinkIdx[i], 
                              foot_x, foot_y, 
                              friction_coeff_);

    full_dim_contact_ += contact_container_[i]->getDim();
    b_feet_contact_list_[i] = true;
  }
  dim_contact_ = full_dim_contact_;    

}


void MagnetoTaskAndForceContainer::_InitializeMagnetisms(){
  magnetic_force_ = 0.;
  residual_ratio_ = 0.;
  for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
    b_magnetism_list_[foot_idx]=false;
  }
}

void MagnetoTaskAndForceContainer::setContactFriction(){
  setContactFriction(friction_coeff_);
} 

void MagnetoTaskAndForceContainer::setContactFriction(double _mu){
  // initialize contact
  for(auto &contact : contact_container_)
    ((BodyFrameSurfaceContactSpec*)contact)->setFrictionCoeff(_mu);
} 

void MagnetoTaskAndForceContainer::paramInitialization(const YAML::Node& node) {
  
  try {
    
    my_utils::readParameter(node, "w_qddot", w_qddot_);
    my_utils::readParameter(node, "w_xddot", w_xddot_contact_);
    my_utils::readParameter(node, "w_xddot_nocontact", w_xddot_nocontact_);
    my_utils::readParameter(node, "w_rf", w_rf_);
    my_utils::readParameter(node, "w_rf_z", w_rf_z_contact_);
    my_utils::readParameter(node, "w_rf_z_nocontact", w_rf_z_nocontact_);
    my_utils::readParameter(node, "max_rf_z", max_rf_z_contact_);
    my_utils::readParameter(node, "max_rf_z_nocontact", max_rf_z_nocontact_);

  } catch (std::runtime_error& e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }

  // initialize parameters
  int dim_contact = contact_container_[0]->getDim();
  int idx_rf_z = contact_container_[0]->getFzIndex();

  W_qddot_ = Eigen::VectorXd::Constant(Magneto::n_dof, w_qddot_);
  W_xddot_contact_  = Eigen::VectorXd::Constant(dim_contact, w_xddot_contact_);
  W_xddot_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_xddot_contact_);
  W_xddot_nocontact_[idx_rf_z] = w_xddot_nocontact_;
  W_rf_contact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_contact_[idx_rf_z] = w_rf_z_contact_;
  W_rf_nocontact_ = Eigen::VectorXd::Constant(dim_contact, w_rf_);
  W_rf_nocontact_[idx_rf_z] = w_rf_z_nocontact_;

  // Set Maximum Forces
  for(auto &contact : contact_container_)
    ((BodyFrameSurfaceContactSpec*)contact)->setMaxFz(max_rf_z_contact_);

}

// -------------------------------------------------------
//    set functions
// -------------------------------------------------------
void MagnetoTaskAndForceContainer::set_magnetism(int moving_foot_idx) {
  for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
    b_magnetism_list_[foot_idx] = (moving_foot_idx!=foot_idx);
  } 
}

void MagnetoTaskAndForceContainer::set_residual_magnetic_force(int moving_foot_idx, double contact_distance) {
  if(moving_foot_idx<0){
    J_residual_ = Eigen::MatrixXd::Zero(6, robot_->getNumDofs()); // 6 x ndof
    F_residual_ = Eigen::VectorXd::Zero(J_residual_.rows());
  }else{
    int moving_cop = MagnetoFoot::LinkIdx[moving_foot_idx];
    J_residual_ = robot_->getBodyNodeCoMBodyJacobian(moving_cop); // 6 x ndof
    // set F_residual_
    double distance_ratio;
    double distance_constant = 0.005*4.;
    distance_ratio = distance_constant / (contact_distance + distance_constant);
    distance_ratio = distance_ratio*distance_ratio;
    residual_force_ = distance_ratio * magnetic_force_ * (residual_ratio_/100.);
    F_residual_ = Eigen::VectorXd::Zero(J_residual_.rows());
    F_residual_[F_residual_.size()-1] = residual_force_;
  }
}

void MagnetoTaskAndForceContainer::set_contact_magnetic_force(int moving_foot_idx) {
  // set F_magnetic_ based on b_magnetism_mabp_  
  F_magnetic_ = Eigen::VectorXd::Zero(full_dim_contact_);

  int contact_link_idx(0), dim_contact(0), fz_idx;

  for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
    if(foot_idx != moving_foot_idx){
      fz_idx = dim_contact + 
      ((BodyFrameSurfaceContactSpec*)(contact_container_[foot_idx]))->getFzIndex(); 
      if(b_magnetism_list_[foot_idx]){
        F_magnetic_[fz_idx] = magnetic_force_;
      }
      else{
        F_magnetic_[fz_idx] = residual_force_;
      }
      dim_contact += ((BodyFrameSurfaceContactSpec*)(contact_container_[foot_idx]))->getDim();
    }
  }
  F_magnetic_ = F_magnetic_.head(dim_contact);  
}


void MagnetoTaskAndForceContainer::set_contact_list(int moving_foot_idx) {
  // build contact_list_
  dim_contact_=0;
  contact_list_.clear();

  for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
    if(foot_idx == moving_foot_idx){
      b_feet_contact_list_[foot_idx]=false;
    }
    else{
      b_feet_contact_list_[foot_idx]=true;
      contact_list_.push_back(
        (BodyFrameSurfaceContactSpec*)(contact_container_[foot_idx]));
      dim_contact_ += (contact_container_[foot_idx])->getDim();
    }
  }
}

void MagnetoTaskAndForceContainer::set_maxfz_contact(int moving_foot_idx) {
  set_maxfz_contact(moving_foot_idx,
                    max_rf_z_contact_, 
                    max_rf_z_nocontact_);
}

void MagnetoTaskAndForceContainer::set_maxfz_contact(int moving_foot_idx,
                                                  double max_rfz_cntct,
                                                  double max_rfz_nocntct) {
  for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
    if(foot_idx == moving_foot_idx) {
      ((BodyFrameSurfaceContactSpec*)(contact_container_[foot_idx]))
          ->setMaxFz(max_rfz_nocntct);
    } else {
      ((BodyFrameSurfaceContactSpec*)(contact_container_[foot_idx]))
          ->setMaxFz(max_rfz_cntct);
    }
  }
}

void MagnetoTaskAndForceContainer::compute_weight_param(int moving_foot_idx,
                                    const Eigen::VectorXd &W_contact,
                                    const Eigen::VectorXd &W_nocontact,
                                    Eigen::VectorXd &W_result) {
  int dim_vec = W_contact.size();
  int num_contact = contact_list_.size();
  int idx_accum = 0;
  W_result = Eigen::VectorXd::Zero(dim_vec*num_contact);
  for(auto it = contact_list_.begin(); 
        it != contact_list_.end(); it++) {
    if(((BodyFrameSurfaceContactSpec*)(*it))->getLinkIdx()
                  ==MagnetoFoot::LinkIdx[moving_foot_idx]) {
      // swing foot - no contact
      W_result.segment(idx_accum,dim_vec) = W_nocontact;

    } else {
      W_result.segment(idx_accum,dim_vec) = W_contact;
    }
    idx_accum += dim_vec;
  }
}

void MagnetoTaskAndForceContainer::clear_task_list() {
  for(int i(0);i<MAGNETO_TASK::n_task;++i)
    b_task_list_[i] = false;
}

void MagnetoTaskAndForceContainer::check_task_list(){
  std::cout<<"check_task_list : ";
  for(int i(0);i<MAGNETO_TASK::n_task;++i)
    std::cout<< b_task_list_[i]<< ", ";
  std::cout<<std::endl;
}

Task* MagnetoTaskAndForceContainer::get_foot_pos_task(int foot_idx) {
  return task_container_[MAGNETO_TASK::A1_POS + foot_idx];
}
Task* MagnetoTaskAndForceContainer::get_foot_ori_task(int foot_idx) {
  return task_container_[MAGNETO_TASK::A1_ORI + foot_idx];
}
