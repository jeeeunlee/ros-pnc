
#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoPlanner/MagnetoTrajectoryManager.hpp>

MagnetoTrajectoryManager::MagnetoTrajectoryManager(MagnetoControlArchitecture* _ctrl_arch) {
    my_utils::pretty_constructor(2, "Magneto Trajectory Planner");

    ctrl_arch_ = _ctrl_arch;    

    robot_manager_ = new RobotSystem(*(ctrl_arch_->robot_));

    std::vector<bool> act_list;
    act_list.resize(Magneto::n_dof, true);
    for (int i(0); i < Magneto::n_vdof; ++i) 
        act_list[Magneto::idx_vdof[i]] = false;

    // Initialize WBC
    kin_wbc_ = new KinWBC(act_list);

    // contact
    double mu_ = 0.7; // will be updated later
    for(int foot_idx=0; foot_idx<Magneto::n_leg;++foot_idx){
        foot_contact_list_[foot_idx] = 
            new BodyFramePointContactSpec(robot_manager_,
                                MagnetoFoot::LinkIdx[foot_idx], mu_);
    }   

    joint_task_ = 
      new BasicTask(robot_manager_, BasicTaskType::FULLJOINT, Magneto::n_dof);

    // Set Foot Motion Tasks
    for(int foot_idx=0; foot_idx<Magneto::n_leg;++foot_idx){
        foot_task_list_[foot_idx] = 
            new BasicTask(robot_manager_, BasicTaskType::LINKXYZ, 3,
                            MagnetoFoot::LinkIdx[foot_idx]);
    }
}

MagnetoTrajectoryManager::~MagnetoTrajectoryManager() {
    delete joint_task_;
    delete robot_manager_;
    for(auto &task : foot_task_list_)
        delete task;
    for(auto &contact : foot_contact_list_)
        delete contact;


}

bool MagnetoTrajectoryManager::ParameterizeTrajectory(MotionCommand& motion_cmd,
                                                    const double& x_ratio_height, 
                                                    const double& t_starthold, 
                                                    const double& t_swing,
                                                    const double& t_endhold){
    
    if( !motion_cmd.get_foot_motion_command(foot_motion_data_, moving_foot_idx_) )
        return false;    

    // get init config and goal config                                                        
    n_dim_ = ctrl_arch_->robot_->getNumDofs();
    q_init_ = ctrl_arch_->robot_->getQ();
    dotq_init_ = ctrl_arch_->robot_->getQdot(); 
    ctrl_arch_->goal_planner_->computeGoal(motion_cmd);
    ctrl_arch_->goal_planner_->getGoalConfiguration(q_goal_);

    dotq_goal_ = Eigen::VectorXd::Zero(n_dim_);
    robot_manager_->updateSystem(q_init_, dotq_init_, false);    

    t0_ = 0.0; // step start (move under full support)
    t1_ = t0_ + t_starthold; // swing start
    t2_ = t1_ + t_swing; // full support start
    t3_ = t2_ + t_endhold; // end step

    // 0. contact

    // 1. swing foot trajectory
    foot_motion_data_.motion_period = t_swing;    
    motion_cmd.clear_and_add_motion(moving_foot_idx_,foot_motion_data_);

    ctrl_arch_->foot_trajectory_manager_
                ->setFootPosTrajectory(t1_ , &motion_cmd, x_ratio_height);
    // 2. joint trajectory
    ctrl_arch_->joint_trajectory_manager_
                ->setJointTrajectory(t0_, t3_, q_goal_);

    return true;
    
}

void MagnetoTrajectoryManager::update(const double& curr_time,
                                      Eigen::VectorXd& q,
                                      Eigen::VectorXd& dotq,
                                      Eigen::VectorXd& ddotq,
                                      bool& is_swing) {

    // update task
    task_list_.clear();
    if(curr_time < t0_){ // before start
        q = q_init_;
        dotq = dotq_init_;
        ddotq = Eigen::VectorXd::Zero(n_dim_);
        is_swing = false;
        return;
    }else if(curr_time < t1_){ // full support
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(joint_task_);
        updateContact(-1); //full contact
        is_swing = false;
    }else if(curr_time < t2_){ // swing
        ctrl_arch_->foot_trajectory_manager_->updateTask(curr_time, foot_task_list_[moving_foot_idx_]); 
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(foot_task_list_[moving_foot_idx_]);
        task_list_.push_back(joint_task_);
        updateContact(moving_foot_idx_); // swing contact
        is_swing = true;
    }else if(curr_time < t3_){ // support
        ctrl_arch_->joint_trajectory_manager_->updateTask(curr_time, joint_task_);
        task_list_.push_back(joint_task_);
        updateContact(-1); //full contact
        is_swing = false;
    }else{ // after end
        q = q_goal_;
        dotq = dotq_goal_;
        ddotq = Eigen::VectorXd::Zero(n_dim_);
        is_swing = false;
        return;
    }

    // std::cout<<"here 1 : contact=" << contact_list_.size() << ", task=" << task_list_.size() << std::endl;
    Eigen::VectorXd q_curr = robot_manager_->getQ();
    if(!task_list_.empty()) {
          kin_wbc_->FindFullConfiguration(q_curr, task_list_, contact_list_, 
                                        q, dotq, ddotq); 
    }
    Eigen::VectorXd f_err = foot_task_list_[moving_foot_idx_]->pos_err;
    robot_manager_->updateSystem(q, dotq, false);    
}

// void MagnetoTrajectoryManager::updateDdotQ(const Eigen::VectorXd& dotq_des_next,
//                                         const Eigen::VectorXd& dotq_des,
//                                         const Eigen::VectorXd& q_des,
//                                         const Eigen::VectorXd& dotq,
//                                         const Eigen::VectorXd& q,
//                                         const double& timestep,
//                                         Eigen::VectorXd& ddotq){

//     ddotq = 1./timestep * (dotq_des_next - 2.*dotq + dotq_des) 
//             - 1./timestep/timestep*(q - q_des);
// }

void MagnetoTrajectoryManager::updateContact(int moving_foot_idx){
    // contact
    contact_list_.clear();
    for(int foot_idx=0; foot_idx<Magneto::n_leg;++foot_idx){
        if(moving_foot_idx!=foot_idx)
        contact_list_.push_back(foot_contact_list_[foot_idx]);
    }
}