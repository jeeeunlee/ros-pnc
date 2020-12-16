#include <my_simulator/RosSim/MagnetoRosNode.hpp>
#include <my_ros_msgs/MsgConvertFuction.hpp>
#include <unistd.h>

MagnetoRosNode::MagnetoRosNode(ros::NodeHandle& nh, 
                            const SimulatorParameter& sim_param, 
                            const dart::simulation::WorldPtr& _world)
    : nh_(nh), dart::gui::osg::WorldNode(_world), count_(0), t_(0.0) {
    world_ = _world;

    // ---- GET SKELETON
    robot_ = world_->getSkeleton("magneto");
    ground_ = world_->getSkeleton("ground_skeleton");
    std::cout<< "Magneto mass : " <<  robot_->getMass() << std::endl;
    
    // ---- GET INFO FROM SKELETON
    // CheckRobotSkeleton(robot_);
    n_dof_ = robot_->getNumDofs();        
  
    // ---- SET POSITION LIMIT
    // setPositionLimitEnforced()

    // ---- SET INTERFACE
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand(); 

    cmd_call_back_count_ = 0;
    prev_call_back_count_ = -1;
    sub_cmd_ = nh.subscribe("/magneto_cmd", 1000, &MagnetoRosNode::MagnetoCmdCallBack, this);
    pub_data_ =  nh.advertise<my_ros_msgs::MagnetoSensorData>("/magneto_data", 1000);
    pub_interrupt_logic_ = nh.advertise<std_msgs::UInt16>("/interrupt_logic", 1000);

    // contact dinstance
    contact_threshold_ = 0.005;
    contact_distance_[MagnetoBodyNode::AL_foot_link] = 0.05;
    contact_distance_[MagnetoBodyNode::BL_foot_link] = 0.05;
    contact_distance_[MagnetoBodyNode::AR_foot_link] = 0.05;
    contact_distance_[MagnetoBodyNode::BR_foot_link] = 0.05;


    // (contact COP link name, contact link node)
    contact_bn_list_ = { {
        std::make_pair("AL_foot_link",robot_->getBodyNode("AL_foot_link_3")),
        std::make_pair("BL_foot_link",robot_->getBodyNode("BL_foot_link_3")),
        std::make_pair("AR_foot_link",robot_->getBodyNode("AR_foot_link_3")),
        std::make_pair("BR_foot_link",robot_->getBodyNode("BR_foot_link_3"))
    } };

    // 
    Eigen::MatrixXd R_ground = ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().linear();
    Eigen::VectorXd p_ground = ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().translation();
    R_gw_ = R_ground.transpose();
    p_gw_ = - R_gw_ * p_ground;
    ground_quat_ = Eigen::Quaterniond(ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().linear());

    servo_rate_ = sim_param.servo_rate_;
    kp_ = sim_param.kp_;
    kd_ = sim_param.kd_;

    b_plot_result_ = sim_param.b_plot_result_;
    magnetic_force_ = sim_param.magnetic_force_;
    residual_magnetism_ = sim_param.residual_magnetism_;

    // ---- SET TORQUE LIMIT
    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_); 
    trq_lb_ = Eigen::VectorXd::Constant(n_dof_, -sim_param.torque_limit_);
    trq_ub_ = Eigen::VectorXd::Constant(n_dof_, sim_param.torque_limit_); 
}

MagnetoRosNode::~MagnetoRosNode() {
    // delete interface_;
    delete sensor_data_;
    delete command_;
}

void MagnetoRosNode::MagnetoCmdCallBack(const my_ros_msgs::MagnetoCommandConstPtr cmd) {
    // command_msg_ -> command_    
    // command_msg_ = cmd;
    // ROS_INFO("MagnetoCmdCallBack");
    cmd_data_mutex_.lock();
    my_ros_msgs::JointData2Vector(cmd->joint_position.data,
                                    command_->q);
    my_ros_msgs::JointData2Vector(cmd->joint_velocity.data,
                                    command_->qdot);
    my_ros_msgs::JointData2Vector(cmd->joint_torque.data,
                                    command_->jtrq);    
    command_->b_magnetism_map.clear();
    for (auto &mag_onoff_msg : cmd->magnet_onoffs)
        command_->b_magnetism_map[mag_onoff_msg.link_idx] = mag_onoff_msg.onoff;
    cmd_data_mutex_.unlock();
    cmd_call_back_count_++;    
}

void MagnetoRosNode::CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel){
    size_t n_bodynode = skel->getNumBodyNodes();
    std::string bn_name;

    // check inertia
    Eigen::MatrixXd I_tmp;
    bool b_check;
    for(size_t idx = 0; idx<n_bodynode; ++idx){
        bn_name = skel->getBodyNode(idx)->getName();
        // I_tmp = skel->getBodyNode(idx)->getInertia().getMoment();
        // my_utils::pretty_print(I_tmp, std::cout, bn_name);
        // b_check=skel->getBodyNode(idx)->getInertia().verify(true, 1e-5);
        I_tmp = skel->getBodyNode(idx)->getArticulatedInertia();
        my_utils::pretty_print(I_tmp, std::cout, bn_name);
    }    
}

void MagnetoRosNode::customPostStep(){

}

void MagnetoRosNode::enableButtonFlag(uint16_t key) {
    std::cout << "button(" << (char)key << ") pressed handled @ MagnetoRosNode::enableButtonFlag" << std::endl;
    // ((MagnetoInterface*)interface_) -> interrupt_ -> setFlags(key);
    std_msgs::UInt16 key_msg;
    key_msg.data = key;
    pub_interrupt_logic_.publish(key_msg);
}


void MagnetoRosNode::waitCommand(){
    int wait_micro_sec = 10; // 0.01 ms
    int max_iter_count = 100; // 10 ms
    int iter_count = 0;
    while(prev_call_back_count_ == cmd_call_back_count_){        
        usleep(wait_micro_sec); // micro sec
        if(iter_count++ > max_iter_count){
            ROS_WARN("skip the command - count=%d, callback=%d", 
                                count_, cmd_call_back_count_);
            break;
        }
    }
    if(prev_call_back_count_ == cmd_call_back_count_){
        cmd_data_mutex_.lock();
        command_->q = sensor_data_->q;
        command_->qdot = sensor_data_->qdot;
        command_->jtrq = Eigen::VectorXd::Zero(Magneto::n_adof);
        command_->b_magnetism_map[MagnetoBodyNode::AL_foot_link]=true;
        command_->b_magnetism_map[MagnetoBodyNode::BL_foot_link]=true;
        command_->b_magnetism_map[MagnetoBodyNode::AR_foot_link]=true;
        command_->b_magnetism_map[MagnetoBodyNode::BR_foot_link]=true;
        cmd_data_mutex_.unlock();
    }
    prev_call_back_count_ = cmd_call_back_count_;
}

void MagnetoRosNode::customPreStep() {    

    t_ = (double)count_ * servo_rate_;

    clearSensorDataMsg();
    UpdateSensorDataMsg(); // update sensor_data_msg_
    if(prev_call_back_count_ < cmd_call_back_count_){
        prev_call_back_count_ = cmd_call_back_count_;
        pub_data_.publish(sensor_data_msg_); 
        count_++; 
        ROS_WARN("publish sensor_data_msg_");
    }

    // --------------------------------------------------------------
    //          COMPUTE COMMAND - desired joint acc/trq etc
    // --------------------------------------------------------------
    // ROS_INFO("waitCommand start");
    // ((MagnetoInterface*)interface_)->getCommand(sensor_data_, command_);
    // waitCommand();
    // ROS_INFO("waitCommand end");
    
    if (b_plot_result_) {
        // bool b_planner = ((MagnetoInterface*)interface_)->IsPlannerUpdated();
        // bool b_foot_planner = ((MagnetoInterface*)interface_)->IsFootPlannerUpdated();
        // if(b_planner || b_foot_planner) world_->removeAllSimpleFrames();
        // if (b_planner) PlotResult();        
        // if (b_foot_planner) PlotFootStepResult();        
    }
    // --------------------------------------------------------------

    cmd_data_mutex_.lock();
    trq_cmd_.setZero();
    for(int i=0; i< Magneto::n_adof; ++i) {
        trq_cmd_[Magneto::idx_adof[i]] 
                        = command_->jtrq[i] + 
                          kd_ * (command_->qdot[i] - sensor_data_->qdot[i]) +
                          kp_ * (command_->q[i] - sensor_data_->q[i]);
    }
    // spring in gimbal    
    // double ks = 1.0;// N/rad
    // for(int i=6; i< Magneto::n_vdof; ++i) {
    //     trq_cmd_[Magneto::idx_vdof[i]] = ks * ( 0.0 - sensor_data_->virtual_q[i]);
    // }
    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = kd_ * (command_->qdot[i] - sensor_data_->qdot[i]) +
    //                       kp_ * (command_->q[i] - sensor_data_->q[i]);
    // }  
    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = command_->jtrq[i];
    // }

    EnforceTorqueLimit();
    ApplyMagneticForce();
    robot_->setForces(trq_cmd_);

    // SAVE DATA
    my_utils::saveVector(sensor_data_->alf_wrench, "alf_wrench");
    my_utils::saveVector(sensor_data_->blf_wrench, "blf_wrench");
    my_utils::saveVector(sensor_data_->arf_wrench, "arf_wrench");
    my_utils::saveVector(sensor_data_->brf_wrench, "brf_wrench");

    Eigen::VectorXd trq_act_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);
    for(int i=0; i< Magneto::n_adof; ++i)
        trq_act_cmd[i] = trq_cmd_[Magneto::idx_adof[i]];
    
    my_utils::saveVector(trq_act_cmd, "trq_fb");
    my_utils::saveVector(command_->jtrq, "trq_ff");

    my_utils::saveVector(command_->q, "q_cmd");
    my_utils::saveVector(sensor_data_->q, "q_sen");
    cmd_data_mutex_.unlock();
   

    // count_++;
}

void MagnetoRosNode::EnforceTorqueLimit()  {
    for(int i=0; i< n_dof_; ++i) {
        trq_cmd_[i] = trq_cmd_[i] >  trq_lb_[i] ? trq_cmd_[i] : trq_lb_[i];
        trq_cmd_[i] = trq_cmd_[i] <  trq_ub_[i] ? trq_cmd_[i] : trq_ub_[i];
    }    
}

void MagnetoRosNode::ApplyMagneticForce()  {
    bool is_force_local = true;
    bool is_force_global = false;
    Eigen::Vector3d force_w = Eigen::VectorXd::Zero(3);   
    Eigen::Vector3d force = Eigen::VectorXd::Zero(3);   
    Eigen::Vector3d location = Eigen::VectorXd::Zero(3);
    double distance_ratio;
    double distance_constant = contact_threshold_ * 4.; // 0.045
    
    Eigen::Quaternion<double> quat_ground 
                            = Eigen::Quaternion<double>( 
                                ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().linear() );
    for(auto it : command_->b_magnetism_map) {
        if( it.second ) {
            force[2] = - magnetic_force_;
        } else {
            // distance 0->1 , infinite->0
            distance_ratio = distance_constant / (contact_distance_[it.first] + distance_constant);
            distance_ratio = distance_ratio*distance_ratio;
            force[2] = - distance_ratio*(residual_magnetism_/100.)*magnetic_force_;
            // std::cout<<"res: dist = "<<contact_distance_[it.first]<<", distance_ratio=" << distance_ratio << std::endl;
        }       
        force_w = quat_ground.toRotationMatrix() * force;
        robot_->getBodyNode(it.first)->addExtForce(force, location, is_force_local);
        // robot_->getBodyNode(it.first)->addExtForce(force_w, location, is_force_global);

        my_utils::saveVector(force, "force_" + robot_->getBodyNode(it.first)->getName() );
        // std::cout<< robot_->getBodyNode(it.first)->getName().c_str()
        //          << " : " << force_w.transpose() << std::endl;
        // std::cout << "--------------------------" << std::endl;
    }
}


void MagnetoRosNode::UpdateSensorDataMsg() {

    Eigen::VectorXd q = robot_->getPositions();
    Eigen::VectorXd qdot = robot_->getVelocities();

    for(int i=0; i< Magneto::n_adof; ++i) {
        sensor_data_->q[i] = q[Magneto::idx_adof[i]];
        sensor_data_->qdot[i] = qdot[Magneto::idx_adof[i]];
        sensor_data_msg_.active_joint_position.data.push_back(q[Magneto::idx_adof[i]]);
        sensor_data_msg_.active_joint_velocity.data.push_back(qdot[Magneto::idx_adof[i]]);
    }
    for(int i=0; i< Magneto::n_vdof; ++i) {
        sensor_data_->virtual_q[i] = q[Magneto::idx_vdof[i]];
        sensor_data_->virtual_qdot[i] = qdot[Magneto::idx_vdof[i]];
        sensor_data_msg_.virtual_joint_position.data.push_back(q[Magneto::idx_vdof[i]]);
        sensor_data_msg_.virtual_joint_velocity.data.push_back(qdot[Magneto::idx_vdof[i]]);
    }
    // update contact_distance_
    UpdateContactDistance();
    // update sensor_data_->b_foot_contact 
    UpdateContactSwitchData();
    // update sensor_data_->foot_wrench
    UpdateContactWrenchData();
}

void MagnetoRosNode::UpdateContactDistance() {
    // get normal distance from the ground link frame R_ground
    // p{ground} = R_gw * p{world} 
    Eigen::VectorXd alf = p_gw_ + R_gw_*robot_->getBodyNode("AL_foot_link") // COP frame node?
                                        ->getWorldTransform().translation();
    Eigen::VectorXd blf = p_gw_ + R_gw_*robot_->getBodyNode("BL_foot_link")
                                        ->getWorldTransform().translation();
    Eigen::VectorXd arf = p_gw_ + R_gw_*robot_->getBodyNode("AR_foot_link")
                                        ->getWorldTransform().translation();
    Eigen::VectorXd brf = p_gw_ + R_gw_*robot_->getBodyNode("BR_foot_link")
                                        ->getWorldTransform().translation();

    contact_distance_[MagnetoBodyNode::AL_foot_link] = fabs(alf[2]);
    contact_distance_[MagnetoBodyNode::BL_foot_link] = fabs(blf[2]);
    contact_distance_[MagnetoBodyNode::AR_foot_link] = fabs(arf[2]);
    contact_distance_[MagnetoBodyNode::BR_foot_link] = fabs(brf[2]);
}


void MagnetoRosNode::UpdateContactSwitchData() {

    sensor_data_msg_.alfoot_contact
        = contact_distance_[MagnetoBodyNode::AL_foot_link] < contact_threshold_;
    sensor_data_msg_.blfoot_contact
        = contact_distance_[MagnetoBodyNode::BL_foot_link] < contact_threshold_;
    sensor_data_msg_.arfoot_contact
        = contact_distance_[MagnetoBodyNode::AR_foot_link] < contact_threshold_;
    sensor_data_msg_.brfoot_contact
        = contact_distance_[MagnetoBodyNode::BR_foot_link] < contact_threshold_;   

    sensor_data_->alfoot_contact 
        = contact_distance_[MagnetoBodyNode::AL_foot_link] < contact_threshold_;
    sensor_data_->blfoot_contact
        = contact_distance_[MagnetoBodyNode::BL_foot_link] < contact_threshold_;    
    sensor_data_->arfoot_contact
        = contact_distance_[MagnetoBodyNode::AR_foot_link] < contact_threshold_;
    sensor_data_->brfoot_contact
        = contact_distance_[MagnetoBodyNode::BR_foot_link] < contact_threshold_;
    
}


void MagnetoRosNode::UpdateContactWrenchData() {

    // (contact wrench)
    std::vector<Eigen::VectorXd> wrench_local_list;
    std::vector<Eigen::VectorXd> wrench_global_list;  

    wrench_local_list.clear();    
    wrench_global_list.clear();
    // AL,BL,AR,BR
    for(int i=0; i<contact_bn_list_.size(); ++i) {
        wrench_local_list.push_back(Eigen::VectorXd::Zero(6));
        wrench_global_list.push_back(Eigen::VectorXd::Zero(6));
    }

    const dart::collision::CollisionResult& _result =
                            world_->getLastCollisionResult();

    for (const auto& contact : _result.getContacts()) 
    {
        for(int i=0; i<contact_bn_list_.size(); ++i) 
        {
            for (const auto& shapeNode :
                contact_bn_list_[i].second->getShapeNodesWith<dart::dynamics::CollisionAspect>()) 
            {
                Eigen::VectorXd w_c = Eigen::VectorXd::Zero(6);
                Eigen::Isometry3d T_wc = Eigen::Isometry3d::Identity();

                if ( shapeNode == contact.collisionObject1->getShapeFrame() )               
                    w_c.tail(3) = contact.force;                    
                else if( shapeNode == contact.collisionObject2->getShapeFrame())
                    w_c.tail(3) = - contact.force;                
                else { continue; }
                
                T_wc.translation() = contact.point;
                Eigen::Isometry3d T_wa =
                    robot_->getBodyNode(contact_bn_list_[i].first) // Foot COP Frame
                        ->getTransform(dart::dynamics::Frame::World());
                        
                Eigen::Isometry3d T_ca = T_wc.inverse() * T_wa;
                Eigen::Isometry3d T_cw = T_wc.inverse();

                Eigen::MatrixXd AdT_ca = dart::math::getAdTMatrix(T_ca);
                Eigen::MatrixXd AdT_cw = dart::math::getAdTMatrix(T_cw);

                Eigen::VectorXd w_a = AdT_ca.transpose() * w_c;
                Eigen::VectorXd w_w = AdT_cw.transpose() * w_c;
                
                wrench_local_list[i] += w_a;
                wrench_global_list[i] += w_w;
            }  
        }      
    }

    // my_ros_msgs::Wrench2Vector(my_ros_msgs::ForceWrench wrench, Eigen::VectorXd& vector)
    my_ros_msgs::Wrench2Vector(sensor_data_msg_.alf_wrench, wrench_local_list[0]);
    my_ros_msgs::Wrench2Vector(sensor_data_msg_.blf_wrench, wrench_local_list[1]);
    my_ros_msgs::Wrench2Vector(sensor_data_msg_.arf_wrench, wrench_local_list[2]);
    my_ros_msgs::Wrench2Vector(sensor_data_msg_.brf_wrench, wrench_local_list[3]);

    sensor_data_->alf_wrench = wrench_local_list[0];
    sensor_data_->blf_wrench = wrench_local_list[1];
    sensor_data_->arf_wrench = wrench_local_list[2];
    sensor_data_->brf_wrench = wrench_local_list[3];
}


void MagnetoRosNode::PlotResult() {

    // Eigen::VectorXd com_pos = Eigen::VectorXd::Zero(3);

    // ((MagnetoInterface*)interface_)-> GetOptimalCoM(com_pos);
    // Eigen::Isometry3d com_tf = Eigen::Isometry3d::Identity();
    // com_tf.translation() = com_pos;
    // std::vector <std::pair<double, Eigen::Vector3d>> feasible_com_list;
    // ((MagnetoInterface*)interface_)-> GetFeasibleCoM(feasible_com_list); 

    // // set color
    // Eigen::Vector4d feasible_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue
    // Eigen::Vector4d infeasible_color = Eigen::Vector4d(1.0, 0, 0.0, 1.0); // red

    // // set Shape
    // std::shared_ptr<dart::dynamics::PointCloudShape> feasible_shape =
    //     std::make_shared<dart::dynamics::PointCloudShape>(
    //         dart::dynamics::PointCloudShape(0.003));

    // std::shared_ptr<dart::dynamics::PointCloudShape> infeasible_shape =
    //     std::make_shared<dart::dynamics::PointCloudShape>(
    //         dart::dynamics::PointCloudShape(0.003));

    // std::shared_ptr<dart::dynamics::SphereShape> s_shape =
    //     std::make_shared<dart::dynamics::SphereShape>(
    //         dart::dynamics::SphereShape(0.01));


    // for (auto it = feasible_com_list.begin(); it != feasible_com_list.end(); it++) {
    //     if( (*it).first > 0.0 )
    //         feasible_shape->addPoint((*it).second);
    //     else
    //         infeasible_shape->addPoint((*it).second);
    // }
    // std::cout<<"PlotResult :Feasible Points:" << feasible_shape->getNumPoints()
    //  << ", Infeasible Points:" << infeasible_shape->getNumPoints() << std::endl;

    // // set Frame
    // dart::dynamics::SimpleFramePtr feasible_com_frame, infeasible_com_frame;
    // dart::dynamics::SimpleFramePtr optimal_com_frame;
    
    // feasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
    //     dart::dynamics::Frame::World(), "com_feasible");
    // infeasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
    //     dart::dynamics::Frame::World(), "com_infeasible");
    // optimal_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
    //     dart::dynamics::Frame::World(), "com_target", com_tf);
    
    // feasible_com_frame->setShape(feasible_shape);
    // feasible_com_frame->getVisualAspect(true)->setColor(feasible_color);
    // world_->addSimpleFrame(feasible_com_frame);

    // infeasible_com_frame->setShape(infeasible_shape);
    // infeasible_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    // world_->addSimpleFrame(infeasible_com_frame);

    // optimal_com_frame->setShape(s_shape);
    // optimal_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    // world_->addSimpleFrame(optimal_com_frame);

    
}


void MagnetoRosNode::PlotFootStepResult() {
    // world_->removeAllSimpleFrames();
    
    // Eigen::VectorXd foot_pos;
    // ((MagnetoInterface*)interface_)-> GetNextFootStep(foot_pos);
    // Eigen::Isometry3d foot_tf = robot_->getBodyNode("base_link")->getTransform();
    // foot_tf.translation() = foot_pos;  
    // my_utils::pretty_print(foot_pos, std::cout, "PlotFootStepResult_"); 

    // // set shape
    // dart::dynamics::BoxShapePtr foot_shape =
    //     std::make_shared<dart::dynamics::BoxShape>(
    //         dart::dynamics::BoxShape(Eigen::Vector3d(0.02, 0.02, 0.02))); // Eigen::Vector3d(0.02, 0.02, 0.001)

    // // set color
    // Eigen::Vector4d foot_step_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue

    // // set frame
    // dart::dynamics::SimpleFramePtr foot_frame;
    // foot_frame = std::make_shared<dart::dynamics::SimpleFrame>(
    //     dart::dynamics::Frame::World(), "next_foot", foot_tf);
    // foot_frame->setShape(foot_shape);
    // foot_frame->getVisualAspect(true)->setColor(foot_step_color);
    // world_->addSimpleFrame(foot_frame);
}


void MagnetoRosNode::clearSensorDataMsg(){
    sensor_data_msg_.active_joint_position.data.clear();
    sensor_data_msg_.active_joint_velocity.data.clear();
    sensor_data_msg_.virtual_joint_position.data.clear();
    sensor_data_msg_.virtual_joint_velocity.data.clear();

    sensor_data_msg_.alf_wrench.x = 0.0;
    sensor_data_msg_.alf_wrench.y = 0.0;
    sensor_data_msg_.alf_wrench.z = 0.0;
    sensor_data_msg_.alf_wrench.rx = 0.0;
    sensor_data_msg_.alf_wrench.ry = 0.0;
    sensor_data_msg_.alf_wrench.rz = 0.0;

    sensor_data_msg_.blf_wrench.x = 0.0;
    sensor_data_msg_.blf_wrench.y = 0.0;
    sensor_data_msg_.blf_wrench.z = 0.0;
    sensor_data_msg_.blf_wrench.rx = 0.0;
    sensor_data_msg_.blf_wrench.ry = 0.0;
    sensor_data_msg_.blf_wrench.rz = 0.0;

    sensor_data_msg_.arf_wrench.x = 0.0;
    sensor_data_msg_.arf_wrench.y = 0.0;
    sensor_data_msg_.arf_wrench.z = 0.0;
    sensor_data_msg_.arf_wrench.rx = 0.0;
    sensor_data_msg_.arf_wrench.ry = 0.0;
    sensor_data_msg_.arf_wrench.rz = 0.0;

    sensor_data_msg_.brf_wrench.x = 0.0;
    sensor_data_msg_.brf_wrench.y = 0.0;
    sensor_data_msg_.brf_wrench.z = 0.0;
    sensor_data_msg_.brf_wrench.rx = 0.0;
    sensor_data_msg_.brf_wrench.ry = 0.0;
    sensor_data_msg_.brf_wrench.rz = 0.0;

    sensor_data_msg_.alfoot_contact = false;
    sensor_data_msg_.blfoot_contact = false;
    sensor_data_msg_.arfoot_contact = false;
    sensor_data_msg_.brfoot_contact = false;

    sensor_data_msg_.ground_rot.x = ground_quat_.x();
    sensor_data_msg_.ground_rot.y = ground_quat_.y();
    sensor_data_msg_.ground_rot.z = ground_quat_.z();
    sensor_data_msg_.ground_rot.w = ground_quat_.w();

}