#include <my_simulator/RosSim/MagnetoRosNode.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>


MagnetoRosNode::MagnetoRosNode(ros::NodeHandle& nh, const dart::simulation::WorldPtr& _world)
    : nh_(nh), dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_rate_(0) {
    world_ = _world;

    // ---- GET SKELETON
    robot_ = world_->getSkeleton("magneto");
    ground_ = world_->getSkeleton("ground_skeleton");
    std::cout<< "Magneto mass : " <<  robot_->getMass() << std::endl;
    
    // ---- GET INFO FROM SKELETON
    // CheckRobotSkeleton(robot_);
    n_dof_ = robot_->getNumDofs();        

    // ---- PLOT?
    b_plot_result_ = true;
   
    // ---- SET POSITION LIMIT
    // setPositionLimitEnforced

    // contact dinstance
    contact_threshold_ = 0.005;
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        contact_distance_[foot_idx] = 0.05;
    }

    trq_cmd_ = Eigen::VectorXd::Zero(n_dof_);

    // ---- SET INTERFACE
    interface_ = new MagnetoInterface();
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand();

    sensor_data_->R_ground = ground_->getBodyNode("ground_link")
                                    ->getWorldTransform().linear();

    // ---- SET control parameters %% motion script
    SetParams_();

    // ---- SET TORQUE LIMIT
    trq_lb_ = Eigen::VectorXd::Constant(n_dof_, -torque_limit_);
    trq_ub_ = Eigen::VectorXd::Constant(n_dof_, torque_limit_);
    // trq_lb_ = robot_->getForceLowerLimits();
    // trq_ub_ = robot_->getForceUpperLimits();
    // std::cout<<"trq_lb_ = "<<trq_lb_.transpose() << std::endl;
    // std::cout<<"trq_ub_ = "<<trq_ub_.transpose() << std::endl;  
}

MagnetoRosNode::~MagnetoRosNode() {
    delete interface_;
    delete sensor_data_;
    delete command_;
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
    ((MagnetoInterface*)interface_) -> interrupt_ -> setFlags(key);    
}

void MagnetoRosNode::customPreStep() {

    static Eigen::VectorXd qInit = robot_->getPositions();

    t_ = (double)count_ * servo_rate_;

    Eigen::VectorXd q = robot_->getPositions();
    Eigen::VectorXd qdot = robot_->getVelocities();

    for(int i=0; i< Magneto::n_adof; ++i) {
        sensor_data_->q[i] = q[Magneto::idx_adof[i]];
        sensor_data_->qdot[i] = qdot[Magneto::idx_adof[i]];
    }

    for(int i=0; i< Magneto::n_vdof; ++i) {
        sensor_data_->virtual_q[i] = q[Magneto::idx_vdof[i]];
        sensor_data_->virtual_qdot[i] = qdot[Magneto::idx_vdof[i]];
    }
    // update contact_distance_
    UpdateContactDistance_();
    // update sensor_data_->b_foot_contact 
    UpdateContactSwitchData_();
    // update sensor_data_->foot_wrench
    UpdateContactWrenchData_();

    // --------------------------------------------------------------
    //          COMPUTE COMMAND - desired joint acc/trq etc
    // --------------------------------------------------------------
    CheckInterrupt_();
    // ROS_INFO("getCommand start");
    ((MagnetoInterface*)interface_)->getCommand(sensor_data_, command_);
    // ROS_INFO("getCommand end");
    
    if (b_plot_result_) {
        bool b_planner = ((MagnetoInterface*)interface_)->IsPlannerUpdated();
        bool b_foot_planner = ((MagnetoInterface*)interface_)->IsFootPlannerUpdated();
        if(b_planner || b_foot_planner) world_->removeAllSimpleFrames();
        if (b_planner) PlotResult_();        
        if (b_foot_planner) PlotFootStepResult_();        
    }
    // --------------------------------------------------------------

    trq_cmd_.setZero();
    // spring in gimbal
    
    // double ks = 1.0;// N/rad
    // for(int i=6; i< Magneto::n_vdof; ++i) {
    //     trq_cmd_[Magneto::idx_vdof[i]] = ks * ( 0.0 - sensor_data_->virtual_q[i]);
    // }

    for(int i=0; i< Magneto::n_adof; ++i) {
        trq_cmd_[Magneto::idx_adof[i]] 
                        = command_->jtrq[i] + 
                          kd_ * (command_->qdot[i] - sensor_data_->qdot[i]) +
                          kp_ * (command_->q[i] - sensor_data_->q[i]);
    }

    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = kd_ * (command_->qdot[i] - sensor_data_->qdot[i]) +
    //                       kp_ * (command_->q[i] - sensor_data_->q[i]);
    // }
    
    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = command_->jtrq[i] + 
    //                       kd_ * ( - sensor_data_->qdot[i]) +
    //                       kp_ * (command_->q[i] - sensor_data_->q[i]);
    // }

    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = command_->jtrq[i];
    // }

    // for(int i=0; i< Magneto::n_adof; ++i) {
    //     trq_cmd_[Magneto::idx_adof[i]] 
    //                     = kd_ * ( 0.0 - sensor_data_->qdot[i]) +
    //                       kp_ * ( qInit[Magneto::idx_adof[i]] - sensor_data_->q[i]);
    // }

    EnforceTorqueLimit();
    ApplyMagneticForce();
    robot_->setForces(trq_cmd_);   

    // SAVE DATA
    Eigen::VectorXd trq_act_cmd = Eigen::VectorXd::Zero(Magneto::n_adof);
    for(int i=0; i< Magneto::n_adof; ++i)
        trq_act_cmd[i] = trq_cmd_[Magneto::idx_adof[i]];
    


    count_++;
}

void MagnetoRosNode::CheckInterrupt_() {
    // this moved to enableButton
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
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        if(command_->b_foot_magnetism_on[foot_idx]){
            force[2] = - magnetic_force_;
        }
        else{
            distance_ratio = distance_constant / (contact_distance_[foot_idx] + distance_constant);
            distance_ratio = distance_ratio*distance_ratio;
            force[2] = - distance_ratio*(residual_magnetism_/100.)*magnetic_force_;
        }
        force_w = quat_ground.toRotationMatrix() * force;
        robot_->getBodyNode(MagnetoFoot::LinkIdx[foot_idx])
                ->addExtForce(force, location, is_force_local);
        // robot_->getBodyNode(MagnetoFoot::LinkIdx[foot_idx])-
        //          >addExtForce(force_w, location, is_force_global); 
    }
}

void MagnetoRosNode::PlotResult_() {

    Eigen::VectorXd com_pos = Eigen::VectorXd::Zero(3);

    ((MagnetoInterface*)interface_)-> GetOptimalCoM(com_pos);
    Eigen::Isometry3d com_tf = Eigen::Isometry3d::Identity();
    com_tf.translation() = com_pos;
    std::vector <std::pair<double, Eigen::Vector3d>> feasible_com_list;
    ((MagnetoInterface*)interface_)-> GetFeasibleCoM(feasible_com_list); 

    // set color
    Eigen::Vector4d feasible_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue
    Eigen::Vector4d infeasible_color = Eigen::Vector4d(1.0, 0, 0.0, 1.0); // red

    // set Shape
    std::shared_ptr<dart::dynamics::PointCloudShape> feasible_shape =
        std::make_shared<dart::dynamics::PointCloudShape>(
            dart::dynamics::PointCloudShape(0.003));

    std::shared_ptr<dart::dynamics::PointCloudShape> infeasible_shape =
        std::make_shared<dart::dynamics::PointCloudShape>(
            dart::dynamics::PointCloudShape(0.003));

    std::shared_ptr<dart::dynamics::SphereShape> s_shape =
        std::make_shared<dart::dynamics::SphereShape>(
            dart::dynamics::SphereShape(0.01));


    for (auto it = feasible_com_list.begin(); it != feasible_com_list.end(); it++) {
        if( (*it).first > 0.0 )
            feasible_shape->addPoint((*it).second);
        else
            infeasible_shape->addPoint((*it).second);
    }
    std::cout<<"PlotResult :Feasible Points:" << feasible_shape->getNumPoints()
     << ", Infeasible Points:" << infeasible_shape->getNumPoints() << std::endl;

    // set Frame
    dart::dynamics::SimpleFramePtr feasible_com_frame, infeasible_com_frame;
    dart::dynamics::SimpleFramePtr optimal_com_frame;
    
    feasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_feasible");
    infeasible_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_infeasible");
    optimal_com_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "com_target", com_tf);
    
    feasible_com_frame->setShape(feasible_shape);
    feasible_com_frame->getVisualAspect(true)->setColor(feasible_color);
    world_->addSimpleFrame(feasible_com_frame);

    infeasible_com_frame->setShape(infeasible_shape);
    infeasible_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    world_->addSimpleFrame(infeasible_com_frame);

    optimal_com_frame->setShape(s_shape);
    optimal_com_frame->getVisualAspect(true)->setColor(infeasible_color);
    world_->addSimpleFrame(optimal_com_frame);

    
}


void MagnetoRosNode::PlotFootStepResult_() {
    world_->removeAllSimpleFrames();
    
    Eigen::VectorXd foot_pos;
    ((MagnetoInterface*)interface_)-> GetNextFootStep(foot_pos);
    Eigen::Isometry3d foot_tf = robot_->getBodyNode("base_link")->getTransform();
    foot_tf.translation() = foot_pos;  
    my_utils::pretty_print(foot_pos, std::cout, "PlotFootStepResult_"); 

    // set shape
    dart::dynamics::BoxShapePtr foot_shape =
        std::make_shared<dart::dynamics::BoxShape>(
            dart::dynamics::BoxShape(Eigen::Vector3d(0.02, 0.02, 0.02))); // Eigen::Vector3d(0.02, 0.02, 0.001)

    // set color
    Eigen::Vector4d foot_step_color = Eigen::Vector4d(0.0, 0.0, 1.0, 1.0); // blue

    // set frame
    dart::dynamics::SimpleFramePtr foot_frame;
    foot_frame = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "next_foot", foot_tf);
    foot_frame->setShape(foot_shape);
    foot_frame->getVisualAspect(true)->setColor(foot_step_color);
    world_->addSimpleFrame(foot_frame);
}

void MagnetoRosNode::SetParams_() {
    try {
        YAML::Node simulation_cfg;
        simulation_cfg = YAML::LoadFile(THIS_COM "config/Magneto/SIMULATIONWALK.yaml");            

        my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
        my_utils::readParameter(simulation_cfg["control_configuration"], "torque_limit", torque_limit_);
        my_utils::readParameter(simulation_cfg, "plot_result", b_plot_result_);        

        // setting magnetic force
        my_utils::readParameter(simulation_cfg, "magnetic_force", magnetic_force_);  
        my_utils::readParameter(simulation_cfg, "residual_magnetism", residual_magnetism_);  
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

void MagnetoRosNode::UpdateContactDistance_() {
    // get normal distance from the ground link frame R_ground
    // p{ground} = R_gw * p{world} 
    Eigen::MatrixXd R_ground = ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().linear();
    Eigen::VectorXd p_ground = ground_->getBodyNode("ground_link")
                                        ->getWorldTransform().translation();

    Eigen::MatrixXd R_gw = R_ground.transpose();
    Eigen::MatrixXd p_gw = - R_gw * p_ground;

    Eigen::VectorXd alf = p_gw + R_gw*robot_->getBodyNode("AL_foot_link") // COP frame node?
                            ->getWorldTransform().translation();
    Eigen::VectorXd blf = p_gw + R_gw*robot_->getBodyNode("BL_foot_link")
                            ->getWorldTransform().translation();
    Eigen::VectorXd arf = p_gw + R_gw*robot_->getBodyNode("AR_foot_link")
                             ->getWorldTransform().translation();
    Eigen::VectorXd brf = p_gw + R_gw*robot_->getBodyNode("BR_foot_link")
                             ->getWorldTransform().translation();

    Eigen::VectorXd dist;
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        dist = p_gw + R_gw*robot_->getBodyNode(MagnetoFoot::LinkIdx[foot_idx])
                                ->getWorldTransform().translation();
        contact_distance_[foot_idx]= fabs(dist[2]);
    } 
}


void MagnetoRosNode::UpdateContactSwitchData_() {
    
    // TODO : distance base -> force base ?  
    // TODO : distance base -> force base ? 
    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        sensor_data_->b_foot_contact[foot_idx] = 
            contact_distance_[MagnetoFoot::LinkIdx[foot_idx]] < contact_threshold_;
    }

    // footname_contact_onoff
    // my_utils::saveValue((double)(sensor_data_->b_foot_contact[foot_idx]), "");  


}


void MagnetoRosNode::UpdateContactWrenchData_() {

    // (contact COP link name, contact link node)
    std::vector<std::pair<std::string,dart::dynamics::BodyNode*>> bn_list;

    bn_list.clear();
    bn_list.push_back(std::make_pair("AL_foot_link",robot_->getBodyNode("AL_foot_link_3")));
    bn_list.push_back(std::make_pair("BL_foot_link",robot_->getBodyNode("BL_foot_link_3")));
    bn_list.push_back(std::make_pair("AR_foot_link",robot_->getBodyNode("AR_foot_link_3")));
    bn_list.push_back(std::make_pair("BR_foot_link",robot_->getBodyNode("BR_foot_link_3")));

    // (contact wrench)
    std::vector<Eigen::Vector6d> wrench_local_list;
    std::vector<Eigen::Vector6d> wrench_global_list;  

    wrench_local_list.clear();    
    wrench_global_list.clear();
    for(int i=0; i<bn_list.size(); ++i) {
        wrench_local_list.push_back(Eigen::VectorXd::Zero(6));
        wrench_global_list.push_back(Eigen::VectorXd::Zero(6));
    }

    const dart::collision::CollisionResult& _result =
                            world_->getLastCollisionResult();

    for (const auto& contact : _result.getContacts()) 
    {
        for(int i=0; i<bn_list.size(); ++i) 
        {
            for (const auto& shapeNode :
                bn_list[i].second->getShapeNodesWith<dart::dynamics::CollisionAspect>()) 
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
                    robot_->getBodyNode(bn_list[i].first) // Foot COP Frame
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

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        sensor_data_->foot_wrench[foot_idx] = wrench_local_list[foot_idx];
        // sensor_data_->foot_wrench[foot_idx] = wrench_global_list[i];
    }    
}
