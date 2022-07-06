#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>
#include <pnc_utils/../../Configuration.h>
#include <magneto_pnc/magneto_interface.hpp>

#include <magneto_pnc/magneto_interface.hpp>

#include <magneto_test/hwtest/magneto_interface_node.hpp>


MagnetoInterfaceParams::MagnetoInterfaceParams(){
    init();
}


MagnetoInterfaceNode::MagnetoInterfaceNode(){

    system_state_ = SUSPENDED;
    params_.init();

    // ---- ADVERTISERS FOR CONTROLLER
    desired_joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/desired_joint_states", 1);
    // adhesion_publisher_ = nh_.advertise<>("/", 1);

    // ---- SUBSCRIBER TO HARDWARE TOPICS
    joint_state_subscriber_ = nh_.subscribe("/joint_states", 100, &MagnetoInterfaceNode::jointStateCallback, this);
    imu_state_subscriber_ = nh_.subscribe("/magneto/imu_raw/data", 100, &MagnetoInterfaceNode::imuStateCallback, this);
    // adhesion_subscriber_ = nh_.subscribe("/", 100, &MagnetoInterfaceNode::imuStateCallback, this);

    // ---- SET INTERFACE
    interface_ = new MagnetoInterface();
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand();

    // Initialize

    qa_config_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_dot_ = Eigen::VectorXd::Zero(Magneto::n_adof);

    qa_config_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_dot_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);

    contact_ = {false, false, false, false};
    ft_sensor_ = {Eigen::VectorXd::Zero(6), 
                    Eigen::VectorXd::Zero(6), 
                    Eigen::VectorXd::Zero(6), 
                    Eigen::VectorXd::Zero(6)};
    adhesion_cmd_ = {true, true, true, true};

    
    // ---- SET control parameters %% motion script
    // run_mode_ = ((MagnetoInterface*)interface_)->getRunMode();
}

MagnetoInterfaceNode::~MagnetoInterfaceNode(){
    ROS_INFO("shutting_down Magneto_hw_node");
    delete interface_;
    delete sensor_data_;
    delete command_;
    delete imu_data_;
}


void MagnetoInterfaceNode::jointStateCallback(const sensor_msgs::JointState &joint_states){
    
    bool get_effort_values = (joint_states.effort.size() != 0);
    bool get_velocity_values = (joint_states.velocity.size() != 0);

    // number of joint state
    if(joint_states.name.size() == Magneto::n_adof)
    {
        ROS_ASSERT(joint_states.name.size() != Magneto::n_adof);
        for(int i(0); i<Magneto::n_adof; ++i){
            qa_names_[i] = joint_states.name[i];
            qa_config_[i] = joint_states.position[i];
            qa_dot_[i] = joint_states.velocity[i];
            qa_effort_[i] = joint_states.effort[i];
        }
    }
}

void MagnetoInterfaceNode::imuStateCallback(const sensor_msgs::Imu &data){
  if (system_state_ != SUSPENDED)
  {
    imu_data_->orientation 
        = Eigen::Quaternion<double>(data.orientation.w, 
                                    data.orientation.x, 
                                    data.orientation.y, 
                                    data.orientation.z);
    imu_data_->angular_velocity << data.angular_velocity.x, 
                                    data.angular_velocity.y, 
                                    data.angular_velocity.z;
    imu_data_->linear_acceleration << data.linear_acceleration.x,
                                    data.linear_acceleration.y, 
                                    data.linear_acceleration.z;
  }
}

void MagnetoInterfaceNode::loop(){

    // joint states
    sensor_data_->q = qa_config_;
    sensor_data_->qdot = qa_dot_;

    // imu states
    sensor_data_->imu_data = *imu_data_;

    // contact states
    sensor_data_->alfoot_contact = contact_[MagnetoFoot::AL];
    sensor_data_->blfoot_contact = contact_[MagnetoFoot::BL];
    sensor_data_->arfoot_contact = contact_[MagnetoFoot::AR];
    sensor_data_->brfoot_contact = contact_[MagnetoFoot::BR];

    // F/T sensor data
    sensor_data_->alf_wrench = ft_sensor_[MagnetoFoot::AL];
    sensor_data_->arf_wrench = ft_sensor_[MagnetoFoot::AR];
    sensor_data_->blf_wrench = ft_sensor_[MagnetoFoot::BL];
    sensor_data_->brf_wrench = ft_sensor_[MagnetoFoot::BR];

    // PnC
    interface_->getCommand(sensor_data_, command_);  

    // Save Data
    qa_config_des_ = command_->q;
    qa_dot_des_ = command_->qdot;
    adhesion_cmd_ = command_->magnetism_onoff;
}



void MagnetoInterfaceNode::publishMagnetOnoff(){
    //
    // adhesion_cmd_[MagnetoFoot::AL]    
}

void MagnetoInterfaceNode::publishDesiredJointState(){
    sensor_msgs::JointState joint_state_msg;

    joint_state_msg.header.stamp = ros::Time::now();    
    for (int i(0); i<Magneto::n_adof; ++i)
    {        
        joint_state_msg.name.push_back(qa_names_[i]);
        joint_state_msg.position.push_back(qa_config_des_[i]);
        joint_state_msg.velocity.push_back(qa_dot_des_[i]);
        joint_state_msg.effort.push_back(qa_effort_[i]);
    }
    desired_joint_state_publisher_.publish(joint_state_msg);
}