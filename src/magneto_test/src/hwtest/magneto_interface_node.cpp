#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>
#include <pnc_utils/../../Configuration.h>
#include <magneto_pnc/magneto_interface.hpp>

#include <magneto_pnc/magneto_interface.hpp>

#include <magneto_test/hwtest/magneto_interface_node.hpp>



MagnetoInterfaceNode::MagnetoInterfaceNode(){

    // ---- ADVERTISERS FOR CONTROLLER
    desired_joint_state_publisher_ = nh_.advertise<sensor_msgs::JointState>("/desired_joint_states", 1);
    al_adhesion_publisher_ = nh_.advertise<std_msgs::Int8>("/AL_adhesion_control", 1);
    bl_adhesion_publisher_ = nh_.advertise<std_msgs::Int8>("/BL_adhesion_control", 1);
    ar_adhesion_publisher_ = nh_.advertise<std_msgs::Int8>("/AR_adhesion_control", 1);
    br_adhesion_publisher_ = nh_.advertise<std_msgs::Int8>("/BR_adhesion_control", 1);

    // ---- SUBSCRIBER TO HARDWARE TOPICS
    joypad_subscriber_ = nh_.subscribe("/joy", 100, &MagnetoInterfaceNode::joyCallback, this);
    joint_state_subscriber_ = nh_.subscribe("/syropod/joint_states", 2, &MagnetoInterfaceNode::jointStateCallback, this);
    imu_state_subscriber_ = nh_.subscribe("/magneto/imu_raw/data", 10, &MagnetoInterfaceNode::imuStateCallback, this);

    al_adhesion_subscriber_ = nh_.subscribe("/AL_magnet_state", 100, &MagnetoInterfaceNode::alAdhesionStateCallback, this);
    bl_adhesion_subscriber_ = nh_.subscribe("/BL_magnet_state", 100, &MagnetoInterfaceNode::blAdhesionStateCallback, this);
    ar_adhesion_subscriber_ = nh_.subscribe("/AR_magnet_state", 100, &MagnetoInterfaceNode::arAdhesionStateCallback, this);
    br_adhesion_subscriber_ = nh_.subscribe("/BR_magnet_state", 100, &MagnetoInterfaceNode::brAdhesionStateCallback, this);

    al_proximity_subscriber_ = nh_.subscribe("/AL_proximity_sense", 100, &MagnetoInterfaceNode::alProximitySenseCallback, this);
    bl_proximity_subscriber_ = nh_.subscribe("/BL_proximity_sense", 100, &MagnetoInterfaceNode::blProximitySenseCallback, this);
    ar_proximity_subscriber_ = nh_.subscribe("/AR_proximity_sense", 100, &MagnetoInterfaceNode::arProximitySenseCallback, this);
    br_proximity_subscriber_ = nh_.subscribe("/BR_proximity_sense", 100, &MagnetoInterfaceNode::brProximitySenseCallback, this);

    // ---- SET INTERFACE
    interface_ = new MagnetoInterface();
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand();

    // ---- INITIALIZE STATE VARIABLES
    system_state_ = SUSPENDED;
    joint_state_initialized_ = false;
    imu_state_initialized_ = false;   
    b_stop_ = false;

    imu_data_ = new IMUSensorData();
    qa_config_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_dot_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_stop_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_effort_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_config_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    qa_dot_des_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    
    ft_sensor_ = {Eigen::VectorXd::Zero(6), // Initialize
                    Eigen::VectorXd::Zero(6), 
                    Eigen::VectorXd::Zero(6), 
                    Eigen::VectorXd::Zero(6)};
    adhesion_cmd_ = {true, true, true, true};

    
    // ---- Joystick
    debounce_startbutton_ = true;
    startbutton_pressed = false;
    debounce_stopbutton_ = true;
    stopbutton_pressed = false;

    // index
    
    // magneto_actuator_['AL_coxa_joint']
    // magneto_actuator_['AL_femur_joint']
    // magneto_actuator_['AL_tibia_joint']
    // magneto_actuator_['AR_coxa_joint']
    // magneto_actuator_['AR_femur_joint']
    // magneto_actuator_[]
    // magneto_actuator_[]
    // magneto_actuator_[]
    // magneto_actuator_[]
    // magneto_actuator_[]

   
}

MagnetoInterfaceNode::~MagnetoInterfaceNode(){
    ROS_INFO("shutting_down Magneto_hw_node");
    delete interface_;
    delete sensor_data_;
    delete command_;
    delete imu_data_;
}

void MagnetoInterfaceNode::setSensorData(){
    // joint states
    // ROS_INFO("setSensorData - q, qdot");
    for(int i(0); i<Magneto::n_adof; ++i){       
        sensor_data_->q[magneto_actuator_[qa_names_[i]]] = qa_config_[i];
        sensor_data_->qdot[magneto_actuator_[qa_names_[i]]] = qa_dot_[i];
    }

    // imu states
    // ROS_INFO("setSensorData - imu");
    sensor_data_->imu_data = *imu_data_;

    // contact states
    // ROS_INFO("setSensorData - states");
    sensor_data_->alfoot_contact = AL_contact_state_;
    sensor_data_->blfoot_contact = BL_contact_state_;
    sensor_data_->arfoot_contact = AR_contact_state_;
    sensor_data_->brfoot_contact = BR_contact_state_;

    // F/T sensor data
    // ROS_INFO("setSensorData - ft_sensor_");
    sensor_data_->alf_wrench = ft_sensor_[MagnetoFoot::AL];
    sensor_data_->arf_wrench = ft_sensor_[MagnetoFoot::AR];
    sensor_data_->blf_wrench = ft_sensor_[MagnetoFoot::BL];
    sensor_data_->brf_wrench = ft_sensor_[MagnetoFoot::BR];
}

bool MagnetoInterfaceNode::systemInitialized() {

    if(!joint_state_initialized_)
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "joint state is not initialized yet");
    if(!imu_state_initialized_)
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "imu state is not initialized yet");

    if(joint_state_initialized_ && imu_state_initialized_){
        ROS_INFO("both state is initialized!!");
        setSensorData();
        interface_->initialize(sensor_data_, command_);
        system_state_ = CHECKED;
        return true;
    }
    else{
        ROS_INFO_THROTTLE(THROTTLE_PERIOD,"joint_state_initialized_,imu_state_initialized_ =%d, %d", 
                        joint_state_initialized_, imu_state_initialized_);
        system_state_ = SUSPENDED;
        return false;
    }
        
}

bool MagnetoInterfaceNode::resetEstimator(){

    setSensorData();

    // reset Estimator at the current position
    if( interface_->resetEstimator(sensor_data_, command_) ){
        system_state_ = READY;
        return true;
    }
    else{
        system_state_ = IDLE;
        return false;
    }    
}

bool MagnetoInterfaceNode::initializeJointConfiguration(){
    setSensorData();
    interface_->getInitialCommand(sensor_data_, command_);

    // save data
    for(int i(0); i<Magneto::n_adof; ++i){       
        qa_config_des_[i] = command_->q[magneto_actuator_[qa_names_[i]]];
        qa_dot_des_[i] = command_->qdot[magneto_actuator_[qa_names_[i]]];        
    }
    
    // adhesion_cmd_ = command_->magnetism_onoff;
    for(int i(0);i<Magneto::n_leg;++i)
        adhesion_cmd_[i] = 1;

    // check initial position
    if ( interface_->checkInitialJointConfiguration() ){
        ROS_INFO("Reached intial joint configuration!");
        system_state_ = IDLE;
        return true;
    } else return false;    
}

void MagnetoInterfaceNode::loop(){

    b_stop_ = false;
    // set sensor_data_
    setSensorData();

    // PnC
    interface_->getCommand(sensor_data_, command_);  

    // Save Data
    for(int i(0); i<Magneto::n_adof; ++i){       
        qa_config_des_[i] = command_->q[magneto_actuator_[qa_names_[i]]];
        qa_dot_des_[i] = command_->qdot[magneto_actuator_[qa_names_[i]]];        
    }
    adhesion_cmd_ = command_->magnetism_onoff;

    if( interface_->checkSystemIdle() )
        system_state_ = IDLE;    
}

void MagnetoInterfaceNode::stoploop(){
    // set sensor_data_
    setSensorData();

    // PnC
    interface_->getStopCommand(sensor_data_, command_); 
    if(b_stop_) command_->q = qa_stop_;

    // Save Data
    for(int i(0); i<Magneto::n_adof; ++i){
        qa_config_des_[i] = command_->q[magneto_actuator_[qa_names_[i]]];
        qa_dot_des_[i] = command_->qdot[magneto_actuator_[qa_names_[i]]];        
    }
    for(int i(0);i<Magneto::n_leg;++i)
        adhesion_cmd_[i] = 1;
}

void MagnetoInterfaceNode::enableButtonFlag(uint16_t key){
    // interface interrupt
    interface_-> interrupt_ -> setFlags(key);
    interface_-> interrupt_-> processInterrupts();
    // if( interface_->getNumStates() > 0 ){
        system_state_ = RUNNING;
    // }
}

void MagnetoInterfaceNode::setStop(){
    b_stop_ = true;
    qa_stop_ = sensor_data_->q;
    interface_-> interrupt_ -> setFlags('t');
    interface_-> interrupt_-> processInterrupts();
    system_state_= STOP;
}

void MagnetoInterfaceNode::releaseStop(){
    b_stop_ = false;
    system_state_ = IDLE;
}

void MagnetoInterfaceNode::publishMagnetOnoff(){
    
    static bool check_publsih_magnet_called = false;
    if(!check_publsih_magnet_called){
        std::cout<<"check_publsih_magnet_called is called "<< std::endl;
        check_publsih_magnet_called = true;
    }

    std_msgs::Int8 adhesion_control_msg;

    adhesion_control_msg.data = adhesion_cmd_[MagnetoFoot::AL];
    al_adhesion_publisher_.publish(adhesion_control_msg);
    
    adhesion_control_msg.data = adhesion_cmd_[MagnetoFoot::AR];
    ar_adhesion_publisher_.publish(adhesion_control_msg);

    adhesion_control_msg.data = adhesion_cmd_[MagnetoFoot::BL];
    bl_adhesion_publisher_.publish(adhesion_control_msg);

    adhesion_control_msg.data = adhesion_cmd_[MagnetoFoot::BR];
    br_adhesion_publisher_.publish(adhesion_control_msg);

    // check magnetism
    // if( !adhesion_cmd_[MagnetoFoot::AL] || !adhesion_cmd_[MagnetoFoot::AR] || 
    //     !adhesion_cmd_[MagnetoFoot::BL] || !adhesion_cmd_[MagnetoFoot::BR] ){
    //         std::cout << "adhesion_cmd_ = " 
    //         << adhesion_cmd_[0] << ", " << adhesion_cmd_[1] << ", "
    //         << adhesion_cmd_[2] << ", " << adhesion_cmd_[3] << std::endl;
    // }
    
}

void MagnetoInterfaceNode::publishDesiredJointState(){
    
    static bool check_publsih_joint_state_called = false;
    if(!check_publsih_joint_state_called){
        std::cout<<"check_publsih_joint_state_called is called "<< std::endl;
        check_publsih_joint_state_called = true;
    }

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


void MagnetoInterfaceNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    static bool check_callback_joy_called = false;
    if(!check_callback_joy_called){
        std::cout<<"check_callback_joy_called is called "<< std::endl;
        check_callback_joy_called = true;
    }

    joypad_control_ = *joy;

    if(debounce_startbutton_) // set to true once the work is done
    {
        startbutton_pressed = joypad_control_.buttons[JoypadButtonIndex::START];
        if(startbutton_pressed) debounce_startbutton_ = false;
    }

    if(debounce_stopbutton_)
    {
        stopbutton_pressed = joypad_control_.buttons[JoypadButtonIndex::BACK];
        if(stopbutton_pressed) debounce_stopbutton_ = false;        
    }

    std::cout<< "debounce_startbutton_" << debounce_startbutton_
        << ", startbutton_pressed = "<< startbutton_pressed <<std::endl;

    std::cout<< "debounce_stopbutton_" << debounce_stopbutton_
        << ", stopbutton_pressed = "<< stopbutton_pressed <<std::endl;
        
    
}

void MagnetoInterfaceNode::jointStateCallback(const sensor_msgs::JointState &joint_states){
    
    static bool check_joint_state_call_back = false;
    if(!check_joint_state_call_back){
        std::cout<<"chcek_joint_state_call_back is called"<< std::endl;
        check_joint_state_call_back = true;
    }

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
        joint_state_initialized_ = true;
    }else
        std::cout<<"joint_states.name.size() = "<<joint_states.name.size()<<std::endl;
}

void MagnetoInterfaceNode::imuStateCallback(const sensor_msgs::Imu &data){

    static bool check_imu_state_call_back = false;
    if(!check_imu_state_call_back){
        std::cout<<"check_imu_state_call_back is called "<< std::endl;
        check_imu_state_call_back = true;
    }

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
    imu_state_initialized_ = true;  
}