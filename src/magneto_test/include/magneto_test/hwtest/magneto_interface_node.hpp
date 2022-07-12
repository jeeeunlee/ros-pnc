#include <ros/ros.h>
#include <magneto_test/hwtest/magneto_interface_include.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <pnc_utils/io_utilities.hpp>

#define THROTTLE_PERIOD 5

class MagnetoInterface;
class MagnetoSensorData;
class MagnetoCommand;
class IMUSensorData;

enum SystemState
{
    SUSPENDED,      ///< Controller system is temporarily suspended, 
    CHECKED,        ///< waiting for user input="GO TO INITIAL CONFIGURATION."
    IDLE,           ///< waiting for user input="RESET ESTIMATOR."
    READY,          ///< Estimator is initialized, ready to operate
    RUNNING,        ///< Controller system is running.
    STOP,           ///< Stop button clicked
};

enum JoypadButtonIndex
{
  A_BUTTON, // 0
  B_BUTTON, // 1
  X_BUTTON, // 2
  Y_BUTTON, // 3
  LEFT_BUMPER, // 4
  RIGHT_BUMPER, // 5
  BACK, // 6
  START, // 7
  LOGITECH, // 8
  LEFT_JOYSTICK, // 9
  RIGHT_JOYSTICK, // 10
};

class MagnetoInterfaceNode {
  public:
    MagnetoInterfaceNode();
    ~MagnetoInterfaceNode();

    /** Accessor for system state member */
    inline SystemState getSystemState(void) { return system_state_; };

    // get, set, initializes
    bool startButtonPressed(){ 
      if(startbutton_pressed) { debounce_startbutton_=true; return true; }
      else return false;  }
    bool stopButtonPressed(){ 
      if(stopbutton_pressed) { debounce_stopbutton_=true; return true; }
      else return false;  }

    void setSensorData();

    bool systemInitialized();    
    bool initializeJointConfiguration();
    bool resetEstimator();
    void enableButtonFlag(uint16_t key);
    void setStop();
    void releaseStop();

    // state function and main loop
    void loop();
    void stoploop();

    // publish
    void publishMagnetOnoff();
    void publishDesiredJointState();

    // callbacks
    void jointStateCallback(const sensor_msgs::JointState &joint_states);
    void imuStateCallback(const sensor_msgs::Imu &data);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

    void alAdhesionStateCallback(const std_msgs::Int8 &_msg) { this->AL_magnet_state_ = _msg.data; }
    void blAdhesionStateCallback(const std_msgs::Int8 &_msg) { this->BL_magnet_state_ = _msg.data; }
    void arAdhesionStateCallback(const std_msgs::Int8 &_msg) { this->AR_magnet_state_ = _msg.data; }
    void brAdhesionStateCallback(const std_msgs::Int8 &_msg) { this->BR_magnet_state_ = _msg.data; }

    void alProximitySenseCallback(const std_msgs::Int8 &_msg) { this->AL_contact_state_ = _msg.data; }
    void blProximitySenseCallback(const std_msgs::Int8 &_msg) { this->BL_contact_state_ = _msg.data; }
    void arProximitySenseCallback(const std_msgs::Int8 &_msg) { this->AR_contact_state_ = _msg.data; }
    void brProximitySenseCallback(const std_msgs::Int8 &_msg) { this->BR_contact_state_ = _msg.data; }

  private:
    ros::NodeHandle nh_;

    // system state
    SystemState system_state_;

    // joy stick state
    JoypadButtonIndex joy_state_;
    sensor_msgs::Joy joypad_control_;
    bool debounce_startbutton_;
    bool startbutton_pressed;
    bool debounce_stopbutton_;
    bool stopbutton_pressed;
    
    // magnet state
    bool AL_magnet_state_ = 1;
    bool AR_magnet_state_ = 1;
    bool BL_magnet_state_ = 1;
    bool BR_magnet_state_ = 1;
    bool AL_contact_state_ = 0;
    bool AR_contact_state_ = 0;
    bool BL_contact_state_ = 0;
    bool BR_contact_state_ = 0;

    MagnetoInterface* interface_;
    MagnetoSensorData* sensor_data_;
    MagnetoCommand* command_;

    bool joint_state_initialized_, imu_state_initialized_, b_stop_;

    //
    ros::Publisher desired_joint_state_publisher_, 
    al_adhesion_publisher_, ar_adhesion_publisher_, bl_adhesion_publisher_, br_adhesion_publisher_;

    ros::Subscriber joint_state_subscriber_, imu_state_subscriber_, adhesion_subscriber_, joypad_subscriber_,
    al_adhesion_subscriber_, ar_adhesion_subscriber_, bl_adhesion_subscriber_, br_adhesion_subscriber_,
    al_proximity_subscriber_, ar_proximity_subscriber_, bl_proximity_subscriber_, br_proximity_subscriber_;  

    // JOINT STATE VALUES
    std::array<std::string, Magneto::n_adof> qa_names_;
    Eigen::VectorXd qa_effort_;
    Eigen::VectorXd qa_config_;
    Eigen::VectorXd qa_dot_;

    Eigen::VectorXd qa_config_des_;
    Eigen::VectorXd qa_dot_des_;
    
    Eigen::VectorXd qa_stop_;
    
    std::array<bool, Magneto::n_leg> contact_;
    std::array<Eigen::VectorXd, Magneto::n_leg> ft_sensor_;
    std::array<bool, Magneto::n_leg> adhesion_cmd_;

    // IMU DATA
    IMUSensorData* imu_data_;

    std::map<std::string,int> magneto_actuator_ = { {"AL_coxa_joint", 0}, {"AL_femur_joint",1}, {"AL_tibia_joint",2},
                    {"AR_coxa_joint",3}, {"AR_femur_joint",4}, {"AR_tibia_joint",5},
                    {"BL_coxa_joint",6}, {"BL_femur_joint",7}, {"BL_tibia_joint",8},
                    {"BR_coxa_joint",9}, {"BR_femur_joint",10},{"BR_tibia_joint",11} };

};
