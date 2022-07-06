#include <ros/ros.h>
#include <magneto_test/hwtest/magneto_interface_include.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <pnc_utils/io_utilities.hpp>


class MagnetoInterface;
class MagnetoSensorData;
class MagnetoCommand;
class IMUSensorData;

enum SystemState
{
  SUSPENDED,          ///< Controller system is temporarily suspended, waiting for user input.
  OPERATIONAL,        ///< Controller system is operational and running.
  SYSTEM_STATE_COUNT, ///< Misc enum defining number of System States
};

class MagnetoInterfaceParams {
  public: 
    MagnetoInterfaceParams();
    ~MagnetoInterfaceParams(){}

    void init(){
        control_rate = MagnetoAux::servo_rate;
    }

    // control parameters
    double control_rate;
};

class MagnetoInterfaceNode {
    public:
    MagnetoInterfaceNode();
    ~MagnetoInterfaceNode();

    /** Accessor for parameter member */
    inline const MagnetoInterfaceParams& getParameters(void) { return params_; };
    /** Accessor for system state member */
    inline SystemState getSystemState(void) { return system_state_; };

    // get, set, initializes
    void getParams();

    // state function and main loop
    void loop();

    // publish
    void publishMagnetOnoff();
    void publishDesiredJointState();

    // callbacks
    void jointStateCallback(const sensor_msgs::JointState &joint_states);
    void imuStateCallback(const sensor_msgs::Imu &data);

    private:
    ros::NodeHandle nh_;

    MagnetoInterfaceParams params_;
    SystemState system_state_;

    MagnetoInterface* interface_;
    MagnetoSensorData* sensor_data_;
    MagnetoCommand* command_;

    //
    ros::Publisher desired_joint_state_publisher_, adhesion_publisher_;    
    ros::Subscriber joint_state_subscriber_, imu_state_subscriber_, adhesion_subscriber_;

    // JOINT STATE VALUES
    Eigen::VectorXd qa_config_;
    Eigen::VectorXd qa_dot_;
    Eigen::VectorXd qa_effort_;

    Eigen::VectorXd qa_config_des_;
    Eigen::VectorXd qa_dot_des_;
    std::array<std::string, Magneto::n_adof> qa_names_;

    std::array<bool, Magneto::n_leg> contact_;
    std::array<Eigen::VectorXd, Magneto::n_leg> ft_sensor_;
    std::array<bool, Magneto::n_leg> adhesion_cmd_;

    // IMU DATA
    IMUSensorData* imu_data_;

};
