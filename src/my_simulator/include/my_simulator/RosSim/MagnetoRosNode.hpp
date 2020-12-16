#pragma once

#include <Eigen/Dense>
#include <queue> 
#include <mutex>

#include <../my_utils/Configuration.h>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>

#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include "my_ros_msgs/MagnetoCommand.h"
#include "my_ros_msgs/MagnetoSensorData.h"
#include "my_ros_msgs/MagnetismOnOff.h"
#include "my_ros_msgs/ForceWrench.h"
#include "my_ros_msgs/Quaternion.h"
#include "my_ros_msgs/JointData.h"


// MagnetoInterface
class EnvInterface;
class MagnetoSensorData;
class MagnetoCommand;

class SimulatorParameter{
  public: 
    SimulatorParameter(std::string _config_file){
        
        // READ PARAMETERS FROM CONFIGURATION FILE
        _config_file.insert(0, THIS_COM);
        try {
            YAML::Node simulation_cfg = YAML::LoadFile(_config_file);
            my_utils::readParameter(simulation_cfg, "servo_rate", servo_rate_);
            my_utils::readParameter(simulation_cfg, "is_record", is_record_);
            my_utils::readParameter(simulation_cfg, "show_joint_frame", b_show_joint_frame_);
            my_utils::readParameter(simulation_cfg, "show_link_frame", b_show_link_frame_);

            my_utils::readParameter(simulation_cfg, "ground", fp_ground_);
            my_utils::readParameter(simulation_cfg, "robot", fp_robot_);
            my_utils::readParameter(simulation_cfg, "initial_pose", q_virtual_init_); 
            my_utils::readParameter(simulation_cfg, "friction", coeff_fric_);
            
            // 
            my_utils::readParameter(simulation_cfg, "plot_result", b_plot_result_);
            my_utils::readParameter(simulation_cfg, "magnetic_force", magnetic_force_);  
            my_utils::readParameter(simulation_cfg, "residual_magnetism", residual_magnetism_);  

            my_utils::readParameter(simulation_cfg["control_configuration"], "kp", kp_);
            my_utils::readParameter(simulation_cfg["control_configuration"], "kd", kd_);
            my_utils::readParameter(simulation_cfg["control_configuration"], "torque_limit", torque_limit_);           

        } catch (std::runtime_error& e) {
            std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                    << __FILE__ << "]" << std::endl
                    << std::endl;
        }        

        // SET DEFAULT VALUES && POST PROCESSING
        gravity_ << 0.0, 0.0, -9.81;

        // SET FILE PATH
        fp_ground_.insert(0, THIS_COM);
        fp_robot_.insert(0, THIS_COM);

        // CHECK THE VALUES
        my_utils::pretty_print(q_virtual_init_, std::cout, "q_virtual_init_");  
        my_utils::pretty_print(gravity_, std::cout, "gravity_");  
    }

  public:
    // visualization
    double servo_rate_;
    bool is_record_;
    bool b_show_joint_frame_;
    bool b_show_link_frame_;
    bool b_plot_result_;
    Eigen::VectorXd q_virtual_init_;

    // file path
    std::string fp_ground_;
    std::string fp_robot_;

    // physic config
    Eigen::Vector3d gravity_;
    double coeff_fric_;
    double magnetic_force_; // 147. #[N] 
    double residual_magnetism_; //  3.0 #[%]

    // control config
    double kp_;
    double kd_;
    double torque_limit_;
};

class MagnetoRosNode : public dart::gui::osg::WorldNode {

  public:
    MagnetoRosNode(ros::NodeHandle& nh, 
                  const SimulatorParameter& sim_param, 
                  const dart::simulation::WorldPtr& _world);
    virtual ~MagnetoRosNode();

    void customPreStep() override;
    void customPostStep() override;

    // user button
    void enableButtonFlag(uint16_t key);
    void MagnetoCmdCallBack(const my_ros_msgs::MagnetoCommandConstPtr cmd);

   private:

    void UpdateSensorDataMsg();
    void UpdateContactDistance();
    void UpdateContactSwitchData();
    void UpdateContactWrenchData();

    void PlotResult();
    void PlotFootStepResult();
    
    void CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel);
    
    void EnforceTorqueLimit(); 
    void ApplyMagneticForce();

    void clearSensorDataMsg();
    void waitCommand();

    ros::NodeHandle nh_;
    ros::Subscriber sub_cmd_;
    ros::Publisher pub_data_;
    ros::Publisher pub_interrupt_logic_;
    my_ros_msgs::MagnetoSensorData sensor_data_msg_;
    my_ros_msgs::MagnetoCommand command_msg_;
    std::mutex cmd_data_mutex_;
    int cmd_call_back_count_;
    int prev_call_back_count_;
    
    // EnvInterface* interface_;
    MagnetoSensorData* sensor_data_;
    MagnetoCommand* command_;

    std::array<std::pair<std::string,dart::dynamics::BodyNode*>,4> contact_bn_list_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::SkeletonPtr ground_;

    Eigen::MatrixXd R_gw_;
    Eigen::MatrixXd p_gw_;
    Eigen::Quaterniond ground_quat_;

    Eigen::VectorXd trq_cmd_;
    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    int count_;
    double t_;
    double servo_rate_;
    int n_dof_;

    double magnetic_force_; // 147. #[N] 
    double residual_magnetism_; //  3.0 #[%]
    double kp_;
    double kd_;

    float contact_threshold_;
    std::map<int, double> contact_distance_;
    bool b_plot_result_;
};
