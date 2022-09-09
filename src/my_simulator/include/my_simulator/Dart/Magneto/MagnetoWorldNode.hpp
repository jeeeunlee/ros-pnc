#pragma once

#include <Eigen/Dense>
#include <queue> 
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>
#include <my_pnc/MagnetoPnC/MagnetoDefinition.hpp>

// MagnetoInterface
class EnvInterface;
class MagnetoSensorData;
class MagnetoCommand;

class MagnetoWorldNode : public dart::gui::osg::WorldNode {
   private:

    void UpdateContactDistance_(const std::string& ground_link_name);
    void UpdateContactSwitchData_();
    void UpdateContactWrenchData_();

    void SetParams_();
    void PlotResult_();
    void PlotFootStepResult_();
    void CheckInterrupt_();
    
    void CheckRobotSkeleton(const dart::dynamics::SkeletonPtr& skel);
    
    void EnforceTorqueLimit(); 
    void ApplyMagneticForce();


    EnvInterface* interface_;
    MagnetoSensorData* sensor_data_;
    MagnetoCommand* command_;

    dart::simulation::WorldPtr world_;
    dart::dynamics::SkeletonPtr robot_;
    dart::dynamics::SkeletonPtr ground_;

    Eigen::VectorXd trq_cmd_;

    int count_;
    double t_;
    double servo_rate_;
    int n_dof_;
    double kp_;
    double kd_;
    double torque_limit_;



    double magnetic_force_; // 147. #[N] 
    double residual_magnetism_; //  3.0 #[%]


    float contact_threshold_;
    std::array<double, Magneto::n_leg> contact_distance_;

    Eigen::VectorXd trq_lb_;
    Eigen::VectorXd trq_ub_;

    bool b_plot_result_;


   public:
    MagnetoWorldNode(const dart::simulation::WorldPtr& world);
    virtual ~MagnetoWorldNode();

    void customPreStep() override;
    void customPostStep() override;

    // user button
    void enableButtonFlag(uint16_t key);


};
