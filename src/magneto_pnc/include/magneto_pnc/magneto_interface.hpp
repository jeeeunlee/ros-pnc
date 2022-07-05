#pragma once

#include "pnc_core/env_interface.hpp"
#include "magneto_pnc/magneto_definition.hpp"
#include "magneto_pnc/magneto_command.hpp"


class MagnetoStateProvider;
class MagnetoStateEstimator;

namespace RUN_MODE {
constexpr int BALANCE = 0;
constexpr int STATICWALK = 1;
constexpr int MPCCLIMBING = 2;
constexpr int HWTEST = 3;
};  // namespace RUN_MODE


class IMUSensorData{
    public:
    IMUSensorData() {
        orientation = Eigen::Quaternion<double>::Identity();
         
        angular_velocity.setZero();
        linear_acceleration.setZero();

        orientation_covariance.setZero();
        angular_velocity_covariance.setZero();
        linear_acceleration_covariance.setZero();
    }
    ~IMUSensorData(){}
    Eigen::Quaternion<double> orientation;
    Eigen::Matrix3d orientation_covariance;
    Eigen::Vector3d angular_velocity;
    Eigen::Matrix3d angular_velocity_covariance;
    Eigen::Vector3d linear_acceleration;
    Eigen::Matrix3d linear_acceleration_covariance;

};

class MagnetoSensorData {
   public:
    MagnetoSensorData() {
        q = Eigen::VectorXd::Zero(Magneto::n_adof);
        qdot = Eigen::VectorXd::Zero(Magneto::n_adof);
        virtual_q = Eigen::VectorXd::Zero(Magneto::n_vdof);
        virtual_qdot = Eigen::VectorXd::Zero(Magneto::n_vdof);
        alf_wrench = Eigen::VectorXd::Zero(6);
        blf_wrench = Eigen::VectorXd::Zero(6);
        arf_wrench = Eigen::VectorXd::Zero(6);
        brf_wrench = Eigen::VectorXd::Zero(6);
        alfoot_contact = false;
        blfoot_contact = false;
        arfoot_contact = false;
        brfoot_contact = false;

        tau_cmd_prev = Eigen::VectorXd::Zero(Magneto::n_adof);
        surface_normal = {Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ(), 
                        Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ()};

        imu_data = IMUSensorData();
    }
    virtual ~MagnetoSensorData() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd virtual_q;
    Eigen::VectorXd virtual_qdot;
    Eigen::VectorXd alf_wrench;
    Eigen::VectorXd blf_wrench;
    Eigen::VectorXd arf_wrench;
    Eigen::VectorXd brf_wrench;
    bool alfoot_contact;
    bool blfoot_contact;
    bool arfoot_contact;
    bool brfoot_contact;

    Eigen::VectorXd tau_cmd_prev;
    
    std::array<Eigen::Vector3d, Magneto::n_leg> surface_normal;
    IMUSensorData imu_data;
};

class MagnetoCommand {
   public:
    MagnetoCommand() {
        q = Eigen::VectorXd::Zero(Magneto::n_adof);
        qdot = Eigen::VectorXd::Zero(Magneto::n_adof);
        jtrq = Eigen::VectorXd::Zero(Magneto::n_adof);

        for(auto &onoff : magnetism_onoff)
            onoff =  false;
    }
    virtual ~MagnetoCommand() {}

    Eigen::VectorXd q;
    Eigen::VectorXd qdot;
    Eigen::VectorXd jtrq;
    std::array<bool, Magneto::n_leg> magnetism_onoff;

    // double alfoot_magnetism_on; // 0~1
    // double blfoot_magnetism_on; // 0~1
    // double arfoot_magnetism_on; // 0~1
    // double brfoot_magnetism_on; // 0~1
};

class MagnetoInterface : public EnvInterface {
   protected:
    void _ParameterSetting();
    bool _Initialization(MagnetoSensorData*, MagnetoCommand*);
    bool _CheckCommand(MagnetoCommand* cmd);
    void _SetStopCommand(MagnetoSensorData*, MagnetoCommand* cmd);
    void _SaveDataCmd(MagnetoSensorData*, MagnetoCommand* cmd);

    std::string test_name_;

    MagnetoStateEstimator* state_estimator_;
    MagnetoStateProvider* sp_;


    int count_;
    int waiting_count_;
    Eigen::VectorXd cmd_jpos_;
    Eigen::VectorXd cmd_jvel_;
    Eigen::VectorXd cmd_jtrq_;

    int check_com_planner_updated;
    int check_foot_planner_updated;
    int run_mode_;

   public:
    MagnetoInterface();
    virtual ~MagnetoInterface();

    virtual void getCommand(void* _sensor_data, void* _command_data);
    int getRunMode() {return run_mode_;}    

    void GetFeasibleCoM(std::vector <std::pair<double, Eigen::Vector3d>>& 
                        feasible_com_list);
    void GetCurrentCoM(Eigen::VectorXd& com_pos);
    void GetOptimalCoM(Eigen::VectorXd& com_pos);
    void GetCurrentFootStep(Eigen::VectorXd& foot_pos);
    void GetNextFootStep(Eigen::VectorXd& foot_pos);

    void GetCoMPlans(Eigen::VectorXd& com_pos_ini,
                    Eigen::VectorXd& com_pos_goal);
    
    bool IsPlannerUpdated();
    bool IsFootPlannerUpdated();

    void AddScriptMotion(const YAML::Node& motion_cfg);
    int getCurrentMovingFootLinkIdx();
    int getCurrentMovingFootIdx();
};
