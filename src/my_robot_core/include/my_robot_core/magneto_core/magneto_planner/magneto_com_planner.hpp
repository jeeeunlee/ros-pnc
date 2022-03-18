#pragma once

#include <Eigen/Dense>

#include <my_robot_core/magneto_core/magneto_definition.hpp>
#include <my_robot_core/magneto_core/magneto_command_api.hpp>


class RobotSystem;
class ContactSpec;
class MagnetSpec;
class MagnetoStateProvider;


class MagnetoCoMPlanner{
    public:
    MagnetoCoMPlanner(RobotSystem* robot);
    ~MagnetoCoMPlanner() {};

    void computeSequence(const Eigen::Vector3d& pcom_goal,
                        MotionCommand &_motion_command,
                        const std::array<ContactSpec*, Magneto::n_leg>& f_contacts,
                        const std::array<MagnetSpec*, Magneto::n_leg>& f_mag);
    void setTransitionDuration(double _Tt) {Tt_ = _Tt;}

    ComMotionCommand getFullSupportCoMCmd();
    ComMotionCommand getSwingStartCoMCmd();
    ComMotionCommand getSwingCoMCmd();
    ComMotionCommand getSwingEndCoMCmd();

    private:
        void _buildPfRf();
        void _buildSwingSystem();
        void _buildFrictionCone(const std::array<ContactSpec*, 
                                Magneto::n_leg>& f_contacts_);
        void _buildWeightMatrices(const std::array<ContactSpec*, 
                                Magneto::n_leg>& f_contacts_);
        void _buildFm(const std::array<MagnetSpec*, 
                        Magneto::n_leg>& f_mag);
        void _solveSwingQuadProg();

    private:
        Eigen::Vector3d p_init_;
        Eigen::Vector3d p_goal_;
        Eigen::Vector3d zero_vel_;
        Eigen::Vector3d Ldot_;

        double Tf_; // full_period;
        double Ts_; // swing_period;        
        double Tt_; // trans_period;

        double mass_;
        Eigen::Vector3d grav_;        

        int swing_foot_idx_;

        // system matrices
        Eigen::MatrixXd A_;
        Eigen::VectorXd C_;

        // contact configuration matrices 
        Eigen::MatrixXd Pf_; // stack full
        Eigen::MatrixXd Rf_; // stack full
        Eigen::MatrixXd Pc_; // stack only contact
        Eigen::MatrixXd Rc_; // stack only contact
        // stacked magnetic force
        Eigen::VectorXd Fmf_; // magnetic force stack full
        Eigen::VectorXd Fmc_; // magnetic force stack only contact
        // Friction cone : Df*Fc >= df
        Eigen::MatrixXd Df_;
        Eigen::MatrixXd Dc_;
        Eigen::VectorXd df_;
        Eigen::VectorXd dc_;
        // weight matrices
        Eigen::MatrixXd invWf_;
        Eigen::MatrixXd invWc_;

        // Results
        Eigen::Vector3d delP_swing_;
        double ssdot_swing_;

        Eigen::Vector3d acc_swing_;
        Eigen::Vector3d p_swing_init_;
        Eigen::Vector3d v_swing_init_;

        bool initialized;
        RobotSystem* robot_;
        MagnetoStateProvider* sp_;
};

