#pragma once

#include <Eigen/Dense>
#include <Goldfarb/QuadProg++.hh>
#include <magneto_pnc/magneto_wbc_controller/mcwbc/mc_wbc.hpp>

class ContactSpec;
class MagnetSpec;

// MFWBCC (Minimum Friction - Whole Body Climbing Control)
// force variable to be minimized: Fx=Fc-Fm (friction cone force)

// residual magnetic force acting on swing foot 
// is considered in the dynamics equation

class MFWBCC: public MCWBC{
    public:
        MFWBCC(const std::vector<bool> & act_list);
        virtual ~MFWBCC() {}

        virtual void makeTorqueGivenRef(const Eigen::VectorXd & ref_cmd,
                const std::vector<ContactSpec*> & contact_list,
                const std::vector<MagnetSpec*> &magnet_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL);

    protected:
        Eigen::VectorXd qddot_ref_; // qddot ref

        // ----------- OPTIMIZATION ----------- // 
        // Setup the followings:
        int dim_opt_;
        int dim_eq_cstr_; // equality constraints
        int dim_ieq_cstr_; // inequality constraints
        int dim_rf_; // reaction force

        virtual void _BuildMagnetMtxVect(const std::vector<MagnetSpec*> &magnet_list);
        Eigen::MatrixXd Jm_;
        Eigen::VectorXd Fm_;

        virtual void _BuildContactMtxVect(const std::vector<ContactSpec*> & contact_list);
        Eigen::MatrixXd Uf_;
        Eigen::VectorXd Fr_ieq_;
        Eigen::MatrixXd Jc_;
        Eigen::VectorXd JcDotQdot_;
        Eigen::VectorXd JcQdot_;


        virtual void _Build_Equality_Constraint();
        Eigen::MatrixXd Aeq_;
        Eigen::VectorXd beq_;

        virtual void _Build_Inequality_Constraint();
        Eigen::MatrixXd Cieq_;
        Eigen::VectorXd dieq_;

        void _OptimizationPreparation();
        GolDIdnani::GVect<double> z; // Optimal Variable        
        GolDIdnani::GMatr<double> G; // Cost
        GolDIdnani::GVect<double> g0;        
        GolDIdnani::GMatr<double> CE; // Equality
        GolDIdnani::GVect<double> ce0;        
        GolDIdnani::GMatr<double> CI; // Inequality
        GolDIdnani::GVect<double> ci0;

        virtual void _GetSolution(Eigen::VectorXd & cmd);
        Eigen::VectorXd delta_qddot_;
        Eigen::VectorXd Fc_;
        Eigen::VectorXd xc_ddot_;
        Eigen::VectorXd tau_cmd_; 

    private:
        void _saveDebug();
};
