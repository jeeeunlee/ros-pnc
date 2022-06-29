#ifndef WHOLE_BODY_CONTROLLER
#define WHOLE_BODY_CONTROLLER

#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>
#include <pnc_utils/Math/pseudo_inverse.hpp>
#include <pnc_core/wbc/Task/task.hpp>
#include <pnc_core/wbc/Contact/ContactSpec.hpp>

// Assume first 6 (or 3 in 2D case) joints are for the representation of 
// a floating base.
class WBC{
    public:
        WBC(const std::vector<bool> & act_list):
            num_act_joint_(0),
            num_passive_(0)
    {
        num_qdot_ = act_list.size();
        for(int i(0); i<num_qdot_; ++i){
            if(act_list[i] == true) ++num_act_joint_;
            else ++num_passive_;
        }
        Sa_ = Eigen::MatrixXd::Zero(num_act_joint_, num_qdot_);
        Sv_ = Eigen::MatrixXd::Zero(num_passive_, num_qdot_);

        // Set virtual & actuated selection matrix
        int j(0);
        int k(0);
        for(int i(0); i <num_qdot_; ++i){
            if(act_list[i] == true){
                Sa_(j, i) = 1.;
                ++j;
            }
            else{
                Sv_(k, i) = 1.;
                ++k;
            }
        }

    }
        virtual ~WBC(){}

        virtual void updateSetting(const Eigen::MatrixXd & A,
                const Eigen::MatrixXd & Ainv,
                const Eigen::VectorXd & cori,
                const Eigen::VectorXd & grav,
                void* extra_setting = NULL) = 0;

        virtual void makeTorque(const std::vector<Task*> & task_list,
                const std::vector<ContactSpec*> & contact_list,
                Eigen::VectorXd & cmd,
                void* extra_input = NULL) =0;

    protected:
        // full rank fat matrix only
        void _WeightedInverse(const Eigen::MatrixXd & J,
                const Eigen::MatrixXd & Winv,
                Eigen::MatrixXd & Jinv){
            Eigen::MatrixXd lambda(J* Winv * J.transpose());
            Eigen::MatrixXd lambda_inv;
            pnc_utils::pseudoInverse(lambda, 0.0001, lambda_inv);
            Jinv = Winv * J.transpose() * lambda_inv;
        }

        int num_qdot_;
        int num_act_joint_;
        int num_passive_;

        Eigen::MatrixXd Sa_; // Actuated joint
        Eigen::MatrixXd Sv_; // Virtual joint

        Eigen::MatrixXd A_;
        Eigen::MatrixXd Ainv_;
        Eigen::VectorXd cori_;
        Eigen::VectorXd grav_;

        bool b_updatesetting_;

};

#endif