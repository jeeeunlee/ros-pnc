#pragma once

#include <../my_utils/Configuration.h>
#include <Eigen/Dense>

class RobotSystem;
class ContactSpec;
class MagnetoContactTestPlanner {
   public:
        MagnetoContactTestPlanner(RobotSystem* robot);
        ~MagnetoContactTestPlanner();

        void setFullContact(const std::vector<ContactSpec*>& contact_list);
        void setNewContact(const int& _foot_idx);
        void setContactInfo(const std::map<int, double>& f_pull_max,
                           const std::map<int, double>& friction_coeff);
        // CASE 1
        void findMinForcePullOnly(int next_foot_idx_i);
        // CASE 2
        void findMinForceAll(int next_foot_idx_i);

   protected:
        
        void _setNextfoodIndices(const int& next_foot_idx); //next_step_idices_

        void _setContactListForComputation(bool b_new_contact, 
                                          const int& next_foot_idx,
                                          std::vector<ContactSpec*> & contact_list);

        Eigen::MatrixXd _getAiMatrix(int contact_link_idx);                                          
        void _buildAstanceMatrix(const std::vector<ContactSpec*> & contact_list);
        void _buildUMatrix(const std::vector<ContactSpec*> & contact_list);



        void _updateConvexHull();

        void _ineqMat2Uf(Eigen::MatrixXd &Uf);  // delete the last row for max Fz in U Matrix

      //   void _buildHyperPlane(const::Eigen::MatrixXd& gravity, Eigen::MatrixXd& abc, Eigen::VectorXd& d);        
      //   void _buildHyperPlane(Eigen::MatrixXd& abc, Eigen::VectorXd& d);  
      //   void _buildProjectionMatrix(const Eigen::MatrixXd& J, Eigen::MatrixXd& N);
      //   void _buildJacobianFromContacts(const std::vector<ContactSpec*>& contact_list, Eigen::MatrixXd& Jc);
        
      //   double _getMinDistance(const Eigen::VectorXd& pos, const Eigen::MatrixXd& abc, const Eigen::VectorXd& d);
      //   void _getRandomSample(const Eigen::Vector3d& com_pos_ini, Eigen::Vector3d& com_pos_sample);
      //   void _enforceJointLimit(const Eigen::VectorXd& q_curr, Eigen::VectorXd& dq);        
        
        Eigen::MatrixXd _null(const Eigen::MatrixXd& input);
        Eigen::MatrixXd _pinv(const Eigen::MatrixXd& input);
        Eigen::MatrixXd _skew(const Eigen::Vector3d& vec);

        // variables

        RobotSystem* robot_;
        RobotSystem* robot_temp_;

        std::vector<ContactSpec*> full_contact_list_;
        std::map<int, double> f_pull_max_;
        Eigen::Vector6d Fg_;
        Eigen::Vector3d gravity_;
        double mass_;
        int new_contact_idx_;
        std::vector<int> next_step_idices_; // possible next footstep except new contact
        std::vector<int> link_idx_; // U2 ~

        Eigen::MatrixXd U_;
        Eigen::MatrixXd Uav_;
        Eigen::MatrixXd Anew_;
        Eigen::MatrixXd Astance_;


};