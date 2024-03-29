#pragma once

#include <Eigen/Dense>
#include <my_pnc/Constraint/Constraint.hpp>
#include <my_wbc/Contact/BodyFrameContactSpec.hpp>
#include <my_wbc/WBQPD/WBQPD.hpp>

class RobotSystem;
class ContactSpec;
class MagnetoControlArchitecture;

struct ReachabilityState{

   ReachabilityState(){};

   Eigen::VectorXd q;
   Eigen::VectorXd dq;
   Eigen::VectorXd ddq;
   Eigen::VectorXd trq;
   bool is_swing;
};
class MagnetoReachabilityContact {
   public:
      MagnetoReachabilityContact(RobotSystem* robot_planner);
      ~MagnetoReachabilityContact();
      void initialization(const YAML::Node& node);

      // solve
      void update(const Eigen::VectorXd& q,
                  const Eigen::VectorXd& dotq,
                  const Eigen::VectorXd& ddotq);
      bool solveContactDyn(Eigen::VectorXd& tau, 
                           Eigen::VectorXd& ddq_plan);
      bool computeDdotq(Eigen::VectorXd& tau,
                        Eigen::VectorXd& ddq);
      void computeNextState(const Eigen::VectorXd& tau_a,
                           Eigen::VectorXd& q_next,
                           Eigen::VectorXd& dotq_next);

      // setting
      void clearContacts();
      void addContacts(ContactSpec* contact);
      void FinishContactSet(); // set friction cone in wbqpd

   private:
      void _deleteContacts();
      void _updateContacts(const Eigen::VectorXd& q,
                           const Eigen::VectorXd& dotq);
      void _buildContactJacobian(Eigen::MatrixXd& Jc);
      void _buildContactJcDotQdot(Eigen::VectorXd&  Jdotqdot);

      Eigen::MatrixXd getPseudoInverse(const Eigen::MatrixXd& Mat);
      Eigen::MatrixXd getNullSpaceMatrix(const Eigen::MatrixXd& Mat);

   private:       
      RobotSystem* robot_planner_;
      std::vector<ContactSpec*> contact_list_;

      Eigen::MatrixXd Sa_;
      Eigen::MatrixXd Sv_;

      // updated variables
      Eigen::MatrixXd Jc_;
      Eigen::VectorXd Jcdotqdot_;

      Eigen::MatrixXd M_;
      Eigen::MatrixXd MInv_;
      Eigen::VectorXd cori_grav_;

      Eigen::MatrixXd AInv_;
      Eigen::MatrixXd AMat_;
      Eigen::MatrixXd Jc_bar_T_;
      Eigen::MatrixXd Nc_T_;
      Eigen::MatrixXd Q_;      

      // contact
      int dim_joint_;
      int dim_contact_;
      bool is_update_centroid_;      

      // wbqpd
      bool b_wbqpd_set;
      double magnetic_force_;
      double residual_ratio_;
      WbqpdParam* wbqpd_param_;
      WbqpdResult* wbqpd_result_;
      WBQPD* wbqpd_;

};


class MagnetoReachabilityNode {
   public:
      MagnetoReachabilityNode(MagnetoReachabilityContact* contact,
                              const Eigen::VectorXd& q,
                              const Eigen::VectorXd& dotq,
                              const bool& is_swing);
      ~MagnetoReachabilityNode();

      bool computeTorque(const Eigen::VectorXd& ddq_des,
                        Eigen::VectorXd& ddq_plan,
                        Eigen::VectorXd& tau_a);
      bool computeDdotq(Eigen::VectorXd& tau,
                        Eigen::VectorXd& ddq);

   public:
      Eigen::VectorXd q_;
      Eigen::VectorXd dotq_;
      bool is_swing_;

   private:     
      MagnetoReachabilityContact* contact_state_;
      std::vector<std::pair<std::shared_ptr<MagnetoReachabilityNode>, double >> next_node_set_;
};

class MagnetoReachabilityEdge {
   public:
      MagnetoReachabilityEdge(MagnetoReachabilityNode* src_node,
                              MagnetoReachabilityNode* dst_node,
                              const Eigen::VectorXd& trq_atv);
      ~MagnetoReachabilityEdge();

      void getReachabilityState(ReachabilityState &rcstate);

   private:
      MagnetoReachabilityNode* src_node_; // source
      MagnetoReachabilityNode* dst_node_; // destination
      Eigen::VectorXd trq_atv_;
      double duration_;
};


class MagnetoReachabilityPlanner {
   public:
      MagnetoReachabilityPlanner(RobotSystem* robot, MagnetoControlArchitecture* _ctrl_arch);
      ~MagnetoReachabilityPlanner();
      
      void initialization(const YAML::Node& node);
      void setMovingFoot(int moving_foot);
      void compute(const Eigen::VectorXd& q_goal);
      void addGraph(const std::vector<ReachabilityState> &state_list);
      void getOptimalTraj(std::deque<ReachabilityState> &traj);
      void initializeVariables();
  
   private:      
      void _setInitGoal(const Eigen::VectorXd& q_goal,
                        const Eigen::VectorXd& qdot_goal);  

   protected:
      MagnetoControlArchitecture* ctrl_arch_;

      RobotSystem* robot_;
      RobotSystem* robot_planner_;

      ContactSpec* alfoot_contact_;
      ContactSpec* blfoot_contact_;
      ContactSpec* arfoot_contact_;
      ContactSpec* brfoot_contact_;
      std::vector<ContactSpec*> full_contact_list_;

      // contact state -> qp solver under contact condition     
      MagnetoReachabilityContact* full_contact_state_;
      MagnetoReachabilityContact* swing_contact_state_;

      // nodes and edges
      std::vector<MagnetoReachabilityNode*> node_list_;
      std::vector<MagnetoReachabilityEdge*> edge_list_;
      std::vector<MagnetoReachabilityEdge*> edge_path_; // optimal path edge

   private:
      // constant
      Eigen::MatrixXd Sa_;
      Eigen::MatrixXd Sv_;      
      Eigen::VectorXd q_zero_;
      bool is_update_centroid_;

      // boundary conditions
      Eigen::VectorXd q_init_;
      Eigen::VectorXd dotq_init_;
      Eigen::VectorXd q_goal_;
      Eigen::VectorXd dotq_goal_;

           
      // state variables
      Eigen::VectorXd q_;
      Eigen::VectorXd dotq_;
      Eigen::VectorXd delq_;

      // contact
      double mu_;
      int moving_foot_idx_;
     
};