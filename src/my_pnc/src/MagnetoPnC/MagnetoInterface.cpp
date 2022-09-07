#include <math.h>
#include <stdio.h>
#include <my_robot_system/RobotSystem.hpp>
#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateEstimator.hpp>
#include <my_pnc/MagnetoPnC/MagnetoStateProvider.hpp>
#include <my_pnc/MagnetoPnC/MagnetoMotionAPI.hpp>
#include <my_utils/IO/IOUtilities.hpp>
#include <my_utils/Math/MathUtilities.hpp>
#include <string>

MagnetoInterface::MagnetoInterface() : EnvInterface() {
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    my_utils::color_print(myColor::BoldCyan, border);
    my_utils::pretty_constructor(0, "Magneto Interface");

    robot_ = new RobotSystem(Magneto::n_vdof, 
        // THIS_COM "robot_description/Robot/Magneto/MagnetoSim_Dart.urdf");
        THIS_COM "robot_description/Robot/Magneto/magneto_hexa.urdf");   

    robot_->setActuatedJoint(Magneto::idx_adof);
    

    state_estimator_ = new MagnetoStateEstimator(robot_);
    sp_ = MagnetoStateProvider::getStateProvider(robot_);

    control_architecture_ = new MagnetoControlArchitecture(robot_);
    interrupt_ = new WalkingInterruptLogic(
          static_cast<MagnetoControlArchitecture*>(control_architecture_));    

    sp_->stance_foot = MagnetoBodyNode::base_link; // todo base_link

    count_ = 0;
    waiting_count_ = 2;
    cmd_jpos_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Magneto::n_adof);

    prev_planning_moment_ = 0.;
    check_com_planner_updated = 0;
    check_foot_planner_updated = 0;

    _ParameterSetting(); //text_ = new Test();

    my_utils::color_print(myColor::BoldCyan, border);
}

MagnetoInterface::~MagnetoInterface() {
    delete robot_;
    delete state_estimator_;
    delete control_architecture_;
    delete interrupt_;
}

void MagnetoInterface::checkContactDynamics(const Eigen::VectorXd& torque,
                                            Eigen::VectorXd& qddot,
                                            Eigen::VectorXd& Fc) {

    // GET CURRENT STATE
    Eigen::MatrixXd Sa = robot_->getSelectionMatrix();
    Eigen::MatrixXd tau_a = Sa*torque;

    Eigen::VectorXd q = robot_->getQ();
    Eigen::VectorXd dq = robot_->getQdot();
    Eigen::VectorXd ddq = robot_->getQddot();

    // GET JACOBIAN STACK MATRIX
    Eigen::MatrixXd Jc_b;
    Eigen::VectorXd Jdotqdot;
    int Jc_row_size = 0;
    int Jc_col_size = robot_->getNumDofs();
    bool b_contact = false;
    int link_idx = -1;
    int contact_dim = 6;

    for (int foot_idx = 0; foot_idx < Magneto::n_leg; ++foot_idx) {
        if(sp_->b_contact_plan_list[foot_idx]){// contact is true 
            b_contact = true;
            link_idx = MagnetoFoot::LinkIdx[foot_idx];
            Eigen::MatrixXd Jtmp = robot_->getBodyNodeCoMBodyJacobian(link_idx);        
            Eigen::MatrixXd Jc = Jtmp.block(6-contact_dim, 0, contact_dim, robot_->getNumDofs());
            Jc_row_size = Jc_b.rows();
            Jc_b.conservativeResize(Jc_row_size + Jc.rows(), Jc_col_size);
            Jc_b.block(Jc_row_size,0, Jc.rows(), Jc_col_size) = Jc;

            Eigen::VectorXd JcDotQdot_tmp =robot_->getBodyNodeCoMBodyJacobianDot(link_idx) * dq;
            Jdotqdot.conservativeResize(Jc_row_size + Jc.rows());
            Jdotqdot.segment(Jc_row_size, Jc.rows()) = JcDotQdot_tmp.tail(contact_dim);   
        }
    }   
    
    Eigen::MatrixXd M = robot_->getMassMatrix();
    Eigen::MatrixXd MInv = robot_->getInvMassMatrix();
    Eigen::VectorXd cori_grav = robot_->getCoriolisGravity();
    Eigen::MatrixXd AInv, AMat, Jc_bar_T, Nc_T;

    if(b_contact) {
        Eigen::MatrixXd Inxn = Eigen::MatrixXd::Identity(robot_->getNumDofs(),robot_->getNumDofs());
        
        AInv = Jc_b * MInv * Jc_b.transpose(); // nc x nc
        my_utils::pseudoInverse(AInv, 0.0001, AMat); // inv(Ainv) = AMat
        Jc_bar_T = AMat * Jc_b * MInv;
        Nc_T =  Inxn - Jc_b.transpose()*Jc_bar_T;
        
        Fc = Jc_bar_T * ( (Inxn - Nc_T) * (cori_grav - Sa.transpose()*tau_a) - M*Jc_b.transpose()*AMat*Jdotqdot );
        qddot = MInv * (Jc_b.transpose()*Fc + Sa.transpose()*tau_a - cori_grav);
    }
}

void MagnetoInterface::getCommand(void* _data, void* _command) {
    MagnetoCommand* cmd = ((MagnetoCommand*)_command);
    MagnetoSensorData* data = ((MagnetoSensorData*)_data);
    if(!_Initialization(data, cmd)) {
        state_estimator_->Update(data); // robot skelPtr in robotSystem updated 
        interrupt_->processInterrupts();
        control_architecture_->getCommand(cmd);
        
        if(!_CheckCommand(cmd)) { _SetStopCommand(data,cmd); }    
    }   
    // save data
    _SaveDataCmd(data, cmd);
    running_time_ = ((double)count_)*MagnetoAux::servo_rate;
    sp_->curr_time = running_time_;
    sp_->phase_copy = control_architecture_->getState();
    ++count_;
}

bool MagnetoInterface::_Initialization(MagnetoSensorData* data,
                                        MagnetoCommand* _command) {
    static bool test_initialized(false);
    if (!test_initialized) {
        control_architecture_->ControlArchitectureInitialization();
        test_initialized = true;
    }
    if (count_ < waiting_count_) {
         _SetStopCommand(data, _command);
        state_estimator_->Initialization(data);
        return true;
    }
    return false;
}

void MagnetoInterface::_ParameterSetting() {
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "config/Magneto/INTERFACE.yaml");
        // std::string test_name =
        //     my_utils::readParameter<std::string>(cfg, "test_name");
        std::string motion_script;        
        my_utils::readParameter(cfg, "motion_script", motion_script);
        std::ostringstream motion_file_name;    
        motion_file_name << THIS_COM << motion_script;
        YAML::Node motion_cfg = YAML::LoadFile(motion_file_name.str());
        int num_motion;
        my_utils::readParameter(motion_cfg, "num_motion", num_motion);

        for(int i(0); i<num_motion; ++i){
            std::ostringstream stringStream;
            stringStream << "motion" << i;
            std::string conf = stringStream.str();
            interrupt_->setInterruptRoutine(motion_cfg[conf]);           
        }

        
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }
}

bool MagnetoInterface::_CheckCommand(MagnetoCommand* cmd){
    // return false, if it fails to update test command
    // e.g.) check limit cmd->q, cmd->qdot, cmd-<jtrq 
    return true;
}

void MagnetoInterface::_SetStopCommand(MagnetoSensorData* data, MagnetoCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void MagnetoInterface::_SaveDataCmd(MagnetoSensorData* data, MagnetoCommand* cmd)  {
    // cmd->jtrq = my_utils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq in interface");
    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;
}

bool MagnetoInterface::IsTrajectoryUpdated() {
    if (prev_planning_moment_ == sp_->planning_moment) {
        prev_planning_moment_ = sp_->planning_moment;
        return false;
    } else {
        prev_planning_moment_ = sp_->planning_moment;
        return true;
    }
}

bool MagnetoInterface::IsPlannerUpdated() {
    if (check_com_planner_updated == sp_->check_com_planner_updated) {
        return false;
    } else {
        check_com_planner_updated = sp_->check_com_planner_updated;
        return true;
    }   
}


bool MagnetoInterface::IsFootPlannerUpdated() {
    if (check_foot_planner_updated == sp_->check_foot_planner_updated) {
        return false;
    } else {
        check_foot_planner_updated = sp_->check_foot_planner_updated;
        return true;
    }   
}

void MagnetoInterface::GetCoMTrajectory(
    std::vector<Eigen::VectorXd>& com_des_list) {
    com_des_list = sp_->com_des_list;
}
void MagnetoInterface::GetContactSequence(
    std::vector<Eigen::Isometry3d>& foot_target_list) {
    foot_target_list = sp_->foot_target_list;
}

void MagnetoInterface::GetFeasibleCoM(
    std::vector <std::pair<double, Eigen::Vector3d>>& feasible_com_list) {
    feasible_com_list = sp_->feasible_com_list;
}

void MagnetoInterface::GetCurrentCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_init;
}

void MagnetoInterface::GetOptimalCoM(Eigen::VectorXd& com_des) {
    com_des = sp_->com_pos_target;
}

void MagnetoInterface::GetCurrentFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_init;
}

void MagnetoInterface::GetNextFootStep(Eigen::VectorXd& foot_pos) {
    foot_pos = sp_->foot_pos_target;
}

///////////////////////////////////////////////////////////////////////////////////






