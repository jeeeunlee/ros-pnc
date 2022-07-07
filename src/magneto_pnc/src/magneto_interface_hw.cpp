#include <math.h>
#include <stdio.h>
#include <pnc_utils/robot_system.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_control_architecture_set.hpp>
#include <magneto_pnc/magneto_interface_hw.hpp>
#include <magneto_pnc/magneto_estimator/magneto_state_estimator.hpp>
#include <magneto_pnc/magneto_estimator/magneto_state_estimator_hw.hpp>
#include <magneto_pnc/magneto_state_provider.hpp>
#include <magneto_pnc/magneto_command.hpp>
#include <magneto_pnc/magneto_logic_interrupt/magneto_logic_interrupt_set.hpp>

#include <pnc_utils/io_utilities.hpp>
#include <pnc_utils/math_utilities.hpp>
#include <string>

MagnetoHWInterface::MagnetoHWInterface() : EnvInterface() {
    // MagnetoHWInterface 
    std::string border = "=";
    for (int i = 0; i < 79; ++i) {
        border += "=";
    }
    pnc_utils::color_print(myColor::BoldCyan, border);
    pnc_utils::pretty_constructor(0, "Magneto Interface");

    // set run_mode from interface.yaml
    _ParameterSetting(); 

    // declare
    robot_ = new RobotSystem(6+3*4, 
        // THIS_COM "robot_description/Robot/Magneto/MagnetoSim_limitup.urdf");
        // THIS_COM "robot_description/Robot/Magneto/MagnetoSim_Dart.urdf");//MagnetoSim_Dart?
        THIS_COM "src/magneto_2_description/robots/magneto_2_floatingbase.urdf");//MagnetoSim_Dart

        
    robot_->setActuatedJoint(Magneto::idx_adof);
    // robot_->setRobotMass();
    // robot_->printRobotInfo();
    sp_ = MagnetoStateProvider::getStateProvider(robot_);

    // control_architecture_ = new MagnetoWbcArchitecture(robot_);
    control_architecture_ = new MagnetoMpcControlArchitecture(robot_);

    interrupt_ = new HWTestInterruptLogic(control_architecture_);  
    state_estimator_ = new MagnetoHWStateEstimator(robot_);

    count_ = 0;
    reset_count_ = 0;
    waiting_count_ = 2;
    
    cmd_jpos_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jvel_ = Eigen::VectorXd::Zero(Magneto::n_adof);
    cmd_jtrq_ = Eigen::VectorXd::Zero(Magneto::n_adof);

    pnc_utils::color_print(myColor::BoldCyan, border);
}

MagnetoHWInterface::~MagnetoHWInterface() {
    delete robot_;
    delete state_estimator_;
    delete control_architecture_;
    delete interrupt_;
}

void MagnetoHWInterface::getInitialCommand(const Eign::VectorXd &qa_ini, 
                                            void* _data, void* _command){
    MagnetoCommand* cmd = ((MagnetoCommand*)_command);
    MagnetoSensorData* data = ((MagnetoSensorData*)_data);
    state_estimator_->Update(data);
    control_architecture_->getInitialCommand(qa_ini ,cmd); 
}

void MagnetoHWInterface::getCommand(void* _data, void* _command) {
    MagnetoCommand* cmd = ((MagnetoCommand*)_command);
    MagnetoSensorData* data = ((MagnetoSensorData*)_data);

    state_estimator_->Update(data); // robot skelPtr in robotSystem updated 
    interrupt_->processInterrupts();
    control_architecture_->getCommand(cmd);   
    
    if(!_CheckCommand(cmd)) { 
        _SetStopCommand(data,cmd); } 
    
    // save data
    _SaveDataCmd(data,cmd);

    running_time_ = ((double)count_)*MagnetoAux::servo_rate;
    sp_->curr_time = running_time_;
    ++count_;
}

bool MagnetoHWInterface::initialize(MagnetoSensorData* data, 
                                    MagnetoCommand* cmd) {
    static bool ctrlarch_initialized(false);
    if (!ctrlarch_initialized) {
        control_architecture_->ControlArchitectureInitialization();
        ctrlarch_initialized = true;
    }
    static int initialize_count(0);
    if ( ++initialize_count < waiting_count_) {
         _SetStopCommand(data, cmd);
        state_estimator_->Initialization(data);
        return false;    
    } else{
        _SetStopCommand(data, cmd);
        state_estimator_->Initialization(data);
        return true;
    }
}

bool MagnetoHWInterface::resetEstimator(MagnetoSensorData* data, 
                                        MagnetoCommand* cmd) {    
    if ( ++reset_count_ < waiting_count_) {
         _SetStopCommand(data, cmd);
        state_estimator_->Initialization(data);
        return false;    
    } else{
        _SetStopCommand(data, cmd);
        state_estimator_->Initialization(data);
        count_ = 0;
        reset_count_ = 0;
        return true;
    }    
}

void MagnetoHWInterface::_ParameterSetting() {
    std::string motion_file_name;    
    Eigen::VectorXd q_config_ini;
    try {
        YAML::Node cfg =
            YAML::LoadFile(THIS_COM "config/Magneto/HWTEST.yaml");
            pnc_utils::readParameter(cfg, "motion_script", motion_file_name);
            pnc_utils::readParameter(cfg, "initialize_configuration", q_config_ini);
        
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
        exit(0);
    }    

    _AddScriptMotion(motion_file_name);
}

void MagnetoHWInterface::_AddScriptMotion(const std::string& _motion_file_name){
    std::ostringstream motion_file_name;    
    motion_file_name << THIS_COM << _motion_file_name;
    std::cout<< "add script motions : " << motion_file_name.c_str() << std::endl;

    // script motion
    try { 
        YAML::Node motion_cfg = YAML::LoadFile(motion_file_name.str());
        int num_motion;
        int type_motion;
        pnc_utils::readParameter(motion_cfg, "num_motion", num_motion);
        pnc_utils::readParameter(motion_cfg, "type_motion", type_motion);
        for(int i(0); i<num_motion; ++i){
            std::ostringstream stringStream;
            stringStream << "motion" << i;
            interrupt_->setInterruptRoutine(motion_cfg[stringStream.str()]);            
        }
    } catch (std::runtime_error& e) {
        std::cout << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl
                  << std::endl;
    }
}

bool MagnetoHWInterface::_CheckCommand(MagnetoCommand* cmd){
    // return false, if it fails to update test command
    // e.g.) check limit cmd->q, cmd->qdot, cmd-<jtrq 
    return true;
}

void MagnetoHWInterface::_SetStopCommand(MagnetoSensorData* data, MagnetoCommand* cmd) {
  for (int i(0); i < robot_->getNumActuatedDofs(); ++i) {
    cmd->jtrq[i] = 0.;
    cmd->q[i] = data->q[i];
    cmd->qdot[i] = 0.;
  }
}

void MagnetoHWInterface::_SaveDataCmd(MagnetoSensorData* data, MagnetoCommand* cmd)  {
    // cmd->jtrq = pnc_utils::CropVector(cmd->jtrq,
    // robot_->GetTorqueLowerLimits(), robot_->GetTorqueUpperLimits(), "clip trq in interface");
    cmd_jtrq_ = cmd->jtrq;
    cmd_jvel_ = cmd->qdot;
    cmd_jpos_ = cmd->q;
}
