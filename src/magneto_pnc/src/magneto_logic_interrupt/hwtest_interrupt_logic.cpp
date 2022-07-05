#include <magneto_pnc/magneto_logic_interrupt/hwtest_interrupt_logic.hpp>
#include <magneto_pnc/magneto_control_architecture/magneto_control_architecture_set.hpp>



HWTestInterruptLogic::HWTestInterruptLogic(
        ControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  pnc_utils::pretty_constructor(1, "Magneto Climbing Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  script_user_cmd_deque_.clear();
  user_state_cmd_ = new MagnetoUserStateCommand(); 
}

HWTestInterruptLogic::~HWTestInterruptLogic() {
}

// Process Interrupts here
void HWTestInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Climbing Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "@@@@ [Climbing Interrupt Logic] button S pressed << SCRIPT MOTION ADDED" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          // set stateMachine sequences
          for(auto &it : script_user_cmd_deque_) {
            // set env for simulation
            addStateCommand(MAGNETO_STATES::BALANCE, it);
            addStateCommand(MAGNETO_STATES::SWING_START_TRANS, it);
            addStateCommand(MAGNETO_STATES::SWING, it);
            addStateCommand(MAGNETO_STATES::SWING_END_TRANS, it); 
            }         
          addStateCommand(MAGNETO_STATES::BALANCE, MotionCommand() );
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
  
}

void HWTestInterruptLogic::addStateCommand(int _state_id, 
                              const MotionCommand& _smc){
  user_state_cmd_->setCommand(_state_id, _smc);
  ctrl_arch_->addState(user_state_cmd_);
}


// climbset.yaml
// # foot : AL-0, AR-1, BL-2, BR-3
// # frame : base-0, foot-1
// foot: 2
// pos: [-0.1,0,0]
// ori: [1,0,0,0]
// swing_height: 0.05
// duration: 0.4  
// frame: 0
// new_contact_spec: [0.7, 100] # for only simulation [mu, adhesion]

void HWTestInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg) {
  // motion
  Eigen::VectorXd pos_temp, ori_temp;
  int frame;
  pnc_utils::readParameter(motion_cfg, "pos",pos_temp);
  pnc_utils::readParameter(motion_cfg, "ori", ori_temp);    
  pnc_utils::readParameter(motion_cfg, "frame", frame);

  SWING_DATA swing_motion;
  swing_motion.dpose = POSE_DATA(pos_temp, ori_temp, frame==0);  
  pnc_utils::readParameter(motion_cfg, "foot", swing_motion.foot_idx);    
  pnc_utils::readParameter(motion_cfg, "swing_height", swing_motion.swing_height);

  Eigen::VectorXd motion_periods;
  pnc_utils::readParameter(motion_cfg, "durations", motion_periods);

  MotionCommand mc_temp = MotionCommand(swing_motion, motion_periods);


  script_user_cmd_deque_.push_back( mc_temp );
}