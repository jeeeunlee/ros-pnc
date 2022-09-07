#include <my_pnc/MagnetoPnC/MagnetoCtrlArchitecture/MagnetoCtrlArchitecture.hpp>
#include <my_pnc/MagnetoPnC/MagnetoLogicInterrupt/WalkingInterruptLogic.hpp>

WalkingInterruptLogic::WalkingInterruptLogic(
        MagnetoControlArchitecture* _ctrl_arch)
        : InterruptLogic() {
  my_utils::pretty_constructor(1, "Magneto Walking Interrupt Logic");
  ctrl_arch_ = _ctrl_arch;
  sp_ = MagnetoStateProvider::getStateProvider(ctrl_arch_->robot_);
  
  // Initialize motion commands
  motion_command_script_list_.clear();

  motion_data_default_.pose = POSE_DATA(-0.1,0,0, 1,0,0,0);
  motion_data_default_.swing_height = 0.04;
  motion_data_default_.motion_period = 0.9;  

  motion_command_instant_ = new MotionCommand();
}

WalkingInterruptLogic::~WalkingInterruptLogic() {
  delete motion_command_instant_;
}

// Process Interrupts here
void WalkingInterruptLogic::processInterrupts() {   
  if(b_button_pressed) {
    // std::cout << "[Walking Interrupt Logic] button pressed : " << pressed_button << std::endl;
    switch(pressed_button){
      case 's':
        std::cout << "[Walking Interrupt Logic] button S pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     SCRIPT MOTION      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          // initialize trajectory_manager
          // ctrl_arch_->floating_base_lifting_up_manager_->
          // set stateMachine sequences
          for(auto &it : motion_command_script_list_) {          
            ctrl_arch_->add_next_state(MAGNETO_STATES::ONE_STEP_WALKING, it );      
          }
          // ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, MotionCommand() );
        }
      break;
      case 'w':
        std::cout << "[Walking Interrupt Logic] button w pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com up      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          MotionCommand mc_com_up;
          POSE_DATA pose_up(0,0,0.01, 1,0,0,0);
          double duration_up = 0.5;
          mc_com_up.add_com_motion(pose_up, duration_up);
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, mc_com_up);
        }
      break;
      case 'x':
        std::cout << "[Walking Interrupt Logic] button x pressed" << std::endl;
        std::cout << "---------                        ---------" << std::endl;
        std::cout << "---------     com down      ---------" << std::endl;
        if (ctrl_arch_->getState() == MAGNETO_STATES::BALANCE) {
          MotionCommand mc_com_up;
          POSE_DATA pose_up(0,0,-0.01, 1,0,0,0);
          double duration_up = 0.5;
          mc_com_up.add_com_motion(pose_up, duration_up);
          ctrl_arch_->add_next_state(MAGNETO_STATES::BALANCE, mc_com_up);
        }
      break;
      default:
        break;
    }
  }
  resetFlags();
}

void WalkingInterruptLogic::setInterruptRoutine(const YAML::Node& motion_cfg){

  MOTION_DATA md_temp;
  bool is_bodyframe;
  int foot_idx;
  Eigen::VectorXd pos_temp;
  Eigen::VectorXd ori_temp;

  my_utils::readParameter(motion_cfg, "foot", foot_idx);
  my_utils::readParameter(motion_cfg, "duration", md_temp.motion_period);
  my_utils::readParameter(motion_cfg, "swing_height", md_temp.swing_height);
  my_utils::readParameter(motion_cfg, "pos", pos_temp);
  my_utils::readParameter(motion_cfg, "ori", ori_temp);
  my_utils::readParameter(motion_cfg, "b_relative", is_bodyframe);
  md_temp.pose = POSE_DATA(pos_temp, ori_temp, is_bodyframe);

  MotionCommand motion_command = MotionCommand(foot_idx,md_temp);
  motion_command_script_list_.push_back(motion_command);
}