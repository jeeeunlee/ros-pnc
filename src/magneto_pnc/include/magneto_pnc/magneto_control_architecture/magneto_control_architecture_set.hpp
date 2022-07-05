#include <magneto_pnc/magneto_control_architecture/mpc_architecture.hpp>
#include <magneto_pnc/magneto_control_architecture/wbmc_architecture.hpp>

namespace MAGNETO_STATES {
constexpr int INITIALIZE = 0;
constexpr int BALANCE = 1; // DEFAULT
constexpr int SWING_START_TRANS = 2;
constexpr int SWING = 3;
constexpr int SWING_END_TRANS = 4;
constexpr int IDLE = 5;
};  // namespace MAGNETO_STATES

class MotionCommand;
class MagnetoUserStateCommand {
   public:
   MagnetoUserStateCommand(){
       state_id = -1;
       user_cmd = MotionCommand();
   }
   void setCommand(int _state_id, const MotionCommand& _state_cmd) {
       state_id = _state_id;
       user_cmd = _state_cmd;
   }
   int state_id;
   MotionCommand user_cmd;
};