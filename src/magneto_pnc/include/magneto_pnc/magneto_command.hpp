#pragma once

#include <pnc_utils/robot_command.hpp>

class SimulationCommand  {
  public:
    SimulationCommand(): mu(0.7), f_adhesive(100.) {}
    SimulationCommand(double _mu, double _fm): mu(_mu), f_adhesive(_fm) {}
    ~SimulationCommand() {};
    void getSimEnv(double& _mu, double& _fm){
      _mu = mu;
      _fm = f_adhesive;
    }

  protected:
    double mu;
    double f_adhesive;
};

class SimMotionCommand : public MotionCommand, public SimulationCommand {
  public:
    SimMotionCommand(): MotionCommand(), SimulationCommand() {}
    SimMotionCommand(const MotionCommand& motion_cmd)
                    : MotionCommand(motion_cmd), SimulationCommand() {}
    SimMotionCommand(const MotionCommand& motion_cmd,
                    double _mu,  double _fm )
                    : MotionCommand(motion_cmd), SimulationCommand(_mu, _fm) {}
    // SimMotionCommand(const SimMotionCommand& sim_motion_cmd);
    ~SimMotionCommand() {}
};

