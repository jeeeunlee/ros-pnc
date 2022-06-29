#pragma once

#include <magneto_pnc/magneto_state_provider.hpp>
#include <pnc_core/state_machine.hpp>

class MagnetoControlSpecContainer;
class MagnetoReferenceGeneratorContainer;

class Swing : public StateMachine {
 public:
  Swing(const StateIdentifier state_identifier_in,
              MagnetoReferenceGeneratorContainer* rg_container);
  ~Swing();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

  void switchStateButtonTrigger() { state_switch_button_trigger_ = true; }

 protected:
  MagnetoStateProvider* sp_;
  MagnetoControlSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  int moving_foot_link_idx_; // link index
  int moving_foot_idx_; // link index


  bool state_switch_button_trigger_;

  void _taskUpdate();
  void _weightUpdate();
  void _ResidualMagnetismUpdate();
};