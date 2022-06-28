#pragma once

#include <pnc_utils/Math/BSplineBasic.h>
#include <magneto_pnc/magneto_state_provider.hpp>
#include <pnc_core/state_machine.hpp>

class MagnetoControlSpecContainer;
class MagnetoReferenceGeneratorContainer;

class Transition : public StateMachine {
 public:
  Transition(const StateIdentifier state_identifier_in,
              MagnetoReferenceGeneratorContainer* rg_container, 
              bool contact_start);
  ~Transition();

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

  double trans_duration_;

  bool b_contact_start_;
  bool state_switch_button_trigger_;

  int moving_foot_link_idx_;
  int moving_foot_idx_;

  void _taskUpdate();
  void _weightUpdate();
};
