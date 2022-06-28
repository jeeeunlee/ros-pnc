#pragma once

#include <magneto_pnc/magneto_state_provider.hpp>
#include <pnc_core/state_machine.hpp>

class MagnetoControlSpecContainer;
class MagnetoReferenceGeneratorContainer;

class FullSupport : public StateMachine {
 public:
  FullSupport(const StateIdentifier state_identifier_in,
              MagnetoReferenceGeneratorContainer* rg_container);
  ~FullSupport();

  void oneStep();
  void firstVisit();
  void lastVisit();
  bool endOfState();
  void initialization(const YAML::Node& node);

 protected:
  MagnetoStateProvider* sp_;
  MagnetoControlSpecContainer* ws_container_;
  MagnetoReferenceGeneratorContainer* rg_container_;

  void _taskUpdate();
  void _weightUpdate();
};
