#pragma once

#include <vector>
#include <array>

class RobotSystem;
class Task;
class ContactSpec;

// Object which publicly contains all the tasks, contacts and reaction forces
class ControlSpecContainer {
 public:
  ControlSpecContainer(RobotSystem* _robot){
    robot_ = _robot;  
  }
  ~ControlSpecContainer(){} 

 public:
  RobotSystem* robot_;
  std::vector<Task*> task_list_;
  std::vector<ContactSpec*> contact_list_;  
};
