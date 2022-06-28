#include <magneto_pnc/magneto_wbc_controller/tasks/com_task.hpp>
#include <magneto_pnc/magneto_definition.hpp>
#include <pnc_utils/Configuration.h>
#include <pnc_utils/io_utilities.hpp>

CoMTask::CoMTask(RobotSystem* robot):Task(robot, 3)
{
    pnc_utils::pretty_constructor(3, "COM XYZ Task");
    Jt_ = Eigen::MatrixXd::Zero(dim_task_, robot_->getNumDofs());
    JtDotQdot_ = Eigen::VectorXd::Zero(dim_task_);   
}

CoMTask::~CoMTask(){}

bool CoMTask::_UpdateCommand(const Eigen::VectorXd & _pos_des,
                            const Eigen::VectorXd & _vel_des,
                            const Eigen::VectorXd & _acc_des) {
    // x,y,z (position)
    // update     
    // For Dyn WBC: op_cmd;
    // For Kin WBC: pos_err; vel_des; acc_des;

    pos_err.setZero();
    pos_err = _pos_des - robot_->getCoMPosition();
    // pos_err = _vel_des * control_period;
    vel_des = _vel_des;
    acc_des = _acc_des;

    //0112 pnc_utils::saveVector(pos_err, "com_pos_err");
    // pnc_utils::pretty_print(_pos_des, std::cout, "_pos_des  @  _UpdateCommand");
    // pnc_utils::pretty_print(_vel_des, std::cout, "_vel_des  @  _UpdateCommand");
    // pnc_utils::pretty_print(_acc_des, std::cout, "_acc_des  @  _UpdateCommand");

    return true;
}

bool CoMTask::_UpdateTaskJacobian(){
    // (X, Y, Z), // 6x30
    Eigen::MatrixXd Jtmp = robot_->getCoMJacobian(); 
    // std::cout<< "COM Jacobian size: " << Jtmp.rows() << ", " << Jtmp.cols() << std::endl;

    Jt_ = Jtmp.block(3, 0, 3, robot_->getNumDofs());

    return true;
}

bool CoMTask::_UpdateTaskJDotQdot(){
    JtDotQdot_.setZero();
    return true;
}
