#include <Eigen/Dense>
#include "my_ros_msgs/MagnetoCommand.h"
#include "my_ros_msgs/MagnetoSensorData.h"
#include "my_ros_msgs/MagnetismOnOff.h"
#include "my_ros_msgs/ForceWrench.h"
#include "my_ros_msgs/Quaternion.h"
#include "my_ros_msgs/JointData.h"

namespace my_ros_msgs{
    
    void Vector2JointData(const Eigen::VectorXd& vector, std::vector<double>& joint_data);
    void JointData2Vector(const std::vector<double>& joint_data, Eigen::VectorXd& vector);
    void Wrench2Vector(my_ros_msgs::ForceWrench wrench, Eigen::VectorXd& vector);
    void Quaternion2Matrix(my_ros_msgs::Quaternion quat, Eigen::MatrixXd& mat);
    
} // namespace my_ros_msgs