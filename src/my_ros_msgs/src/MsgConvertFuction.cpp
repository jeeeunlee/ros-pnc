#include "my_ros_msgs/MsgConvertFuction.hpp"

namespace my_ros_msgs{
    
void Vector2JointData(const Eigen::VectorXd& vector, std::vector<double>& joint_data){
    joint_data.clear();
    for(int i(0); i<vector.size(); ++i){
        joint_data.push_back(vector(i));
    }
}

void JointData2Vector(const std::vector<double>& joint_data, Eigen::VectorXd& vector){
    // double* data = &joint_data[0];
    // vector = Eigen::VectorXd(data);
    // vector.resize(joint_data.size());
    // for( auto &q : joint_data )
    //     vector(i) = q;
    int vsize = joint_data.size();
    vector.resize(vsize);
    for(int i(0); i<vsize; ++i){
        vector(i) = joint_data[i];
    }
}

void Wrench2Vector(my_ros_msgs::ForceWrench wrench, Eigen::VectorXd& vector){
    vector = Eigen::VectorXd::Zero(6);
    vector(0) = wrench.rx; 
    vector(1) = wrench.ry;
    vector(2) = wrench.rz;
    vector(3) = wrench.x;
    vector(4) = wrench.y;
    vector(5) = wrench.z;      
}

void Quaternion2Matrix(my_ros_msgs::Quaternion quat, Eigen::MatrixXd& mat){
    Eigen::Quaterniond q;
    q.x() = quat.x;
    q.y() = quat.y;
    q.z() = quat.z;
    q.w() = quat.w;

    mat = q.normalized().toRotationMatrix();
}
} // namespace my_ros_msgs