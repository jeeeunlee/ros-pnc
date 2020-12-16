#include "ros/ros.h"
#include <std_msgs/UInt16.h>
#include "my_ros_msgs/MagnetoCommand.h"
#include "my_ros_msgs/MagnetoSensorData.h"
#include "my_ros_msgs/MagnetismOnOff.h"
#include "my_ros_msgs/ForceWrench.h"
#include "my_ros_msgs/Quaternion.h"
#include "my_ros_msgs/JointData.h"

#include <my_pnc/MagnetoPnC/MagnetoInterface.hpp>
#include <my_ros_msgs/MsgConvertFuction.hpp>

#include <mutex>

EnvInterface* interface_;
MagnetoSensorData* sensor_data_;
MagnetoCommand* command_;

my_ros_msgs::MagnetoCommand command_msg;
my_ros_msgs::MagnetismOnOff mag_onoff_msg;
ros::Publisher pub_cmd_;

void InterruptLogicCallBack(const std_msgs::UInt16::ConstPtr& key){
    // uint16_t key
    ROS_INFO("InterruptLogicCallBack");
    std::cout << "button(" << (char)key->data << ") pressed handled @ MagnetoPncNode::enableButtonFlag" << std::endl;
    ((MagnetoInterface*)interface_) -> interrupt_ -> setFlags(key->data); 
}

void MagnetoDataCallBack(const my_ros_msgs::MagnetoSensorDataConstPtr data) {
    // todo : data -> sensor_data
    ROS_INFO("MagnetoDataCallBack  -- start");    

    // msg -> sensordata
    my_ros_msgs::JointData2Vector(data->active_joint_position.data, sensor_data_->q);
    my_ros_msgs::JointData2Vector(data->active_joint_velocity.data, sensor_data_->qdot);
    my_ros_msgs::JointData2Vector(data->virtual_joint_position.data, sensor_data_->virtual_q);
    my_ros_msgs::JointData2Vector(data->virtual_joint_velocity.data, sensor_data_->virtual_qdot);
    my_ros_msgs::Wrench2Vector(data->alf_wrench, sensor_data_->alf_wrench);
    my_ros_msgs::Wrench2Vector(data->blf_wrench, sensor_data_->blf_wrench);
    my_ros_msgs::Wrench2Vector(data->arf_wrench, sensor_data_->arf_wrench);
    my_ros_msgs::Wrench2Vector(data->brf_wrench, sensor_data_->brf_wrench);
    sensor_data_->alfoot_contact = data->alfoot_contact;
    sensor_data_->blfoot_contact = data->blfoot_contact;
    sensor_data_->arfoot_contact = data->arfoot_contact;
    sensor_data_->brfoot_contact = data->brfoot_contact;
    my_ros_msgs::Quaternion2Matrix(data->ground_rot, sensor_data_->R_ground);
    
    // getCommand
    ROS_INFO("MagnetoDataCallBack  -- getCommand"); 
    ((MagnetoInterface*)interface_)->getCommand(sensor_data_, command_);

    // cmd -> msg
    ROS_INFO("MagnetoDataCallBack  -- cmd -> msg");
    my_ros_msgs::Vector2JointData(command_->q, command_msg.joint_position.data);
    my_ros_msgs::Vector2JointData(command_->qdot, command_msg.joint_velocity.data);
    my_ros_msgs::Vector2JointData(command_->jtrq, command_msg.joint_torque.data);            
    command_msg.magnet_onoffs.clear();
    for(auto &knv : command_->b_magnetism_map) {
        mag_onoff_msg.link_name = "todo";
        mag_onoff_msg.link_idx = knv.first;
        mag_onoff_msg.onoff = knv.second;
        command_msg.magnet_onoffs.push_back(mag_onoff_msg);
    }

    // pub
    pub_cmd_.publish(command_msg);
    ROS_WARN("MagnetoCmdPublish  -- end");   
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_pnc");
    ros::NodeHandle nh("~");    

    interface_ = new MagnetoInterface();
    sensor_data_ = new MagnetoSensorData();
    command_ = new MagnetoCommand();

    ros::Rate loop_rate(1000); // 1000 Hz
    ros::AsyncSpinner spinner(0);
    spinner.start();

    pub_cmd_ = nh.advertise<my_ros_msgs::MagnetoCommand >("/magneto_cmd", 1000);
    ros::Subscriber sub_data_ = nh.subscribe("/magneto_data", 1000, MagnetoDataCallBack);
    ros::Subscriber sub_interrupt_logic_ = nh.subscribe("/interrupt_logic", 1000, InterruptLogicCallBack);

    // ros::spin();
    while(ros::ok())
    {
        loop_rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}