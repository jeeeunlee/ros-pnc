#include <magneto_test/hwtest/magneto_interface_node.hpp>

#define JOINT_INITIALIZE_TIME 10
#define THROTTLE_PERIOD 5

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "shc");
    ros::NodeHandle n;

    MagnetoInterfaceNode hw_interface;
    const MagnetoInterfaceParams& params = hw_interface.getParameters();

    // INITIALIZE PARAMS

    // Set ros rate from params
    ros::Rate r(round(1.0 / MagnetoAux::servo_rate ));

    // Wait specified time to aquire all published joint positions via callback    
    while (true)
    {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nAcquiring robot state . . .\n");
        // End wait if joints are intitialised or debugging in rviz (joint states will never initialise).
        if (hw_interface.systemInitialized())        
            break;
        
        ros::spinOnce();
        r.sleep();
    }

    // Loop waiting for initialize button press
    ROS_INFO("\nController checked. Press START buttons to initialize joint position \n");
    int spin = JOINT_INITIALIZE_TIME/ MagnetoAux::servo_rate;
    // while(spin--)
    while (hw_interface.getSystemState() == CHECKED)
    {
        if (spin-- < 0)        
            ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nFailed to initialise joint position values!\n");
        
        hw_interface.initializeJointConfiguration();
        hw_interface.publishMagnetOnoff();
        hw_interface.publishDesiredJointState();
        
        ros::spinOnce();
        r.sleep();
    }

    ROS_INFO("\nController started. Press START/BACK buttons to transition state of robot.\n");


    // hw_interface.init();
    // hw_interface.initModel(use_default_joint_positions);


    while (ros::ok())
    {
        // waiting for reset Estimator
        if (hw_interface.getSystemState() == IDLE){
            ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController Idle, Press Reset!\n");
            
        } // waiting for user input
        else if (hw_interface.getSystemState() == READY){
            ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController Ready, Press Run!\n");
        }
        else if (hw_interface.getSystemState() == RUNNING)
        {
            hw_interface.loop(); // main loop
            hw_interface.publishMagnetOnoff();
            hw_interface.publishDesiredJointState();
        }
        else
        {
            ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController suspended. Press Logitech button to resume . . .\n");
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;

}