#include <magneto_test/hwtest/magneto_interface_node.hpp>

#define ACQUISTION_TIME 10
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
    int spin = ACQUISTION_TIME / MagnetoAux::servo_rate; //Spin cycles from time
    // while (spin--)
    // {
    //     ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nAcquiring robot state . . .\n");
    //     // End wait if joints are intitialised or debugging in rviz (joint states will never initialise).
    //     if (hw_interface.jointPositionsInitialised())
    //     {
    //         spin = 0;
    //     }
    //     ros::spinOnce();
    //     r.sleep();
    // }

    // Loop waiting for start button press
    // while (state.getSystemState() == SUSPENDED)
    // {
    //     if (use_default_joint_positions)
    //     {
    //     ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nFailed to initialise joint position values!\n");
    //     }
    //     ROS_INFO_THROTTLE(THROTTLE_PERIOD, "%s", start_message.c_str());
    //     ros::spinOnce();
    //     r.sleep();
    // }

    ROS_INFO("\nController started. Press START/BACK buttons to transition state of robot.\n");


    // hw_interface.init();
    // hw_interface.initModel(use_default_joint_positions);


    while (ros::ok())
    {
        if (hw_interface.getSystemState() != SUSPENDED)
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