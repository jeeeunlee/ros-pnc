#include <magneto_test/hwtest/magneto_interface_node.hpp>


#define JOINT_INITIALIZE_TIME 10


int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pnc");
    ros::NodeHandle n;
    MagnetoInterfaceNode hw_interface;
    ros::Rate r(round(1.0 / MagnetoAux::servo_rate ));

    // 1. Wait specified time to aquire all published joint positions via callback    
    while (ros::ok())
    {
        ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nAcquiring robot state . . .\n");
        // End wait if joints are intitialised or debugging in rviz (joint states will never initialise).
        if (hw_interface.systemInitialized())        
            break;
        
        ros::spinOnce();        
        r.sleep();
    }

    // 2. Loop waiting for initialize (once button pressed)
    ROS_INFO("\nController checked. Press START buttons to initialize joint position! \n");
    while( ros::ok() && !hw_interface.startButtonPressed() ) {
        ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nPress START buttons to initialize joint position!\n"); 
        ros::spinOnce();
    }               
    ROS_INFO("\nButton pressed. \n");
    int spin = JOINT_INITIALIZE_TIME/MagnetoAux::servo_rate;
    while ( ros::ok() && hw_interface.getSystemState() == CHECKED)    
    {        
        hw_interface.initializeJointConfiguration();
        hw_interface.publishMagnetOnoff();
        hw_interface.publishDesiredJointState();

        if(spin-- < 0)
            ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nFailed to initialise joint position values!\n");  
        
        ros::spinOnce();
        // ros::spin();
        r.sleep();
    }

    // 4. run the robot
    ROS_INFO("\nController started. Press START/BACK buttons to run the robot.\n");
    // while( ros::ok() && !hw_interface.startButtonPressed() ){
    //     ros::spinOnce();
    //     ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nPress START buttons to run the robot\n"); 
    // }       
    
    while (ros::ok())
    {
        if(hw_interface.stopButtonPressed())
            hw_interface.setStop();        

        if((hw_interface.getSystemState()==IDLE && !hw_interface.startButtonPressed()) ||
            (hw_interface.getSystemState()==READY && !hw_interface.startButtonPressed()) ){
            ROS_WARN_THROTTLE(THROTTLE_PERIOD, "\nPress START buttons to run the robot\n");  
        }      
        else{
            switch(hw_interface.getSystemState()){
            case IDLE:             
                ROS_INFO("\nButton pressed. \n");                       
                hw_interface.resetEstimator();   
                ROS_INFO("\nReset System...\n");
                break;
            case READY:                
                ROS_INFO("\nController Ready!, start running\n");
                hw_interface.enableButtonFlag('s'); // 's' : add script motions                 
                break;
            case RUNNING:            
                hw_interface.loop(); // main loop
                hw_interface.publishMagnetOnoff();
                hw_interface.publishDesiredJointState();        
                break;
            case STOP:
            default:
                if(hw_interface.startButtonPressed()) hw_interface.releaseStop();
                hw_interface.stoploop();
                hw_interface.publishMagnetOnoff();
                hw_interface.publishDesiredJointState();        
                ROS_INFO_THROTTLE(THROTTLE_PERIOD, "\nController suspended. Press Logitech button to resume . . .\n");
                break;
            }
        }      
       
        
        ros::spinOnce();
        // ros::spin();
        r.sleep();
    }


}