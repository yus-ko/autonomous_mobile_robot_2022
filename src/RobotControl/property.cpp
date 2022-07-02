#include<autonomous_mobile_robot_2022/RobotControl.h>

void RobotControlClass::setLaunchParam(){
    
    ros::NodeHandle n("~");
    n.getParam("IS_SIMULATOR",IS_SIMULATOR);
    n.getParam("PUBLISH_COMMAND",PUBLISH_COMMAND);
    n.getParam("MAX_VELOCITY",MAX_VELOCITY);
    n.getParam("MAX_ANGULAR_VELOCITY",MAX_ANGULAR_VELOCITY);
    n.getParam("TARGET_POSITION_X",TARGET_POSITION_X);
    n.getParam("TARGET_POSITION_Y",TARGET_POSITION_Y);
    n.getParam("GAIN_PROPORTIONAL",GAIN_PROPORTIONAL);
    n.getParam("GAIN_INTEGRAL",GAIN_INTEGRAL);
    n.getParam("GAIN_DIFFERENTIAL",GAIN_DIFFERENTIAL);

}