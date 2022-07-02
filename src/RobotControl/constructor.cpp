#include<autonomous_mobile_robot_2022/RobotControl.h>

RobotControlClass::RobotControlClass()
{

	setLaunchParam();	// lanchファイルの読み込み

	if(IS_SIMULATOR)
	{
		sub_encoder=nhSub.subscribe("/vmegarover/diff_drive_controller/odom",1,&RobotControlClass::encoder_callback_sim,this);
		pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/vmegarover/diff_drive_controller/cmd_vel", 1);
	}
	else
	{
		sub_encoder=nhSub.subscribe("/rover_odo",1,&RobotControlClass::encoder_callback,this);
		pub_cmd= nhPub.advertise<geometry_msgs::Twist>("/rover_twist", 1);
	}
	
}
RobotControlClass::~RobotControlClass(){
}
