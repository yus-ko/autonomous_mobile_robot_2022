#include<autonomous_mobile_robot_2022/RobotControl.h>

void RobotControlClass::encoder_callback(const geometry_msgs::Twist& msg)
{
    encoder_value = msg;
    manage();
}

void RobotControlClass::encoder_callback_sim(const nav_msgs::Odometry& msg)
{
    encoder_value = msg.twist.twist;
    //std::cout<<"encoder_callback_sim"<<std::endl;
    manage();
}

void RobotControlClass::manage()
{
    odometry();
    pid_control();
    publishcmd();
}

void RobotControlClass::odometry()
{
    if (encoder_firsttime)
    {
		encoder_time_pre = ros::Time::now();
		encoder_firsttime = false;
        odom.twist.twist.linear.x = odom.twist.twist.linear.y = odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = odom.twist.twist.angular.y = odom.twist.twist.angular.z = 0.0;
        odom.pose.pose.position.x = odom.pose.pose.position.y = odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = odom.pose.pose.orientation.y = odom.pose.pose.orientation.z = 0.0;
        odom.pose.pose.orientation.w = 1.0;
	}
    else
    {
        ros::Time encoder_time_now = ros::Time::now();
		encoder_deltatime = encoder_time_now.toSec() - encoder_time_pre.toSec();
		odom.pose.pose.orientation.z += encoder_value.angular.z * encoder_deltatime;
		odom.pose.pose.position.x += encoder_value.linear.x * cos(odom.pose.pose.orientation.z) * encoder_deltatime;
        odom.pose.pose.position.y += encoder_value.linear.x * sin(odom.pose.pose.orientation.z) * encoder_deltatime;

        std::cout<< encoder_value <<std::endl;
		std::cout<< odom.pose.pose.position <<std::endl;
        std::cout<< odom.pose.pose.orientation <<std::endl;

		encoder_time_pre = encoder_time_now;
    }

    if (!done_turn && abs(odom.pose.pose.orientation.z) >= atan(TARGET_POSITION_Y / TARGET_POSITION_X)*0.99 && abs(odom.pose.pose.orientation.z) <= atan(TARGET_POSITION_Y / TARGET_POSITION_X)*1.01) done_turn = true;
    if (!done_straight && sqrt(odom.pose.pose.position.x * odom.pose.pose.position.x + odom.pose.pose.position.y * odom.pose.pose.position.y) > sqrt(TARGET_POSITION_X * TARGET_POSITION_X + TARGET_POSITION_Y * TARGET_POSITION_Y)) done_straight = true;

}


void RobotControlClass::pid_control()
{

    cmd.linear.x = cmd.linear.y = cmd.linear.z = 0.0;
    cmd.angular.x = cmd.angular.y = cmd.angular.z = 0.0;

    cmd.linear.x = MAX_VELOCITY;
	cmd.angular.x = -odom.pose.pose.orientation.z;

    // if (!done_turn)
    // {
    //     //角速度
    //     double error_angvel = MAX_ANGULAR_VELOCITY - encoder_value.angular.z;
    //     integral_angvel_error += error_angvel * encoder_deltatime;  //台形近似にするかも

    //     cmd.angular.z = (GAIN_PROPORTIONAL * error_angvel) + (GAIN_INTEGRAL * integral_angvel_error) + (GAIN_DIFFERENTIAL * (error_angvel - error_angvel_pre) / encoder_deltatime);

    //     error_angvel_pre = error_angvel;
    //     return;
    // }

    // if (!done_straight)
    // {
    //     //速度
    //     double error_vel = MAX_VELOCITY - encoder_value.linear.x;
    //     integral_vel_error += error_vel * encoder_deltatime;  //台形近似にするかも

    //     cmd.linear.x = (GAIN_PROPORTIONAL * error_vel) + (GAIN_INTEGRAL * integral_vel_error) + (GAIN_DIFFERENTIAL * (error_vel - error_vel_pre) / encoder_deltatime);

    //     error_vel_pre = error_vel;

    // }
    

}

void RobotControlClass::publishcmd()
{
    if(!PUBLISH_COMMAND) return;
    pub_cmd.publish(cmd);
}