#include<autonomous_mobile_robot_2022/PointCloud.h>

void PointCloudClass::pcl_callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    pcl::fromROSMsg (*msg, rawcloud);
    //rawPC2=PC2=msg;
    manage();
}

void PointCloudClass::odom_callback(const nav_msgs::Odometry& msg)
{
    odom = msg;
}

// void PointCloudClass::encoder_callback(const beego_control::beego_encoder& msg)
// {
// 	if (!encoder_firsttime){
// 		encoder_time_pre = ros::Time::now();
// 		encoder_firsttime = true;
// 	}else{
// 		ros::Time encoder_time_now = ros::Time::now();
// 		double encoder_deltatime = encoder_time_now.toSec() - encoder_time_pre.toSec();
// 		//std::cout<< encoder_deltatime <<std::endl;
// 		double robot_vel = (-msg.vel.r + msg.vel.l) / 2.0;
// 		distance_traveled_robot = distance_traveled_robot + robot_vel * encoder_deltatime;
// 		//std::cout<< distance_traveled_robot <<std::endl;
// 		encoder_time_pre = encoder_time_now;
// 	}
//     //std::cout<< msg.vel <<std::endl;
// }

void PointCloudClass::depthimage_callback(const sensor_msgs::Image& msg)
{
    try{
        // bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
        bridgeImage=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::MONO16);//CV_16UC1
    }
    catch(cv_bridge::Exception& e) {//エラー処理
        std::cout<<"sensor_depth_image_callback Error \n";
        ROS_ERROR("Could not convert from '%s' to 'MONO16'.",
        msg.encoding.c_str());
        return ;
    }

    int imageheight = bridgeImage->image.rows;
    int imagewidth = bridgeImage->image.cols;
    int datanum = imageheight * imagewidth;
    cloudfromimage.data.resize(datanum);
    int cnt = 0;
    for (int imgrow = 0;imgrow < imageheight ; imgrow++)
    {
        for (int imgcol = 0;imgcol < imagewidth ; imgcol++)
        {
            unsigned short int depth = bridgeImage->image.at<unsigned short int>(imgrow,imgcol);
            if (depth > 0)
            {
                cloudfromimage.data[cnt].y = depth / 1000.0;
                cloudfromimage.data[cnt].z = (-(float)imgrow + (float)imageheight) * cloudfromimage.data[cnt].y / 350.0;//高さ算出,370.985,0.0021
                cloudfromimage.data[cnt].x = -((float)imgcol - (float)imagewidth / 2.0) * cloudfromimage.data[cnt].y / 350.0;//0.0021/0.000006=350.0

                cnt++;
            }
            
        }
    }
    cloudfromimage.data.resize(cnt);
    pubpc1.publish(cloudfromimage);
    cloudfromimage.data.clear();

}

//manage method
void PointCloudClass::manage(){
    Extract();
    publishPointCloud();
    clearMessages();
}

void PointCloudClass::Extract()
{

    int cnt=0;
    float X,Y,Z;
    float OCx = DISTANCE_TO_OBJECT * sin(abs(ANGLE_TO_OBJECT)/180.0*M_PI);
    if(ANGLE_TO_OBJECT < 0.0) OCx = -OCx;
    float OCy = DISTANCE_TO_OBJECT * cos(abs(ANGLE_TO_OBJECT)/180.0*M_PI);

    if(IS_MOVING)
    {
        OCy -= odom.pose.pose.position.x;
        std::cout<< odom.pose.pose.position <<std::endl;
    }

    float OCz = -CAMERAPOS_HEIGHT + OBJECTSIZE_HEIGHT / 2;

    float minX = OCx - OBJECTSIZE_WIDTH / 2.0 - 0.2;
    float maxX = OCx + OBJECTSIZE_WIDTH / 2.0 + 0.2;

    float minY = OCy - OBJECTSIZE_DEPTH / 2.0 - 0.2;
    float maxY = OCy + OBJECTSIZE_DEPTH / 2.0 + 0.2;

    float minZ = OCz - OBJECTSIZE_HEIGHT / 2.0 - 0.2;
    float maxZ = OCz + OBJECTSIZE_HEIGHT / 2.0 + 0.2;

    int datanum = rawcloud.height * rawcloud.width;
    cloud.data.resize(datanum);
    for(int i = 0 ; i < datanum; i++)
    {

        X = rawcloud.points[i].y;   //横幅
        Y = rawcloud.points[i].x;   //奥行
        Z = rawcloud.points[i].z;   //高さ

        if(!(X == 0.0 && Y == 0.0 && Z == 0.0))
        {

            if((X >= minX && X <= maxX) && (Y >= minY && Y <= maxY) && (Z >= minZ && Z <= maxZ))
            {
                cloud.data[cnt].x = X;
                cloud.data[cnt].y = Y;
                cloud.data[cnt].z = Z;
                //std::cout<< cloud.data[cnt] <<std::endl;
                cnt++;
            
            }
            
        }
    }
    std::cout<<"cnt:"<< cnt <<std::endl;
    cloud.data.resize(cnt);

}


void PointCloudClass::publishPointCloud(){//データ送信
    pubpc.publish(cloud);
}

void PointCloudClass::clearMessages(){
    rawcloud.clear();
    cloud.data.clear();
}
