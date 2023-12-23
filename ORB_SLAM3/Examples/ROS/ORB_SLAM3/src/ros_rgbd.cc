/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
    void GrabImageMask(const sensor_msgs::ImageConstPtr &img_msg);
    cv::Mat GetSemancticMask(const sensor_msgs::ImageConstPtr &img_msg);
    
    ORB_SLAM3::System* mpSLAM;
    
    std::mutex mBufMutexMask;
    queue<sensor_msgs::ImageConstPtr> imgMaskBuf;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();

    if(argc != 3)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD path_to_vocabulary path_to_settings" << endl;        
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::RGBD,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nh;
    //parking
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/color/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/aligned_depth_to_color/image_raw", 100);
    
    //TUM
    //message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_color", 100);
    //message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image", 100);
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    
    ros::Subscriber mask_sub = nh.subscribe("/semantic_mask", 10, &ImageGrabber::GrabImageMask, &igb);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

//void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const sensor_msgs::ImageConstPtr& msgM)

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    //Semantic Mask
    cv::Mat cv_ptrM;
    //cv::Mat imgMask;
    mBufMutexMask.lock();
    if (!imgMaskBuf.empty()){
        cv::Mat imgMask = GetSemancticMask(imgMaskBuf.front());
        try
        {
            cv_ptrM = imgMask;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    else{
        //parking
        cv::Mat imgMask = cv::Mat::zeros(cv::Size(720, 1280), CV_8UC1);
        
        //TUM
        //cv::Mat imgMask = cv::Mat::zeros(cv::Size(480, 640), CV_8UC1);
        
        std::cout << "Semantic Mask queue is empty" << std::endl;
        try
        {
            cv_ptrM = imgMask;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    mBufMutexMask.unlock();
    //cv::Mat imgMask = imread("/home/jeff/3DCV/SemanticMask.png", cv::IMREAD_GRAYSCALE);
    //cv_bridge::CvImageConstPtr cv_ptrM;
    
    // mpSLAM->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
    //std::cout << "A Mask Gen" << std::endl;
    mpSLAM->TrackRGBDWithMask(cv_ptrRGB->image,cv_ptrD->image,cv_ptrM,cv_ptrRGB->header.stamp.toSec());
}

void ImageGrabber::GrabImageMask(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexMask.lock();
  if (!imgMaskBuf.empty()){
    imgMaskBuf.pop();
    //std::cout << "refresh Mask" << std::endl;
  }
  imgMaskBuf.push(img_msg);
  mBufMutexMask.unlock();
}

cv::Mat ImageGrabber::GetSemancticMask(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat cv_ptr_resize;
  cv_ptr = cv_bridge::toCvShare(img_msg);
  //cv::resize(cv_ptr->image.clone(), cv_ptr_resize, cv::Size(720, 1280), cv::INTER_NEAREST);
  return cv_ptr->image.clone();
  
  /*
  try
  {
    std::cout << "1 Mask Gen" << std::endl;
    cv_ptr = cv_bridge::toCvShare(img_msg);
    std::cout << "2 Mask Gen" << std::endl;
    cv::resize(cv_ptr->image.clone(), cv_ptr_resize, cv::Size(720, 1280), cv::INTER_NEAREST);
    std::cout << "3 Mask Gen" << std::endl;
    return cv_ptr_resize;
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_INFO("CV image exception");
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cv_ptr->image.clone();
  }
  */
}

