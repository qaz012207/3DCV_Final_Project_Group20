#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"
#include"../include/ImuTypes.h"

class ImuGrabber
{
public:
    ImuGrabber(){};
    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM, ImuGrabber *pImuGb, const bool bClahe): mpSLAM(pSLAM), mpImuGb(pImuGb), mbClahe(bClahe){}

    void GrabImageRGB(const sensor_msgs::ImageConstPtr& msg);
    void GrabImageD(const sensor_msgs::ImageConstPtr& msg);
    cv::Mat GetImageRGB(const sensor_msgs::ImageConstPtr &img_msg);
    cv::Mat GetImageD(const sensor_msgs::ImageConstPtr &img_msg);
    void SyncWithImu();
    
    queue<sensor_msgs::ImageConstPtr> imgRGBBuf, imgDBuf;
    std::mutex mBufMutexRGB,mBufMutexD;
    
   
    ORB_SLAM3::System* mpSLAM;
    ImuGrabber *mpImuGb;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    bool bEqual = false;
    
    if(argc < 3 || argc > 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 RGBD-Inertial path_to_vocabulary path_to_settings [do_equalize]" << endl;
        ros::shutdown();
        return 1;
    }
  

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_RGBD,true);

    //ImageGrabber igb(&SLAM);
    ImuGrabber imugb;
    ImageGrabber igb(&SLAM,&imugb,bEqual);
    

    ros::Subscriber rgb_sub = n.subscribe("/camera/color/image_raw", 100, &ImageGrabber::GrabImageRGB,&igb);
    ros::Subscriber depth_sub = n.subscribe("/camera/aligned_depth_to_color/image_raw", 100, &ImageGrabber::GrabImageD,&igb);
    ros::Subscriber imu_sub = n.subscribe("/camera/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    
    //typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    //message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub);
    //sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2));
    
    std::thread sync_thread(&ImageGrabber::SyncWithImu,&igb);
    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImageRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexRGB.lock();
  if (!imgRGBBuf.empty())
    imgRGBBuf.pop();
  imgRGBBuf.push(img_msg);
  mBufMutexRGB.unlock();
}

void ImageGrabber::GrabImageD(const sensor_msgs::ImageConstPtr &img_msg)
{
  mBufMutexD.lock();
  if (!imgDBuf.empty())
    imgDBuf.pop();
  imgDBuf.push(img_msg);
  mBufMutexD.unlock();
}

cv::Mat ImageGrabber::GetImageRGB(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
    return cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_INFO("CV image exception");
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cv_ptr->image.clone();
  }
}

cv::Mat ImageGrabber::GetImageD(const sensor_msgs::ImageConstPtr &img_msg)
{
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img_msg);
    return cv_ptr->image.clone();
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_INFO("CV image exception");
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return cv_ptr->image.clone();
  }
}

void ImageGrabber::SyncWithImu()
{
  const double maxTimeDiff = 0.01;
  while(1)
  {
    cv::Mat imRGB, imD;
    double tImRGB = 0, tImD = 0;
    if (!imgRGBBuf.empty()&&!imgDBuf.empty()&&!mpImuGb->imuBuf.empty())
    {
      tImRGB = imgRGBBuf.front()->header.stamp.toSec();
      tImD = imgDBuf.front()->header.stamp.toSec();

      this->mBufMutexD.lock();
      while((tImRGB-tImD)>maxTimeDiff && imgDBuf.size()>1)
      {
        imgDBuf.pop();
        tImD = imgDBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexD.unlock();

      this->mBufMutexRGB.lock();
      while((tImD-tImRGB)>maxTimeDiff && imgRGBBuf.size()>1)
      {
        imgRGBBuf.pop();
        tImRGB = imgRGBBuf.front()->header.stamp.toSec();
      }
      this->mBufMutexRGB.unlock();

      if((tImRGB-tImD)>maxTimeDiff || (tImD-tImRGB)>maxTimeDiff)
      {
        // std::cout << "big time difference" << std::endl;
        continue;
      }
      if(tImRGB>mpImuGb->imuBuf.back()->header.stamp.toSec())
          continue;

      this->mBufMutexRGB.lock();
      imRGB = GetImageRGB(imgRGBBuf.front());
      imgRGBBuf.pop();
      this->mBufMutexRGB.unlock();

      this->mBufMutexD.lock();
      imD = GetImageD(imgDBuf.front());
      imgDBuf.pop();
      this->mBufMutexD.unlock();

      vector<ORB_SLAM3::IMU::Point> vImuMeas;
      mpImuGb->mBufMutex.lock();
      if(!mpImuGb->imuBuf.empty())
      {
        // Load imu measurements from buffer
        vImuMeas.clear();
        while(!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec()<=tImRGB)
        {
          double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
          cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x, mpImuGb->imuBuf.front()->linear_acceleration.y, mpImuGb->imuBuf.front()->linear_acceleration.z);
          cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x, mpImuGb->imuBuf.front()->angular_velocity.y, mpImuGb->imuBuf.front()->angular_velocity.z);
          vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc,gyr,t));
          mpImuGb->imuBuf.pop();
        }
      }
      mpImuGb->mBufMutex.unlock();
      if(mbClahe)
      {
        mClahe->apply(imRGB,imRGB);
        mClahe->apply(imD,imD);
      }


      mpSLAM->TrackRGBD(imRGB,imD,tImRGB,vImuMeas);

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mBufMutex.lock();
  imuBuf.push(imu_msg);
  mBufMutex.unlock();
  return;
}
