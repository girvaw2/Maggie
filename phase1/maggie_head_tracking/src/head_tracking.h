#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/Image.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/CvBridge.h>
#include "geometry_msgs/PointStamped.h"
#include "dynamixel_controllers/SetSpeed.h"
#include <sensor_msgs/JointState.h>
#include "std_msgs/String.h"
#include <boost/algorithm/string.hpp>


#include <iostream>
#include <stdio.h>

#define FOV_WIDTH 	1.094
#define FOV_HEIGHT 	1.094

#define DEBUG_			0
#define DISABLE_HEAD_MOVEMENT	0

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;

class HeadTracking
{
  public:

  HeadTracking ();
  
  void detectFace( cv_bridge::CvImagePtr cv_ptr );
  void trackFace(cv_bridge::CvImagePtr cv_ptr);
  void adjustSpeedForDisplacement(float x, float y);
  void SetPanSpeed (float speed);
  void SetTiltSpeed (float speed);
  void scanForFace();
  void setHeadPosition(string frame, float x, float y, float z);
  void controllerCallback(const std_msgs::String::ConstPtr& msg);
  void jointState_cb (const sensor_msgs::JointState& state);
  void image_cb (const sensor_msgs::Image& rgbImage);
  void depth_cb (const sensor_msgs::Image& depthImage);
  void faceTimerCallback(const ros::TimerEvent& event);

private:
      
  typedef struct _scanControl
  {
    bool scanning;
    int targetAngle;
    uint step;
    int timeout;
  } ScanControl;
  
  ros::NodeHandle nh_;

  std::string image_topic_; 
  std::string depth_topic_; 
  std::string joint_state_topic_; 

  ros::Subscriber depth_sub_; //depth subscriber
  ros::Subscriber image_sub_; //image subscriber
  
  ros::Subscriber joint_state_sub_;
  ros::Subscriber controller_sub_;
  
  ros::Publisher image_pub_; //image message publisher
  ros::Publisher target_pub_;
  
  ros::ServiceClient pan_speed_client_;
  ros::ServiceClient tilt_speed_client_;
  dynamixel_controllers::SetSpeed speed_srv_;
  
  bool started_;
  
  float panAngle_;
  float tiltAngle_;
  
  CascadeClassifier face_cascade;
  String face_cascade_name;
  
  Point faceCentre_; 
  
  ros::Timer faceTimer_;

  ScanControl scanControl_;
  
  cv_bridge::CvImagePtr cv_ptr_;
};
