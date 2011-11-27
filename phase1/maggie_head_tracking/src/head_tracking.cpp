#include <ros/ros.h>

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

 #include <iostream>
 #include <stdio.h>
 
 #define FOV_WIDTH 	1.094
 #define FOV_HEIGHT 	1.094
 
 #define DEBUG_		0

 using namespace std;
 using namespace cv;
 namespace enc = sensor_msgs::image_encodings;

class HeadTracking
{
public:
  
  void detectFace( cv_bridge::CvImagePtr cv_ptr )
  {
    std::vector<Rect> faces;
    
    Mat frame = cv_ptr->image;
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    if( !face_cascade.load( face_cascade_name ) )
    { 
      printf("--(!)Error loading\n"); return; 
    };
  
    //-- Detect faces
    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );
    
    for( uint i = 0; i < faces.size(); i++ )
    {
      
      if (i < 3)
      {
	/* 
	 * For the moment, we're only interested in the first face that we pick up.
	 */ 
	faceCentre_ = Point( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
	
	if (DEBUG_)
	{
	  ellipse( frame, faceCentre_, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
	  image_pub_.publish(cv_ptr->toImageMsg());
	}
	return;
      }
    } 
    faceCentre_ = (Point)0;
  }

  void
  image_cb (const sensor_msgs::Image& rgbImage)
  {
    //std::cout << "image_cb" << std::endl;
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(rgbImage, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
     
    detectFace(cv_ptr);
  }

  void
  depth_cb (const sensor_msgs::Image& depthImage)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depthImage, enc::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    if (faceCentre_ != (Point)0)
    {
      float z = cv_ptr->image.at<float>(faceCentre_);
      if (isnan<float>(z) == false)
      {
	/*
	* We've got everything we need, so publish the faceCentre 3d coordinate to the target frame
	*/
	geometry_msgs::PointStamped point_out;          
	point_out.header.frame_id = "camera_depth_optical_frame"; 
	point_out.point.x = z * FOV_WIDTH * (faceCentre_.x - cv_ptr->image.size().width / 2.0) / cv_ptr->image.size().width;
	point_out.point.y = (z * FOV_HEIGHT * (faceCentre_.y - cv_ptr->image.size().height / 2.0) / cv_ptr->image.size().height) + 0.12;
	point_out.point.z = cv_ptr->image.at<float>(faceCentre_);
	
	if (DEBUG_)
	  std::cout << "COG_X = " << point_out.point.x << " y = " << point_out.point.y << " z = " << point_out.point.z << std::endl;
	
	target_pub_.publish<geometry_msgs::PointStamped>(point_out); 
      }
    }
  }
  
  HeadTracking () : image_topic_("/camera/rgb/image_color"),  depth_topic_("/camera/depth_registered/image")
  {
    face_cascade_name = "/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_frontalface_alt.xml";
    
    image_sub_ = nh_.subscribe (image_topic_, 2, &HeadTracking::image_cb, this);
    depth_sub_ = nh_.subscribe (depth_topic_, 2, &HeadTracking::depth_cb, this);
    
    target_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("target_point", 5);
    
    // This is only for debugging purposes, so can be commented-out
    if (DEBUG_)
      image_pub_ = nh_.advertise<sensor_msgs::Image> ("output", 30);
    
    //print some info about the node
    std::string r_ct = nh_.resolveName (depth_topic_);
    std::string r_it = nh_.resolveName (image_topic_);
    ROS_INFO_STREAM("Listening for incoming data on topic " << r_ct );
    ROS_INFO_STREAM("Publishing image on topic " << r_it );
  }
  
private:
  ros::NodeHandle nh_;

  std::string image_topic_; 
  std::string depth_topic_; 

  ros::Subscriber depth_sub_; //depth subscriber
  ros::Subscriber image_sub_; //image subscriber
  
  ros::Publisher image_pub_; //image message publisher
  ros::Publisher target_pub_;
  
  CascadeClassifier face_cascade;
  String face_cascade_name;
  
  Point faceCentre_; 
};

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "convert_pointcloud_to_image");
  HeadTracking ht; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
