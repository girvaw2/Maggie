#include "head_tracking.h"

void 
HeadTracking::detectFace( cv_bridge::CvImagePtr cv_ptr )
{
  std::vector<Rect> faces;
  
  Mat frame = cv_ptr->image;
  Mat frame_gray;
  
  cvtColor( frame, frame_gray, CV_BGR2GRAY ); 
  equalizeHist( frame_gray, frame_gray );

  //-- Detect faces
  face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30));
  
  for( uint i = 0; i < faces.size(); i++ )
  {
    if (i == 0)
    {
      /* 
	* For the moment, we're only interested in the first face that we pick up.
	*/ 
      faceCentre_ = Point( (faces[i].x) + faces[i].width*0.5, (faces[i].y) + faces[i].height*0.5 );
      
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
HeadTracking::trackFace(cv_bridge::CvImagePtr cv_ptr)
{
  float z = cv_ptr->image.at<float>(faceCentre_);
  if (isnan<float>(z) == false)
  {
    float x = z * FOV_WIDTH * (faceCentre_.x - cv_ptr->image.size().width / 2.0) / cv_ptr->image.size().width;
    float y = (z * FOV_HEIGHT * (faceCentre_.y - cv_ptr->image.size().height / 2.0) / cv_ptr->image.size().height); // + 0.12;
    
    adjustSpeedForDisplacement(x, y);
  
    /*
    * We've got everything we need, so publish the faceCentre 3d coordinate to the target frame
    */
    setHeadPosition("camera_depth_optical_frame", x, y, cv_ptr->image.at<float>(faceCentre_));
  }
}
  
void
HeadTracking::adjustSpeedForDisplacement(float x, float y) 
{
  if (DEBUG_)
  {
    ROS_INFO("Setting Pan Speed:  %f", min<float>(max<float>(abs(x) *1.5, 0.05), 0.5));
  }
  
  SetPanSpeed(min<float>(max<float>(abs(x) *1.5, 0.01), 0.75));
  SetTiltSpeed(min<float>(max<float>(abs(y) / 1.5, 0.01), 0.3));
}
  
void
HeadTracking::SetPanSpeed (float speed)
{
  speed_srv_.request.speed = speed;
  if (!pan_speed_client_.call(speed_srv_))
  {
    std::cout << "Failed to set pan speed: " << speed << std::endl;
  }
}

void
HeadTracking::SetTiltSpeed (float speed)
{
  speed_srv_.request.speed = speed;
  if (!tilt_speed_client_.call(speed_srv_))
  {
    std::cout << "Failed to set tilt speed: " << speed << std::endl;
  }
}

void 
HeadTracking::scanForFace()
{
  if (!scanControl_.scanning)
  {
    SetPanSpeed(0.5);
    scanControl_.scanning = true;
    if (scanControl_.step == 0)
    {
      setHeadPosition("base_link", 0, 100, 0);
      scanControl_.targetAngle = 1;
    }
    else
    if (scanControl_.step == 1)
    {
      setHeadPosition("base_link", 0, -100, 0);
      scanControl_.targetAngle = -1;
    }
    else
    {
      setHeadPosition("base_link", 100, 0, 0);
      scanControl_.scanning = false;
    }
  }
}
  
void
HeadTracking::setHeadPosition(string frame, float x, float y, float z)
{
    geometry_msgs::PointStamped point_out;          
    point_out.header.frame_id = frame; 
    point_out.point.x = x; 
    point_out.point.y = y + 0.12; 
    point_out.point.z = z; 
    
    if (DEBUG_)
    {
      ROS_INFO ( "Target coordinates: x = %f, y = %f, z = %f", point_out.point.x, point_out.point.y, point_out.point.z);
    }
    
    if (DISABLE_HEAD_MOVEMENT == 0)
      target_pub_.publish<geometry_msgs::PointStamped>(point_out); 
}
  
void 
HeadTracking::controllerCallback(const std_msgs::String::ConstPtr& msg)
{
  string str = msg->data;
  boost::algorithm::to_lower(str);
  if (str == "start")
  {
    started_ = true;
    faceTimer_.start();
  }
  else
  if (str == "stop")
  {
    started_ = false;
    faceTimer_.stop();
  }
}
    
void
HeadTracking::jointState_cb (const sensor_msgs::JointState& state)
{
  if (DEBUG_)
  {
    ROS_INFO("[pan angle: %f  tilt angle: %f]", state.position.at(0), state.position.at(1));
  }

  if (!started_)
    return;

  panAngle_ = state.position.at(0);
  tiltAngle_ = state.position.at(1);
  
  if (scanControl_.scanning)
  {
    if (scanControl_.step == 0 && panAngle_ > scanControl_.targetAngle)
    {
      scanControl_.step++;
      scanControl_.scanning = false;
    }
    else
    if (scanControl_.step == 1 && panAngle_ < scanControl_.targetAngle)
    {
      scanControl_.step++;
      scanControl_.scanning = false;
    }
  }
}

void
HeadTracking::image_cb (const sensor_msgs::Image& rgbImage)
{
  if (!started_)
    return;
  
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
  
  // set the global image pointer
  cv_ptr_ = cv_ptr;
}

void
HeadTracking::depth_cb (const sensor_msgs::Image& depthImage)
{
  if (!started_)
    return;
  
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
    trackFace(cv_ptr);
    memset(&scanControl_, 0, sizeof (scanControl_));
  }
}
  
void 
HeadTracking::faceTimerCallback(const ros::TimerEvent& event)
{
  if (cv_ptr_)
    detectFace(cv_ptr_);
  
  if (++scanControl_.timeout > 40)
    scanForFace();
}

HeadTracking::HeadTracking () : image_topic_("/camera/rgb/image_color"),  depth_topic_("/camera/depth_registered/image"), joint_state_topic_("/joint_states")
{
  face_cascade_name = "/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_frontalface_alt.xml";
  if( !face_cascade.load( face_cascade_name ) )
  { 
    printf("--(!)Error loading\n"); return; 
  };
  
  image_sub_ = nh_.subscribe (image_topic_, 1, &HeadTracking::image_cb, this);
  depth_sub_ = nh_.subscribe (depth_topic_, 1, &HeadTracking::depth_cb, this);
  
  joint_state_sub_ = nh_.subscribe (joint_state_topic_, 1, &HeadTracking::jointState_cb, this);
  
  target_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("target_point", 1);
  
  controller_sub_ = nh_.subscribe("head_tracking", 1000, &HeadTracking::controllerCallback, this);
  
  faceTimer_ = nh_.createTimer(ros::Duration(0.1), &HeadTracking::faceTimerCallback, this);
  faceTimer_.stop();
  
  pan_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/dynamixel_controller/head_pan_controller/set_speed");
  tilt_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/dynamixel_controller/head_tilt_controller/set_speed");
  
  SetPanSpeed(0.05);
  SetTiltSpeed(0.05);
  
  panAngle_ = 0;
  tiltAngle_ = 0;
  
  memset (&scanControl_, 0, sizeof (ScanControl));
  
  // This is only for debugging purposes, so can be commented-out
  if (DEBUG_)
    image_pub_ = nh_.advertise<sensor_msgs::Image> ("face_image", 30);
  
  std::string r_dt = nh_.resolveName (depth_topic_);
  std::string r_it = nh_.resolveName (image_topic_);
  ROS_INFO_STREAM("Listening for incoming data on topic " << r_dt );
  ROS_INFO_STREAM("Listening for incoming data on topic " << r_it );
  ROS_INFO_STREAM("Publishing image on topic " << "face_image" );
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "head_tracking_node");
  HeadTracking ht; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
