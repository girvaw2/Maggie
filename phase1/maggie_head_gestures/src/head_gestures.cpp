#include "head_gestures.h"

HeadGestures::HeadGestures () 
{
  gesture_sub_ = nh_.subscribe("gesture", 1000, &HeadGestures::gestureCallback, this);
  joint_state_sub_ = nh_.subscribe ("joint_states", 1, &HeadGestures::jointCallback, this);
  
  target_pub_ = nh_.advertise<geometry_msgs::PointStamped> ("target_point", 1);
  head_tracking_pub_ = nh_.advertise<std_msgs::String> ("head_tracking", 1);
  
  pan_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/dynamixel_controller/head_pan_controller/set_speed");
  tilt_speed_client_ = nh_.serviceClient<dynamixel_controllers::SetSpeed>("/dynamixel_controller/head_tilt_controller/set_speed");
  dynamixel_controllers::SetSpeed speed_srv_;
}

void 
HeadGestures::startShake()
{
    setHeadPanSpeed(1);
    gesture_ = SHAKE;
    vTrajectory_.clear();
    
    targetPosition target;
    target.direction = PAN_RIGHT;
    target.angle = currPanAngle_ - 0.5; // ~30 degrees
    vTrajectory_.push_back(target);
    
    target.direction = PAN_LEFT;
    target.angle = currPanAngle_ + 0.5; // ~30 degrees
    vTrajectory_.push_back(target);

    target.direction = PAN_RIGHT;
    target.angle = currPanAngle_; // original position
    vTrajectory_.push_back(target);    
}

void
HeadGestures::processShake()
{
  if (vTrajectory_.size() > 0)
  {
    targetPosition tp = vTrajectory_.at(0);
    
    if (tp.direction == PAN_RIGHT)
    {
      if (DEBUG_)
	std::cout << "Pan Right: currPanAngle_ = " << currPanAngle_ << " tp.angle = " << tp.angle << " x = " << 10 * cos(tp.angle) << " y = " << 10 * sin(tp.angle) << std::endl;
      
      if (currPanAngle_ > (tp.angle + ANGLE_ERROR_MARGIN))
      {
	setHeadPosition("base_link", 10 * cos(tp.angle), 10 * sin(tp.angle), 0);
      }
      else
      {
	vTrajectory_.erase(vTrajectory_.begin());
      }
    }
    else
    if (tp.direction == PAN_LEFT)
    {
      if (DEBUG_)
	std::cout << "Pan Left: currPanAngle_ = " << currPanAngle_ << " tp.angle = " << tp.angle << " x = " << 10 * cos(tp.angle) << " y = " << 10 * sin(tp.angle) << std::endl;
      
      if (currPanAngle_ < tp.angle - ANGLE_ERROR_MARGIN)
      {
	setHeadPosition("base_link", 10 * cos(tp.angle), 10 * sin(tp.angle), 0);
      }
      else
      {
	vTrajectory_.erase(vTrajectory_.begin());
      }
    }
  }
}

void 
HeadGestures::startNod()
{
    setTiltSpeed(0.5);
    gesture_ = NOD;
    vTrajectory_.clear();
    
    targetPosition target;
    target.direction = TILT_UP;
    target.angle = currTiltAngle_ + 0.25; // ~30 degrees
    vTrajectory_.push_back(target);
    
    target.direction = TILT_DOWN;
    target.angle = currTiltAngle_ - 0.25; // ~30 degrees
    vTrajectory_.push_back(target);

    target.direction = TILT_UP;
    target.angle = currTiltAngle_; // original position
    vTrajectory_.push_back(target); 
}

void
HeadGestures::processNod()
{
  if (vTrajectory_.size() > 0)
  {
    targetPosition tp = vTrajectory_.at(0);
    
    if (tp.direction == TILT_UP)
    {
      if (DEBUG_)
	std::cout << "Tilt Up: currTiltAngle_ = " << currTiltAngle_ << " tp.angle = " << tp.angle << " x = " << 10 * cos(tp.angle) << " z = " << 10 * sin(tp.angle) << std::endl;
      
      if (currTiltAngle_ < (tp.angle - ANGLE_ERROR_MARGIN))
      {
	setHeadPosition("base_link", 10 * cos(tp.angle), 10 * sin(currPanAngle_), 10 * sin(tp.angle));
      }
      else
      {
	vTrajectory_.erase(vTrajectory_.begin());
      }
    }
    else
    if (tp.direction == TILT_DOWN)
    {
      if (DEBUG_)
	std::cout << "Tilt Down: currPanAngle_ = " << currPanAngle_ << " tp.angle = " << tp.angle << " x = " << 10 * cos(tp.angle) << " z = " << 10 * sin(tp.angle) << std::endl;
      
      if (currTiltAngle_ > tp.angle + ANGLE_ERROR_MARGIN)
      {
	setHeadPosition("base_link", 10 * cos(tp.angle), 10 * sin(currPanAngle_), 10 * sin(tp.angle));
      }
      else
      {
	vTrajectory_.erase(vTrajectory_.begin());
      }
    }
  }
}

void
HeadGestures::setHeadPosition(string frame, float x, float y, float z)
{
    geometry_msgs::PointStamped point_out;          
    point_out.header.frame_id = frame; 
    point_out.point.x = x; 
    point_out.point.y = y; 
    point_out.point.z = z; 
    
    target_pub_.publish<geometry_msgs::PointStamped>(point_out); 
}

void
HeadGestures::setHeadPanSpeed (float speed)
{
  speed_srv_.request.speed = speed;
  if (!pan_speed_client_.call(speed_srv_))
  {
    std::cout << "Failed to set pan speed: " << speed << std::endl;
  }
}

void
HeadGestures::setTiltSpeed (float speed)
{
  speed_srv_.request.speed = speed;
  if (!tilt_speed_client_.call(speed_srv_))
  {
    std::cout << "Failed to set tilt speed: " << speed << std::endl;
  }
}

void 
HeadGestures::gestureCallback(const std_msgs::String::ConstPtr& msg)
{
  string str = msg->data;
  boost::algorithm::to_lower(str);
  if (str == "shake")
  {
    startShake();
  }
  else if (str == "nod")
  {
    startNod();
  }
  else if (str == "start_tracking")
  {
    std_msgs::String msg;
    msg.data = "start";
    head_tracking_pub_.publish<std_msgs::String>(msg); 
  }
  else if (str == "stop_tracking")
  {
    std_msgs::String msg;
    msg.data = "stop";
    head_tracking_pub_.publish<std_msgs::String>(msg); 
  }    
}
 
void
HeadGestures::jointCallback (const sensor_msgs::JointState& state)
{
  currPanAngle_ = state.position.at(0);
  currTiltAngle_ = state.position.at(1);
  
  if (gesture_ == SHAKE)
    processShake();
  else
  if (gesture_ == NOD)
    processNod();
}

int
main (int argc, char **argv)
{
  ros::init (argc, argv, "head_gestures_node");
  HeadGestures hg; //this loads up the node
  ros::spin (); //where she stops nobody knows
  return 0;
}
