#include <ros/ros.h>
#include <ros/console.h>
#include "std_msgs/String.h"
#include <sensor_msgs/JointState.h>
#include <boost/algorithm/string.hpp>
#include "geometry_msgs/PointStamped.h"
#include "dynamixel_controllers/SetSpeed.h"

#include <math.h>
#include <vector>

#define PAN_LEFT		0
#define PAN_RIGHT		1
#define TILT_UP			0
#define TILT_DOWN		1

#define ANGLE_ERROR_MARGIN	0.1

#define DEBUG_			0	

using namespace std;

typedef struct
{
  float angle;
  int direction;
  bool pan;
} targetPosition;

enum gesture
{
  SHAKE,
  NOD
};

class HeadGestures
{
public:
  HeadGestures () ;
  
private:
  void testTimerCallback(const ros::TimerEvent& event);
  void gestureCallback(const std_msgs::String::ConstPtr& msg);
  void jointCallback (const sensor_msgs::JointState& state);
  
  void startShake();
  void processShake();
  
  void startNod();
  void processNod();
  
  void setHeadPosition(string frame, float x, float y, float z);
  void setHeadPanSpeed (float speed);
  void setTiltSpeed (float speed);
  
  void DoTest();
  
private:
    ros::NodeHandle nh_;
    ros::Publisher target_pub_;
    ros::Publisher head_tracking_pub_;
    ros::Subscriber gesture_sub_;
    ros::Subscriber joint_state_sub_;
    
    dynamixel_controllers::SetSpeed speed_srv_;
    ros::ServiceClient pan_speed_client_;
    ros::ServiceClient tilt_speed_client_;
    
    float currPanAngle_;
    float currTiltAngle_;
    vector<targetPosition> vTrajectory_;
    
    int gesture_;
};

