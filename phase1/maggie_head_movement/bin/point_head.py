#! /usr/bin/env python

# Copyright (c) 2010, Arizona Robotics Research Group, University of Arizona
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Anh Tran

# Based on the wubble_head_action.py script by Anh Tran. Modifications made by Patrick Goebel
# for the Pi Robot Project.

import roslib; roslib.load_manifest('maggie_head_movement')
import rospy
import tf
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

import math

class PointHeadNode():

    def __init__(self):
        # Initialize new node
        rospy.init_node('point_head_node', anonymous=True)
        
        dynamixel_namespace = rospy.get_namespace()
        rate = rospy.get_param('~rate', 10)
        r = rospy.Rate(rate)
        
        #---------------------------------
        
	self.camera_link = 'camera_link'
        self.head_pan_joint = 'head_pan_joint'
        self.head_tilt_joint = 'head_tilt_joint'
        self.head_pan_link = 'head_pan_link'
        self.head_tilt_link = 'head_tilt_link'
        
	self.dynamixels = rospy.get_param('dynamixels', '')
        
        """ The pan/tilt thresholds indicate how far (in meters) the ROI needs to be off-center
            before we make a movement. """
        self.pan_threshold = int(rospy.get_param('~pan_threshold', 0.01))
        self.tilt_threshold = int(rospy.get_param('~tilt_threshold', 0.01))
        
        """ The k_pan and k_tilt parameter determine how responsive the servo movements are.
            If these are set too high, oscillation can result. """
        self.k_pan = rospy.get_param('~k_pan', 1.5)
        self.k_tilt = rospy.get_param('~k_tilt', 1.5)
        
        """ Set limits on how far we can pan or tilt """
        self.max_pan = rospy.get_param('~max_pan', 3)
        self.min_pan = rospy.get_param('~min_pan', -3)
        self.max_tilt = rospy.get_param('~max_tilt', 1.7)
        self.min_tilt = rospy.get_param('~min_tilt', -1.7)
        
        self.servo_speed = dict()
        self.servo_position = dict()
        self.torque_enable = dict()
        
        #---------------------------------
        
        # Initialize the target point
        self.target_point = PointStamped()
        self.last_target_point = PointStamped()
        
        # Subscribe to the target_point topic
        rospy.Subscriber('/target_point', PointStamped, self.update_target_point)

        # Initialize publisher for the pan servo
        self.head_pan_frame = 'head_pan_link'
        self.head_pan_pub = rospy.Publisher(dynamixel_namespace + 'head_pan_controller/command', Float64)
        
        # Initialize publisher for the tilt servo
        self.head_tilt_frame = 'head_tilt_link'
        self.head_tilt_pub = rospy.Publisher(dynamixel_namespace + 'head_tilt_controller/command', Float64)

        # Initialize tf listener
        self.tf = tf.TransformListener()
        
        # Make sure we can see at least the pan and tilt frames
        self.tf.waitForTransform(self.head_pan_frame, self.head_tilt_frame, rospy.Time(), rospy.Duration(5.0))
            
        # Reset the head position to neutral
        rospy.sleep(1)
        self.center_head()
        
        rospy.loginfo("Ready to accept target point")
        
        while not rospy.is_shutdown():
            rospy.wait_for_message('/target_point', PointStamped)
            if self.target_point == self.last_target_point:
                continue
            try:
                target_angles = self.transform_target_point(self.target_point)
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.logerr("tf Failure")
                continue
                
            self.head_pan_pub.publish(target_angles[0])
            self.head_tilt_pub.publish(target_angles[1])
            
            self.last_target_point = self.target_point
            rospy.loginfo("Setting Target Point:\n" + str(self.target_point))
            
            r.sleep()
        
    def update_target_point(self, msg):
        self.target_point = msg
        
    def update_head_position(self, target):
        """ When OpenCV loses the ROI, the message stops updating.  Use this counter to
            determine when it stops. """
        self.tracking_seq += 1
        
        """ Project the target point onto the camera link.
            In case of tf exceptions, simply return without an update. """
        try:         
            self.tf.waitForTransform(self.camera_link, target.header.frame_id, rospy.Time.now(), rospy.Duration(1.0))
            camera_target = self.tf.transformPoint(self.camera_link, target)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            return
        
        """ The virtual camera image is in the y-z plane """
        pan = -camera_target.point.y
        tilt = -camera_target.point.z
        
        """ Compute the distance to the target in the x direction """
        distance = float(abs(camera_target.point.x))
        
        """ Convert the pan and tilt values from meters to radians by dividing by the distance to the target.  Since the Kinect is 
            blind to distance within 0.5 meters, check for an exception and use 0.5 meters as a fall back. """
        try:
            pan /= distance
            tilt /= distance
        except:
            pan /= 0.5
            tilt /= 0.5
                      
        """ Pan the camera only if the displacement of the target point exceeds the threshold """
        if abs(pan) > self.pan_threshold:
            """ Set the pan speed proportion to the horizontal displacement of the target """
            self.pan_speed = trunc(min(self.max_joint_speed, max(ZERO_SPEED, self.k_pan * abs(pan))), 2)
               
            """ Set the target position ahead or behind the current position """
            try:
                current_pan = self.joint_state.position[self.joint_state.name.index(self.head_pan_joint)]
            except:
                return
            if pan > 0:
                self.pan_position = max(self.min_pan, current_pan - self.lead_target_angle)
            else:
                self.pan_position = min(self.max_pan, current_pan + self.lead_target_angle)
        else:
            self.pan_speed = ZERO_SPEED
        
        """ Tilt the camera only if the displacement of the target point exceeds the threshold """
        if abs(tilt) > self.tilt_threshold:
            """ Set the pan speed proportion to the vertical displacement of the target """
            self.tilt_speed = trunc(min(self.max_joint_speed, max(ZERO_SPEED, self.k_tilt * abs(tilt))), 2)
            
            """ Set the target position ahead or behind the current position """
            try:
                current_tilt = self.joint_state.position[self.joint_state.name.index(self.head_tilt_joint)]
            except:
                return
            if tilt < 0:
                self.tilt_position = max(self.min_tilt, current_tilt - self.lead_target_angle)
            else:
                self.tilt_position = min(self.max_tilt, current_tilt + self.lead_target_angle)

        else:
            self.tilt_speed = ZERO_SPEED    
        

    def center_head(self):
        self.head_pan_pub.publish(0.0)
        self.head_tilt_pub.publish(0.0)
        rospy.sleep(3)

    def transform_target_point(self, target):
        # Set the pan and tilt reference frames to the head_pan_frame and head_tilt_frame defined above
        pan_ref_frame = self.head_pan_frame
        tilt_ref_frame = self.head_tilt_frame
        
        # Wait for tf info (time-out in 5 seconds)
        self.tf.waitForTransform(pan_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))
        self.tf.waitForTransform(tilt_ref_frame, target.header.frame_id, rospy.Time(), rospy.Duration(5.0))

        # Transform target point to pan reference frame & retrieve the pan angle
        pan_target = self.tf.transformPoint(pan_ref_frame, target)
        pan_angle = math.atan2(pan_target.point.y, pan_target.point.x)

        # Transform target point to tilt reference frame & retrieve the tilt angle
        tilt_target = self.tf.transformPoint(tilt_ref_frame, target)
        tilt_angle = math.atan2(tilt_target.point.z,
                math.sqrt(math.pow(tilt_target.point.x, 2) + math.pow(tilt_target.point.y, 2)))

        return [pan_angle, tilt_angle]

if __name__ == '__main__':
    try:
        point_head = PointHeadNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

