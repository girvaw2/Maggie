#!/usr/bin/env python

"""
    Relax all servos by disabling the torque for each.
"""
import roslib
roslib.load_manifest('maggie_head_movement')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetSpeed

class Relax():
    def __init__(self):
        rospy.init_node('relax_all_servos')
        
        dynamixel_namespace = rospy.get_namespace()
        if dynamixel_namespace == '/':
            dynamixel_namespace = rospy.get_param('~dynamixel_namespace', '/dynamixel_controller')
        dynamixels = rospy.get_param(dynamixel_namespace + '/dynamixels', dict())
        
        servo_torque_enable = list()
        servo_set_speed = list()
        
        for name in sorted(dynamixels):
            torque_enable_service = dynamixel_namespace + '/' + name + '_controller/torque_enable'
            rospy.wait_for_service(torque_enable_service)  
            servo_torque_enable.append(rospy.ServiceProxy(torque_enable_service, TorqueEnable))
            
            set_speed_service = dynamixel_namespace + '/' + name + '_controller/set_speed'
            rospy.wait_for_service(set_speed_service)
            servo_set_speed.append(rospy.ServiceProxy(set_speed_service, SetSpeed))

        # Give the servos an intial speed that is relatively slow for safety
        for set_speed in servo_set_speed:
            set_speed(0.3)
        
        # Relax all servos to give them a rest.
        for torque_enable in servo_torque_enable:
            torque_enable(False)
        
if __name__=='__main__':
    Relax()
