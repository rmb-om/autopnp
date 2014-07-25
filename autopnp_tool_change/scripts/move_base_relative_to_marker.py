#! /usr/bin/env python
#################################################################
##\file
## \note
#   Copyright (c) 2014 \n
#   Fraunhofer Institute for Manufacturing Engineering
#   and Automation (IPA) \n\n
#
#################################################################
#
# \note
#   Project name: care-o-bot
# \note
#   ROS stack name: autopnp
# \note
#   ROS package name: autopnp_tool_change
#
# \author: Richard Bormann(email:richard.bormann@ipa.fraunhofer.de)
#
# \author
# Supervised by:  
# 
# \date Date of creation: May 2014
#
# \brief
# SMACH representation of movement script
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     - Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer. \n
#     - Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution. \n
#     - Neither the name of the Fraunhofer Institute for Manufacturing
#       Engineering and Automation (IPA) nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
# 
# You should have received a copy of the GNU Lesser General Public 
# License LGPL along with this program. 
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################
import sys, os
import math, numpy
import threading

import roslib; roslib.load_manifest('autopnp_tool_change')
import rospy
import smach
import smach_ros
import tf

from tf.transformations import *

from cob_object_detection_msgs.msg import DetectionArray, Detection
from geometry_msgs import *
from geometry_msgs.msg import *

from simple_script_server import simple_script_server
sss = simple_script_server()

#from exploration_detection_cleaning import *

global TOOL_WAGON_MARKER_OFFSETS
TOOL_WAGON_MARKER_OFFSETS = {  # todo: measure translational offsets
						"front":	Transform(translation=Vector3(x=0.055, y=-0.975, z=0.0), rotation=Quaternion(x=-0.5, y=-0.5, z=-0.5, w=0.5)),  # quaternion_from_euler(-90.0/180.0*math.pi, -90.0/180.0*math.pi, 0.0, 'rzyx')
						"rear":		Transform(translation=Vector3(x=0.06, y=-1.025, z=-0.46), rotation=Quaternion(x=-0.5, y=0.5, z=0.5, w=0.5)),  # quaternion_from_euler(90.0/180.0*math.pi, 90.0/180.0*math.pi, 0.0, 'rzyx')
						"left":		Transform(translation=Vector3(x=-0.17, y=-1.0, z=-0.275), rotation=Quaternion(x=0.0, y=0.707106781, z=0.707106781, w=0.0)),  # quaternion_from_euler(math.pi, 0.0, 90.0/180.0*math.pi, 'rzyx')
						"right":	Transform(translation=Vector3(x=0.29, y=-1.0, z=-0.275), rotation=Quaternion(x=-0.70710678, y=0.0, z=0.0, w=0.70710678))  # quaternion_from_euler(0.0, 0.0, -90.0/180.0*math.pi, 'rzyx')
						}  # offset transformations from respective markers to tool wagon base/center coordinate system

global TOOL_WAGON_ROBOT_OFFSETS
TOOL_WAGON_ROBOT_OFFSETS = {
						"front":	Pose2D(x=-1.1, y=0.0, theta=0.0),
						"front_far":	Pose2D(x=-1.4, y=0.0, theta=0.0),
						"rear":		Pose2D(x=-1.45, y=0.0, theta=math.pi),
						"tool_change_1":		Pose2D(x=-1.15, y=0.05, theta=math.pi+0.05), # left bay
						"tool_change_2":		Pose2D(x=-1.15, y=0.05, theta=math.pi-0.135), # center bay
						"front_frontal_far":	Pose2D(x=1.4, y=0.0, theta=math.pi),
						"front_trash_clearing":	Pose2D(x=-0.95, y=0.0, theta=0.0)  # Pose2D(x=-1.05, y=0.0, theta=0.0)
						}  # describes the offset of the tool wagon center with respect to base_link (x-axis in tool wagon is directed to the front, y-axis to the left)

global FIDUCIALS_MARKER_DICTIONARY
FIDUCIALS_MARKER_DICTIONARY = {
						"tag_tool_wagon_front_l":		"tag_48",  # left = +y of tool wagon base frame
						"tag_tool_wagon_front_c":		"tag_55",
						"tag_tool_wagon_front_r":		"tag_36",  # right = -y
						"tag_tool_wagon_rear_l":		"tag_38",  # left = +y
						"tag_tool_wagon_rear_c":		"tag_79",
						"tag_tool_wagon_rear_r":		"tag_73",  # right = -y
						"tag_tool_wagon_right":			"tag_64",
						"tag_tool_wagon_left":			"tag_69",
						"trash_bin":					"tag_25"
						}


class ScreenFormat:
    # Convenient functions, which should be called when entering and leaving a state
    # to improve readability of the debug output on the console
    
    def __init__(self, text):
        self.text = text
        self.log_enter_state(text)
        
    def __del__(self):
        self.log_exit_state(self.text)
    
    def log_enter_state(self, text):
    
        #HEADER = '\033[95m'
        color = '\033[94m'
        #OKGREEN = '\033[92m'
        #WARNING = '\033[93m'
        #FAIL = '\033[91m'
        #ENDC = '\033[0m'
    
        out_str = ''
        for num in range(0, len(text)):
            out_str += '-'
    
        rospy.loginfo('%s\n', color)
        rospy.loginfo("%s----------------------------------------------", out_str)
        rospy.loginfo("- ENTERING \"%s\" --------------------------------\033[0m", text)
    
    
    def log_exit_state(self, text):
        color = '\033[94m'
        out_str = ''
        for num in range(0, len(text)):
            out_str += '-'
        rospy.loginfo('%s', color)
        rospy.loginfo("- LEAVING \"%s\" ---------------------------------", text)
        rospy.loginfo("%s----------------------------------------------\n\033[0m", out_str)


###############''WORKAROUND FOR TRANSFORMLISTENER ISSUE####################
_tl=None
_tl_creation_lock=threading.Lock()

def get_transform_listener():
	global _tl
	with _tl_creation_lock:
		if _tl==None:
			_tl=tf.TransformListener(True, rospy.Duration(40.0))
		return _tl
#################################################################################


def computeToolWagonPoseFromFiducials(fiducials):
	trROS = tf.TransformerROS(True, rospy.Duration(10.0))
	averaged_tool_wagon_pose = None
	averaged_tool_wagon_markers = 0.0
	for fiducial in fiducials.detections:
		# transform to base pose of tool wagon and then convert to map coordinate system
		offset = None
		if fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_l"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_c"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_front_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['front']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_l"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_c"] or fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_rear_r"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['rear']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_left"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['left']
		elif fiducial.label == FIDUCIALS_MARKER_DICTIONARY["tag_tool_wagon_right"]:
			offset = TOOL_WAGON_MARKER_OFFSETS['right']
		
		if offset != None:  # i.e. if a tool wagon marker was detected
			# compute tool wagon center
			offset_mat = trROS.fromTranslationRotation((offset.translation.x, offset.translation.y, offset.translation.z), (offset.rotation.x, offset.rotation.y, offset.rotation.z, offset.rotation.w))
			fiducial_pose_mat = trROS.fromTranslationRotation((fiducial.pose.pose.position.x, fiducial.pose.pose.position.y, fiducial.pose.pose.position.z), (fiducial.pose.pose.orientation.x, fiducial.pose.pose.orientation.y, fiducial.pose.pose.orientation.z, fiducial.pose.pose.orientation.w))
			tool_wagon_pose_mat = numpy.dot(fiducial_pose_mat, offset_mat)
			q = quaternion_from_matrix(tool_wagon_pose_mat)
			tool_wagon_pose = PoseStamped(header=fiducial.pose.header, pose=Pose(position=Point(x=tool_wagon_pose_mat[0, 3], y=tool_wagon_pose_mat[1, 3], z=tool_wagon_pose_mat[2, 3]),
									orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
			# print "fidpose, offset: ", fiducial.pose, offset
			# print "twpose: ", tool_wagon_pose
			# print "twpose ypr: ", euler_from_quaternion([tool_wagon_pose.pose.orientation.x, tool_wagon_pose.pose.orientation.y, tool_wagon_pose.pose.orientation.z, tool_wagon_pose.pose.orientation.w], "rzyx")
			
			# transform to map system
			try:
				listener = get_transform_listener()
				listener.waitForTransform('/map', tool_wagon_pose.header.frame_id, tool_wagon_pose.header.stamp, rospy.Duration(2))
				tool_wagon_pose_map = listener.transformPose('/map', tool_wagon_pose)
				# print 'tool_wagon_pose = ', tool_wagon_pose_map
				# print "tool_wagon_pose ypr: ", euler_from_quaternion([tool_wagon_pose_map.pose.orientation.x, tool_wagon_pose_map.pose.orientation.y, tool_wagon_pose_map.pose.orientation.z, tool_wagon_pose_map.pose.orientation.w], "rzyx")
				# average position when multiple detections are present
				if averaged_tool_wagon_pose == None:
					averaged_tool_wagon_pose = tool_wagon_pose_map
					averaged_tool_wagon_markers = 1.0
				else:
					averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x + tool_wagon_pose_map.pose.position.x
					averaged_tool_wagon_pose.pose.position.y = averaged_tool_wagon_pose.pose.position.y + tool_wagon_pose_map.pose.position.y
					averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z + tool_wagon_pose_map.pose.position.z
					averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x + tool_wagon_pose_map.pose.orientation.x
					averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y + tool_wagon_pose_map.pose.orientation.y
					averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z + tool_wagon_pose_map.pose.orientation.z
					averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w + tool_wagon_pose_map.pose.orientation.w
					averaged_tool_wagon_markers = averaged_tool_wagon_markers + 1.0
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
				print "computeToolWagonPoseFromFiducials: Could not lookup robot pose: %s" % e
				return averaged_tool_wagon_pose
	# finalize averaging and normalizationroslib.load_manifest(PACKAGE)
	if averaged_tool_wagon_markers > 0.0:
		averaged_tool_wagon_pose.pose.position.x = averaged_tool_wagon_pose.pose.position.x / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.y = averaged_tool_wagon_pose.pose.position.y / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.position.z = averaged_tool_wagon_pose.pose.position.z / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z / averaged_tool_wagon_markers
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w / averaged_tool_wagon_markers
		norm = math.sqrt(averaged_tool_wagon_pose.pose.orientation.x * averaged_tool_wagon_pose.pose.orientation.x + averaged_tool_wagon_pose.pose.orientation.y * averaged_tool_wagon_pose.pose.orientation.y + averaged_tool_wagon_pose.pose.orientation.z * averaged_tool_wagon_pose.pose.orientation.z + averaged_tool_wagon_pose.pose.orientation.w * averaged_tool_wagon_pose.pose.orientation.w)
		averaged_tool_wagon_pose.pose.orientation.x = averaged_tool_wagon_pose.pose.orientation.x / norm
		averaged_tool_wagon_pose.pose.orientation.y = averaged_tool_wagon_pose.pose.orientation.y / norm
		averaged_tool_wagon_pose.pose.orientation.z = averaged_tool_wagon_pose.pose.orientation.z / norm
		averaged_tool_wagon_pose.pose.orientation.w = averaged_tool_wagon_pose.pose.orientation.w / norm
		
		return averaged_tool_wagon_pose

def currentRobotPose():
	# read out current robot pose
	try:
		listener = get_transform_listener()
		t = rospy.Time(0)
		listener.waitForTransform('/map', '/base_link', t, rospy.Duration(10))
		(robot_pose_translation, robot_pose_rotation) = listener.lookupTransform('/map', '/base_link', t)
	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
		print "Could not lookup robot pose: %s" % e
		return (None, None, None)
	robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx')  # yields yaw, pitch, roll
	
	return (robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler)

def positionControlLoopLinear(self_tool_wagon_pose, dx, dy, dtheta):
	# move to corrected pose
	tool_wagon_pose_3d = self_tool_wagon_pose.pose
	tool_wagon_pose_3d_euler = tf.transformations.euler_from_quaternion([tool_wagon_pose_3d.orientation.x, tool_wagon_pose_3d.orientation.y, tool_wagon_pose_3d.orientation.z, tool_wagon_pose_3d.orientation.w], 'rzyx')  # yields yaw, pitch, roll
	print "wagon theta: ", tool_wagon_pose_3d_euler[0], dtheta
	tool_wagon_pose = Pose2D(x=tool_wagon_pose_3d.position.x, y=tool_wagon_pose_3d.position.y, theta=tool_wagon_pose_3d_euler[0])
	robot_goal_pose = Pose2D(x=tool_wagon_pose.x + dx * math.cos(tool_wagon_pose.theta) - dy * math.sin(tool_wagon_pose.theta),
							y=tool_wagon_pose.y + dx * math.sin(tool_wagon_pose.theta) + dy * math.cos(tool_wagon_pose.theta),
							theta=tool_wagon_pose.theta - dtheta)
	print "moving to ", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)]
	handle_base = sss.move("base", [float(robot_goal_pose.x), float(robot_goal_pose.y), float(robot_goal_pose.theta)], mode='linear')
	
	# read out current robot pose
	(robot_pose_translation, robot_pose_rotation, robot_pose_rotation_euler) = currentRobotPose()
	if (robot_pose_translation == None):
		return False
	
	# verify distance to goal pose
	dist = math.sqrt((robot_pose_translation[0] - robot_goal_pose.x) * (robot_pose_translation[0] - robot_goal_pose.x) + (robot_pose_translation[1] - robot_goal_pose.y) * (robot_pose_translation[1] - robot_goal_pose.y))
	delta_theta = robot_pose_rotation_euler[0] - robot_goal_pose.theta
	print "delta_theta first:", delta_theta
	while delta_theta > 1.5 * math.pi:
		delta_theta = delta_theta - 2 * math.pi
	while delta_theta < -1.5 * math.pi:
		delta_theta = delta_theta + 2 * math.pi
	print "(x,y)-dist: ", dist, "  yaw-dist: ", delta_theta
	if dist > 0.04 or abs(delta_theta) > 0.03:  # in m      # rot: 0.02 #trans: 0.03  # josh
		return False
	else:
		return True


class MoveToToolWaggonToolChange(smach.State):
	def __init__(self, tool_bay_number):
		smach.State.__init__(self, outcomes=['arrived', 'failed'], input_keys=['tool_wagon_pose'])
		self.tool_wagon_pose = None
		self.tool_bay_number = tool_bay_number

	def fiducial_callback(self, fiducials):
		
		self.tool_wagon_pose = computeToolWagonPoseFromFiducials(fiducials)

	def execute(self, userdata):
		sf = ScreenFormat(self.__class__.__name__)

		# arm
		sss.move("arm", [[0.8531169383748283, -1.350256522512893, 0.69561842667486, -1.80983662114804, 1.3313022468362348, -1.4329851290574243, 0.6748141019910875]])
		
		sss.move("head", "back", False)
		sss.move("torso", [[-0.000181745039299, -0.3349796272814051, -0.0577915536147, -0.026197249069814434]], False)

		self.tool_wagon_pose = None
		fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
		
		offset_name = "tool_change_" + str(self.tool_bay_number)
		print "moving to ", offset_name
		
		userdata_tool_wagon_pose = userdata.tool_wagon_pose
		while userdata_tool_wagon_pose == None:
			if self.tool_wagon_pose != None:
				tool_wagon_pose_3d = self.tool_wagon_pose.pose
				tool_wagon_pose_3d_euler = tf.transformations.euler_from_quaternion([tool_wagon_pose_3d.orientation.x, tool_wagon_pose_3d.orientation.y, tool_wagon_pose_3d.orientation.z, tool_wagon_pose_3d.orientation.w], 'rzyx')  # yields yaw, pitch, roll
				userdata_tool_wagon_pose = Pose2D(x=tool_wagon_pose_3d.position.x, y=tool_wagon_pose_3d.position.y, theta=tool_wagon_pose_3d_euler[0])
		
		# 1. move to last known position (i.e. move_base to tool_wagon_pose + robot offset)
		robot_offset = Pose2D(x=TOOL_WAGON_ROBOT_OFFSETS[offset_name].x - 0.2, y=TOOL_WAGON_ROBOT_OFFSETS[offset_name].y, theta=TOOL_WAGON_ROBOT_OFFSETS[offset_name].theta)
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		robot_pose = Pose2D(x=userdata_tool_wagon_pose.x + dx * math.cos(userdata_tool_wagon_pose.theta) - dy * math.sin(userdata_tool_wagon_pose.theta),
							y=userdata_tool_wagon_pose.y + dx * math.sin(userdata_tool_wagon_pose.theta) + dy * math.cos(userdata_tool_wagon_pose.theta),
							theta=userdata_tool_wagon_pose.theta - robot_offset.theta)
		handle_base = sss.move("base", [robot_pose.x, robot_pose.y, robot_pose.theta], mode='omni')  # hack
		
		# 2. detect fiducials and move to corrected pose
		robot_offset = TOOL_WAGON_ROBOT_OFFSETS[offset_name]
		dx = -(robot_offset.x * math.cos(robot_offset.theta) + robot_offset.y * math.sin(robot_offset.theta))
		dy = -(-robot_offset.x * math.sin(robot_offset.theta) + robot_offset.y * math.cos(robot_offset.theta))
		# wait until wagon is detected
		robot_approximately_well_positioned = False
		while robot_approximately_well_positioned == False:
			while self.tool_wagon_pose == None:
				rospy.sleep(0.2)
			robot_approximately_well_positioned = positionControlLoopLinear(self.tool_wagon_pose, dx, dy, robot_offset.theta)
			if robot_approximately_well_positioned == False:
				self.tool_wagon_pose = None
		
		# 3. unsubscribe to fiducials
		fiducials_sub.unregister()
		#sss.move("torso", "home")
		
		# arm
		sss.move("arm", [[0.8278445708059503, -0.8787034652090652, 1.1638728116924186, -1.9798491435848076, 1.8234850958986357, -1.7385573744965914, 0.808576135863933]])

		return 'arrived'




def main(p_tool_bay_number):
	rospy.init_node('move_base_relative_to_marker')
	
	# arm:
	# lower, during approach [0.8531169383748283, -1.350256522512893, 0.69561842667486, -1.80983662114804, 1.3313022468362348, -1.4329851290574243, 0.6748141019910875]
	# upper, before putting away [0.8278445708059503, -0.8787034652090652, 1.1638728116924186, -1.9798491435848076, 1.8234850958986357, -1.7385573744965914, 0.808576135863933]
	
	
	# init listener
	dummy_listener = get_transform_listener()
	
	# move to position scenario
	sm_scenario = smach.StateMachine(outcomes=['finished', 'failed'])
	sm_scenario.userdata.tool_wagon_pose = None

	with sm_scenario:

		smach.StateMachine.add('MOVE_TO_TOOL_WAGGON_TOOL_CHANGE_2', MoveToToolWaggonToolChange(tool_bay_number = p_tool_bay_number),
								transitions={'arrived':'finished', 'failed':'failed'})
		
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm_scenario, '/START')
	sis.start()
	
	# Execute SMACH plan
	outcome = sm_scenario.execute()
	
	#rospy.spin()
	
	sis.stop()


if __name__ == '__main__':
	try:
		tool_bay_number = 1
		if len(sys.argv)>=2:
			tool_bay_number = sys.argv[1]

		main(tool_bay_number)
	except:
		print('EXCEPTION THROWN')
		print('Aborting cleanly')
		os._exit(1)













# #!/usr/bin/python
# 
# import sys, os
# 
# import roslib; roslib.load_manifest('autopnp_tool_change') # ; roslib.load_manifest('cob_navigation_global')
# import rospy
# import tf
# 
# from geometry_msgs.msg import PoseStamped
# from cob_object_detection_msgs.msg import DetectionArray, Detection
# 
# from simple_script_server import simple_script_server
# sss = simple_script_server()
# 
# class move_base_relative_to_marker():
# 	def __init__(self):
# 		self.last_fiducial_pose = PoseStamped()
# 
# 		# subscribe to fiducials topic
# 		self.fiducials_sub = rospy.Subscriber("/fiducials/detect_fiducials", DetectionArray, self.fiducial_callback)
# 
# 	def fiducial_callback(self, fiducials):
# 		for fiducial in fiducials.detections:
# 			if fiducial.label == "tag_73":
# 				self.last_fiducial_pose = fiducial.pose
# 				
# 	def execute(self):
# 		# move torso to right position
# 		# sss.move("torso","back_extreme")
# 
# 		while True:
# 			#robot_pose_rotation_euler = tf.transformations.euler_from_quaternion(robot_pose_rotation, 'rzyx') # yields yaw, pitch, roll
# 			print self.last_fiducial_pose
# 			
# if __name__ == '__main__':
# 	try:
# 		rospy.init_node('move_base_relative_to_marker')
# 	
# 		mbrtm = move_base_relative_to_marker()
# 		mbrtm.execute()
# 		
# 	except:
# 		print('EXCEPTION THROWN')
# 		print('Aborting cleanly')
# 		os._exit(1)



# /base_link   /fiducial/tag_board (tag_73)
# - Translation: [-0.688, -0.011, 1.002]
# - Rotation: in Quaternion [0.479, 0.520, 0.509, 0.491]
#            in RPY [1.571, 0.024, 1.629]