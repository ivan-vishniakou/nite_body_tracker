#!/usr/bin/env python

PACKAGE = 'nite_tracker_filter'
NODE = 'body_tracker'

import roslib
roslib.load_manifest(PACKAGE)
import rospy
import tf
import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped, PointStamped, \
                              PoseStamped, Point, Vector3, Quaternion
from std_msgs.msg import Empty, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from openni2_camera.msg import NiteJoint, NiteSkeleton, NitePeople


class BodyTrackerNode:
    
    def __init__(self):
        
        rospy.init_node(NODE)
        
        """
            Private variables
        """
        self._marker_colors = {}
        self._following = False
        self._following_id = -1
        self._detected_people = []
        self._track_history = []
        self._max_hist_poses = 100
        
        """
            Parameters
        """
        rospy.loginfo('Reading params')
        self._follow_distance = rospy.get_param('follow_distance', 1.0)
        self._goal_frame_id = rospy.get_param('goal_frame_id', 'base_link')
        self._goal_topic = rospy.get_param('nav_goal_topic', 'nav_goal')
        self._base_frame_id = rospy.get_param('base_frame_id', 'base_link')

        rospy.logwarn('Goal frame is: ' +self._goal_frame_id)
        
        """
            Services
        """
        #self._tracking_start_client_ = rospy.ServiceProxy('/cam3d/driver/people_tracking/start', Empty);
        #self._tracking_stop_client_ = rospy.ServiceProxy('/cam3d/driver/people_tracking/stop', Empty);
        rospy.loginfo('Waiting for people tracking service')
        
        """
            Topics
        """        
        self._tf= tf.TransformListener()
        self._tfbr = tf.TransformBroadcaster()
        self._tr = tf.TransformerROS(True, rospy.Duration(10.0))
        self._nite_people_subscriber = rospy.Subscriber('/cam3d/driver/people',
                                                        NitePeople,
                                                        self.people_callback,
                                                        queue_size=1)
        self._marker_publisher = rospy.Publisher('/nite_people', MarkerArray, queue_size=10)
        self._nav_goal_publisher = rospy.Publisher(self._goal_topic, PoseStamped, queue_size=10)
        self._following_start_subscriber = rospy.Subscriber('/people_following/start', Empty, self.following_start_cb)
        self._following_start_subscriber = rospy.Subscriber('/people_following/stop', Empty, self.following_stop_cb)        
        
        
    def following_start_cb(self, request):
        if not self._following:
            self._following_id = self.select_tracking_target()
            if self._following_id<0:
                rospy.logwarn('Nobody to follow')
                self._following = False
            else:
                self._following = True
                rospy.logwarn('Following user {}.'.format(self._following_id))
            pass
        pass
    
    
    def following_stop_cb(self, request):
        if self._following:
            self._following_id = -1
            self._following = False
            self._track_history = []
        pass
    
    
    def select_tracking_target(self):
        closest_person = -1
        closest_person_dist = 100000
        if len(self._detected_people)>0:
            for person in self._detected_people:
                dist = person.position.x**2 + person.position.y**2 + person.position.z**2
                print ['NONE','ERROR','CALIBRATIONG','TRACKED'][person.state]
                print 'dist', dist
                print person.position
                if dist > 0.1 and (person.state == NiteSkeleton.SKELETON_TRACKED or NiteSkeleton.SKELETON_CALIBRATING):
                    if dist < closest_person_dist:
                        closest_person = person.user_id
                        closest_person_dist = dist
                    pass
                pass
        return closest_person        
    
    
    def visualize(self, source_frame_id):
        
        
        pass
    
    
    def people_callback(self, msg):
        self._detected_people = msg.skeletons
        #print 'people_cb', msg.header, 'to' , self._goal_frame_id 
        lct = self._tf.getLatestCommonTime(msg.header.frame_id, self._goal_frame_id)
        if not self._tf.canTransform(self._goal_frame_id, msg.header.frame_id, lct):
            rospy.logwarn('Unable to lookup transform from {} to {} \r\n Dropping message.'.format(msg.header.frame_id, self._goal_frame_id))
            rospy.logwarn('msg: '+ str(msg.header.stamp))        
            rospy.logwarn('lct: '+ str(lct))
            rospy.logwarn('dif: '+str((msg.header.stamp.nsecs - lct.nsecs)/1000000) )
            return
        marker_array = MarkerArray()
        source_frame = msg.header.frame_id
        for person in msg.skeletons:
            '''
                Marker pos:
            '''
            marker = Marker()
            marker.header = msg.header
            marker.ns = "nite_people"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale = Vector3(0.3, 0.3, 0.3)
            marker.id = len(marker_array.markers)
            marker.text = 'uid' + str(person.user_id) + \
                (' TARGET' if self._following_id == person.user_id else '')
            
            tmp_point = PointStamped()
            tmp_point.point = Point(person.position.x, -person.position.y, person.position.z)
            tmp_point.header.stamp = lct
            tmp_point.header.frame_id = source_frame
            marker.header.frame_id = self._goal_frame_id
            self._tf.waitForTransform(source_frame, self._goal_frame_id, lct, rospy.Duration(0.1))
            marker.pose.position = self._tf.transformPoint(self._goal_frame_id, tmp_point).point
            
            '''
                keep story of the track:
            '''
            
            color = ColorRGBA()
            color.a = 0.5
            if person.state == NiteSkeleton.SKELETON_NONE:
                #rospy.loginfo('SKELETON NONE')
                color.r, color.g, color.b = 1, 1, 1
            elif person.state == NiteSkeleton.SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
                #rospy.loginfo('SKELETON_CALIBRATION_ERROR_NOT_IN_POSE')
                color.r, color.g, color.b = 1, 0, 0
            elif person.state == NiteSkeleton.SKELETON_CALIBRATING:
                #rospy.loginfo('SKELETON_CALIBRATING')
                color.r, color.g, color.b = 1, 1, 0
            elif person.state == NiteSkeleton.SKELETON_TRACKED:
                #rospy.loginfo('SKELETON_TRACKED')
                color.r, color.g, color.b = 0, 1, 0
            if self._following_id == person.user_id:                    
                color.a = 1.0
                        
            marker.color = color
            marker_array.markers.append(marker)

            for joint in person.joints:
                marker = Marker()
                marker.header = msg.header
                marker.ns = "nite_people"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.scale = Vector3(0.1, 0.1, 0.1)
                
                tmp_point = PointStamped()                
                tmp_point.point = Point(joint.position.x, -joint.position.y, joint.position.z)
                tmp_point.header.stamp = lct
                tmp_point.header.frame_id = source_frame
                marker.header.frame_id = self._goal_frame_id
                marker.pose.position = self._tf.transformPoint(self._goal_frame_id, tmp_point).point
                
                marker.color = color
                marker.id = len(marker_array.markers)
                marker_array.markers.append(marker)
            pass
            
            if self._following_id == person.user_id:                    
                '''
                    Get pose for target
                '''
                target_point = self.low_pass(marker.pose.position)
                lct = self._tf.getLatestCommonTime(self._goal_frame_id, self._base_frame_id)
                robot_point = PointStamped()
                robot_point.point = Point(0,0,0)
                robot_point.header.stamp = lct
                robot_point.header.frame_id = self._base_frame_id
                robot_point = self._tf.transformPoint(self._goal_frame_id, robot_point).point
                target_theta = np.math.atan2(target_point.y-robot_point.y, target_point.x-robot_point.x)
                q = tf.transformations.quaternion_from_euler(0, 0, target_theta )
                
                target_pose = PoseStamped()
                target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                target_pose.pose.position.x = target_point.x - self._follow_distance*np.math.cos(target_theta)
                target_pose.pose.position.y = target_point.y - self._follow_distance*np.math.sin(target_theta)
                target_pose.pose.position.z = 0
                
                target_pose.header = msg.header
                target_pose.header.frame_id = self._goal_frame_id
                
                
                self._nav_goal_publisher.publish(target_pose)
            pass
            
        for m in marker_array.markers:
            m.lifetime = rospy.Duration(0.2)
        self._marker_publisher.publish(marker_array)
    
    
    def low_pass(self, new_point):
        last_points = 7
        self._track_history.append(new_point)
        if len(self._track_history)>self._max_hist_poses:
            self._track_history.pop(0)
        avg = Point()
        for p in self._track_history[-last_points:]:
            avg.x += p.x
            avg.y += p.y
            avg.z += p.z
        avg.x/=last_points
        avg.y/=last_points
        avg.z/=last_points
        return avg
        
        

if __name__ == '__main__':
    rospy.loginfo('Trying to start BodyTrackerNode')
    w = BodyTrackerNode()

    rospy.spin()

