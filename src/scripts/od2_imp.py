#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

class odom_task():

    #Self
    def __init__(self):

        rospy.init_node('odometry_publisher') #node name
    

        #initializations
        self.current_time=rospy.Time.now()
        self.sample_rate=100.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0 #theta
        self.wz=0.0
        self.v=0.0
        self.vx=0.0
        self.vy=0.0
        self.dt=0.01

        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        #print(type(self.odom_quat))


        #Publishers
        self.odom_pub = rospy.Publisher("odom_custom", Odometry, queue_size=50)

        #Subscribers

        rospy.Subscriber("odom",Odometry,self.odom_callback)
        
        #Broadcasters
        self.odom_broadcaster = tf.TransformBroadcaster()


    #Callbacks


    def odom_callback(self,data):
        self.x=data.pose.pose.position.x
        self.y=data.pose.pose.position.y
        self.vx=data.twist.twist.linear.x
        self.vy=data.twist.twist.linear.y
        self.wz=data.twist.twist.angular.z
        self.odom_quat[0]=data.pose.pose.orientation.x
        self.odom_quat[1]=data.pose.pose.orientation.y
        self.odom_quat[2]=data.pose.pose.orientation.z
        self.odom_quat[3]=data.pose.pose.orientation.w


        


    #Broadcast data
    def broadcast(self):
        self.current_time=rospy.Time.now()
        #self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
        (self.x, self.y, 0.),
        self.odom_quat,
        self.current_time,
        "base_link",
        "odom"
    )
    #Publish data to topic
    def publish(self):
        odom = Odometry() 
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.wz))

        # publish the message
        self.odom_pub.publish(odom)




    
if __name__ == '__main__':
    odom=odom_task()
    r = rospy.Rate(odom.sample_rate)
    while not rospy.is_shutdown():
        try:
            odom.broadcast()
            odom.publish()
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
