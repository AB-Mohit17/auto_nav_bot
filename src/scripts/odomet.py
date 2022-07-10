#!/usr/bin/env python

import math
from math import sin, cos, pi
import rospy
import tf 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import *

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
        
        #the angular position of an object can also be determined using 4 numbers called 'quarternions'
        #Ros uses quaternions for transformations so we initialize an odom_quat variable.
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)


        #Publishers
        self.odom_pub = rospy.Publisher("odom_custom", Odometry, queue_size=50)

        #Subscribers
        rospy.Subscriber("/cmd_vel",Twist, self.vel_callback) #find out the message type. Also, import it like we did for Odometry. Youll realize this one is already imported.
        rospy.Subscriber("/imu",Imu,self.angle_callback) ##find out the message type. Also, import it like we did for Odometry. Youll realize this one is NOT already imported.
        
        
        #Broadcasters
        self.odom_broadcaster = tf.TransformBroadcaster()


    #Callbacks

    def vel_callback(self,data): #cmd_vel callback
        #extract vx from data. data is of (message type)...?  and has a lot of attributes. we just need vx. and set v to that value.
        #Note that the vx in cmd_vel is in local frame. it is forward dirn in the robot frame. which is exactly what we call 'v' in the odometry expression which you derive.
        #the vx which we use in our formula is global vx. it is not vx of cmd vel. it is v*cos(theta) and this v is what vx of cmd vel represents.

        self.v=data.linear.x
        
    def angle_callback(self,data):
        #extract wz from data. data is of (message type)...?
        self.wz=data.angular_velocity.z
        
        
    #odometry calculation, which you already know
    def calculate(self):
        self.th+=self.wz*self.dt
        self.vx=self.v*cos(self.th)
        self.vy=self.v*sin(self.th)
        self.x+=self.vx*self.dt
        self.y+=self.vy*self.dt
        #print("pose",self.x,self.y,self.th) can be used for debugging

    #Broadcast data
    #basically, this is how we can do a dynamic trannsform between two frames based on data
    def broadcast(self):
        self.current_time=rospy.Time.now()
        self.odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.odom_broadcaster.sendTransform(
        (self.x, self.y, 0.),
        self.odom_quat,
        self.current_time,
        "base_link",
        "odom"
    )
    #Publish data to topic
    def publish(self):
        odom = Odometry() #We want to publish data of type Odometry, so we create an object of it.

        #Now, we set the relevant attributes.
        #You can read up on what the attributes are of message type Odometry.

        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*self.odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.wz))

        # publish the message
        self.odom_pub.publish(odom) #synatx is publisher.publish(object)




    
if __name__ == '__main__':
    odom=odom_task() #create an odom_task object
    r = rospy.Rate(odom.sample_rate)
    while not rospy.is_shutdown():
        try:
            odom.calculate() #calculate odometry
            odom.broadcast() #use it to transform bw frames
            odom.publish() #publish the data 
            r.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass


    