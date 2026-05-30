#!/usr/bin/env python3
import rospy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose

class End_Effector:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, queue_size=10)
        self.pose_pub = rospy.Publisher('/fr3_end_effector', Pose, queue_size=10)   
    
    def transform(self):
        try:
            tf = self.tf_buffer.lookup_transform('world', 'fr3_hand', rospy.Time(0))
            pose_msg = Pose()
            pose_msg.position.x = tf.transform.translation.x
            pose_msg.position.y = tf.transform.translation.y
            pose_msg.position.z = tf.transform.translation.z
            pose_msg.orientation = tf.transform.rotation
            self.pose_pub.publish(pose_msg)

        except Exception as e:
            rospy.logwarn(str(e))
            return None
    
    def run(self):
        rate = rospy.Rate(30) 
        
        while not rospy.is_shutdown():
            self.transform() 
            rate.sleep()    

if __name__ == '__main__':
    rospy.init_node('end_effector_node')
    ee_node = End_Effector()
    ee_node.run()