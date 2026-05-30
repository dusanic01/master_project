#!/usr/bin/env python3
import rospy
import rospkg
import os
import yaml
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 as cv 
import numpy as np
from tf2_ros import Buffer, TransformListener
import time
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from tf.transformations import euler_from_matrix

class Camera:
    def __init__(self):
        self.tf_buffer = Buffer(rospy.Duration(10.0))
        self.tf_listener = TransformListener(self.tf_buffer)
        self.bridge = CvBridge()

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("hand_eye_calibration")
        self.file_path = os.path.join(pkg_path, "hand_eye_param", "hand_eye_calib.yaml")

        self.pattern_size = (7,4)
        self.size = 0.035
        self.objp = np.zeros((self.pattern_size[0]*self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2) * self.size

        self.K = None
        self.D = None

        self.R_cam_obj, self.T_cam_obj = [], []
        self.R_rob_ee, self.T_rob_ee = [], []

        self.T_cam_cam = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]], dtype=np.float32)
        
        self.on_pose = False
        self.end = False

        self.i = 60

        #rospy.Subscriber('/camera/color/image_raw', Image, self.detection_callback, queue_size=10) # Za pravu kameru
        #rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_callback) # Za pravu kameru
        rospy.Subscriber('/camera_depth/depth/camera_info', CameraInfo, self.info_callback)
        rospy.Subscriber('/camera_depth/color/image_raw', Image, self.detection_callback)
        rospy.Subscriber('/robot_at_pose', Bool, self.pose_callback)
        rospy.Subscriber('/end_pose', Bool, self.end_callback)

        rospy.loginfo("Čvor za kalibraciju je pokrenut") 
    
    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3)
            self.D = np.array(msg.D)
            rospy.loginfo("Parametri kamere su učitani!")

    def detection_callback(self, msg):
        if not self.on_pose or self.K is None:
            return
        
        self.on_pose = False

        try:
            timestamp = msg.header.stamp
            if not self.tf_buffer.can_transform('world', 'fr3_hand', timestamp, rospy.Duration(0.1)):
                rospy.logwarn("Transformacija base -> hand nije dostupna!")
                return
            
            tf = self.tf_buffer.lookup_transform('world', 'fr3_hand', timestamp)
            q = tf.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            R_robot = R.from_quat(quat).as_matrix()
            t = tf.transform.translation
            tvec_robot = np.array([t.x, t.y, t.z])

            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
            ret, corners = cv.findChessboardCorners(gray, (self.pattern_size[0],self.pattern_size[1]))
            if ret:
                corners = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria=(cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                cv.drawChessboardCorners(image, (self.pattern_size[0],self.pattern_size[1]), corners, ret)
                _, rvec, tvec = cv.solvePnP(self.objp, corners, self.K, self.D)
                R_cam, _ = cv.Rodrigues(rvec)
                # tvec = np.array(tvec).reshape(1,3)
                # T_cam_obj = np.eye(4)
                # T_cam_obj[:3, :3] = R_cam
                # T_cam_obj[:3, 3] = tvec
                # T_pom = self.T_cam_cam @ T_cam_obj
                # R_cam = T_pom[:3, :3]
                # tvec = T_pom[:3,3]
                self.R_cam_obj.append(R_cam)
                self.T_cam_obj.append(tvec.reshape(3,1))
                self.R_rob_ee.append(R_robot)
                self.T_rob_ee.append(tvec_robot.reshape(3,1))
            else:
                rospy.logwarn("Šablon nije detektovana na slici")
            cv.imwrite(f"/home/etf/Image/corners{self.i}.png", image)
            self.i += 1
        
        except Exception as e:
            rospy.logerr(f"Greška: {e}")
            
    def pose_callback(self, msg):
        if msg.data:
            self.on_pose = True
            rospy.loginfo("Robot zauzeo poziciju")

    def end_callback(self, msg):
        if msg.data and len(self.R_cam_obj) >= 3:
            rospy.loginfo("Završna kalibracija")
            try:
                R_ee_cam, t_ee_cam = cv.calibrateHandEye(
                    self.R_rob_ee,
                    self.T_rob_ee,
                    self.R_cam_obj,
                    self.T_cam_obj,
                    method=cv.CALIB_HAND_EYE_PARK
                )
                t_ee_cam = np.array(t_ee_cam).reshape(1,3)
                T_hand_cam = np.eye(4)
                T_hand_cam[:3, :3] = R_ee_cam
                T_hand_cam[:3,3] = t_ee_cam
                
                data = {"T_hand_cam": T_hand_cam.tolist()}
                with open(self.file_path, "w") as f:
                    yaml.dump(data, f)
                rospy.loginfo(f"Traslacija je: {t_ee_cam.reshape(1,3)}")
                rospy.loginfo(f"Rotacija je: {R_ee_cam}")
            
            except Exception as e:
                rospy.logerr(f"Kalibracija neuspješna: {e}")

if __name__ == '__main__':
    rospy.init_node('calibration_node')
    camera = Camera()
    rospy.spin()