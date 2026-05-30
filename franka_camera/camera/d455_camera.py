#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv 
import numpy as np
import yaml
import torch
import rospkg, os
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import open3d as o3d
import math
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, CameraInfo

class Camera:
    def __init__(self):
        self.model = YOLO("/home/etf/YoloV8/best.pt")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, queue_size=10)

        self.bridge = CvBridge()
        self.pose = Pose()
        self.pom_pose = Pose()
        self.K = None
        self.D = None
        self.depth_image = None

        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("hand_eye_calibration")
        file_path = os.path.join(pkg_path, "hand_eye_param", "hand_eye_calib.yaml")
        with open(file_path, "r") as f:
            data = yaml.safe_load(f)
        self.T_hand_camera = np.array(data["T_hand_cam"])
        rospy.loginfo(f"Matrica je: {self.T_hand_camera}")
        # Filtriranje centra kamere
        self.center_prev = {}
        self.next_id = 0
        self.max_distance = 0.05
        self.alpha = 0.2

        #rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=10) # Za pravu kameru
        #rospy.Subscriber('/camera/color/image_raw', Image, self.detection_callback, queue_size=10) # Za pravu kameru
        #rospy.Subscriber('/camera/color/camera_info', CameraInfo, self.info_callback) # Za pravu kameru
        rospy.Subscriber('/camera_depth/depth/image_raw', Image, self.depth_callback, queue_size=1) # Za simulaciju
        rospy.Subscriber('/camera_depth/color/image_raw', Image, self.detection_callback, queue_size=1) # Za simulaciju
        rospy.Subscriber('/camera_depth/depth/camera_info', CameraInfo, self.info_callback) # Za simulaciju
        self.pose_pub = rospy.Publisher('/pose', Pose, queue_size=10)
        self.pom_pose_pub = rospy.Publisher('/pom_pose', Pose, queue_size=10)
        
        rospy.loginfo("Kamera čvor je učitan!")
    
    def info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.K).reshape(3, 3)
            self.D = np.array(msg.D)
            rospy.loginfo("Parametri kamere su učitani!")

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = depth.astype(np.float32)

    def detection_callback(self, msg):
        if self.K is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(image, device='cuda', save=False)
        result = results[0]
        h, w, _  = image.shape
        if result.masks is not None:
            rospy.loginfo("Detektovan je objekat")
            for i, m in enumerate(results[0].masks.data):
                mask = m.cpu().numpy()
                mask = cv.resize(mask, (w, h))
                mask = (mask * 255).astype(np.uint8)
                if self.depth_image is None:
                    rospy.loginfo("Depth not received yet")
                    continue
                else:
                    rospy.loginfo("Depth received")
                points_obj = self.point_cloud_obj(mask)
                if points_obj.shape[0] < 3:
                    continue
                points = self.ransac_algorithm(points_obj)
                if points is None: 
                    continue
                if not self.object_dimension(points):
                    continue
                raw_center = points.mean(axis=0).astype(np.float32)
                obj_id = self.assign_id(raw_center)
                center = self.center_filtred(image, points, obj_id)
                points_ = self.obj_coord_sys(points, center)
                rvec_ = tvec_ = np.zeros((3,1))
                self.draw_axis_camera(image, points_, rvec_, tvec_)
                T_world_hand = self.transform()
                if T_world_hand is None:
                    continue
                T_world_camera = T_world_hand @ self.T_hand_camera
                T_world_object = T_world_camera @ self.T_camera_object
                self.publish_callback(T_world_object)
                rospy.loginfo(T_world_object)
        cv.imshow("frame", image)
        cv.waitKey(1)

    def transform(self):
        try:
            tf = self.tf_buffer.lookup_transform('world', 'fr3_hand', rospy.Time(0))
            t = tf.transform.translation
            tvec = np.array([t.x, t.y, t.z])
            q = tf.transform.rotation
            quat = [q.x, q.y, q.z, q.w]
            R_mat = R.from_quat(quat).as_matrix()
            T_world_hand = np.eye(4)
            T_world_hand[0:3, 0:3] = R_mat
            T_world_hand[0:3,3] = tvec
            return T_world_hand
        except Exception as e:
            rospy.logwarn(str(e))
            return None

    def publish_callback(self, T_world_object):
        Xw, Yw, Zw, _ = T_world_object @ np.array([[0], [0], [0.095], [1]], dtype=np.float32)
        T_hand_object = self.T_hand_camera @ self.T_camera_object
        T_h_o = T_hand_object @ np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        R_h_o = T_h_o[:3,:3]
        quat = R.from_matrix(R_h_o).as_quat()
        self.pose.position.x = Xw
        self.pose.position.y = Yw
        self.pose.position.z = Zw
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]
        self.pose_pub.publish(self.pose)
        pom = T_world_object @ np.array([[0], [0], [0.25], [1]], dtype=np.float32)
        Xw_pom, Yw_pom, Zw_pom, _ = pom
        self.pom_pose.position.x = Xw_pom
        self.pom_pose.position.y = Yw_pom
        self.pom_pose.position.z = Zw_pom
        self.pom_pose_pub.publish(self.pom_pose)

    def point_cloud_obj(self, mask):
        ys, xs = np.where(mask > 0)
        Z = self.depth_image[ys,xs].astype(np.float32) + (0.06 - self.T_hand_camera[2,3]) # Dodajem ovo da bi dubina bila dobra!
        valid = (Z > 0) & (np.isfinite(Z))
        xs = xs[valid]
        ys = ys[valid]
        Z = Z[valid]
        Xc = ((xs-self.K[0,2])*Z)/self.K[0,0]
        Yc = ((ys-self.K[1,2])*Z)/self.K[1,1]
        points_obj = np.stack((Xc,Yc,Z), axis=1)
        return points_obj

    def ransac_algorithm(self, points_obj):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_obj) # Kreiranje objekta pogodnog za primjenu RANSAC algoritma
        if len(pcd.points) < 3: # Provjera i traženje ravni prvi put
            rospy.loginfo("RANSAC - too few points!")
            return None
        _, inliers = pcd.segment_plane(
            distance_threshold=0.002,
            ransac_n=3,
            num_iterations=300
        )
        plane_points_1 = pcd.select_by_index(inliers)
        pcd = pcd.select_by_index(inliers, invert=True)
        if len(pcd.points) < 3: # Provjera i traženje druge ravni
            rospy.loginfo("RANSAC - too few points!")
            points = np.asarray(plane_points_1.points)

        else:
            _, inliers = pcd.segment_plane(
                distance_threshold=0.002,
                ransac_n=3,
                num_iterations=300
            )
            plane_points_2 = pcd.select_by_index(inliers)
            if len(plane_points_1.points) > len(plane_points_2.points): # Zadržava se ravan kojoj pripada više tačaka, sa pretpostavkom da je ta ravan bliže kameri 
                points = np.asarray(plane_points_1.points)
            else:
                points = np.asarray(plane_points_2.points)
        return points


    def center_filtred(self, image, points, obj_id):
        center = points.mean(axis=0)
        center = np.array(center, dtype=np.float32)
        if obj_id not in self.center_prev:
            self.center_prev[obj_id] = center
        smoothed_center = self.alpha * center + (1 - self.alpha) * self.center_prev[obj_id]
        smoothed_center = np.array(smoothed_center, dtype=np.float32)
        Xc, Yc, Zc = smoothed_center
        u = (self.K[0,0]*Xc)/Zc + self.K[0,2]
        v = (self.K[1,1]*Yc)/Zc + self.K[1,2]
        rospy.loginfo(f"Dubina centra je: {self.depth_image[int(v),int(u)]+0.06}")
        cv.circle(image, (int(round(u)), int(round(v))), 2, (0, 0, 255), -1)
        self.center_prev[obj_id] = smoothed_center
        return smoothed_center
    
    def assign_id(self, current_center):
        if len(self.center_prev) == 0:
            obj_id = self.next_id
            self.next_id += 1
            return obj_id

        # traži najbliži prethodni centar
        distances = {
            k: np.linalg.norm(current_center - v)
            for k, v in self.center_prev.items()
        }

        nearest_id = min(distances, key=distances.get)

        # ako je blizu → isti objekat
        if distances[nearest_id] < self.max_distance:
            return nearest_id
        else:
            # novi objekat
            obj_id = self.next_id
            self.next_id += 1
            return obj_id

    def obj_coord_sys(self, points, center):
        points_centered = points - center
        cov = np.cov(points_centered.T)
        eig_vals, eig_vect = np.linalg.eigh(cov)
        idx = np.argsort(eig_vals)[::-1]
        eig_vals = eig_vals[idx]
        eig_vect = eig_vect[:, idx]
        xc_obj = eig_vect[:,0]
        yc_obj = eig_vect[:,1]
        zc_obj = eig_vect[:,2]
        xc_obj /= np.linalg.norm(xc_obj) # Jedinični vektor
        zc_obj = zc_obj / np.linalg.norm(zc_obj)
        if zc_obj[2] > 0: # Orijentacija ka kameri
            zc_obj = -zc_obj
        if xc_obj[1] > 0:
            xc_obj = -xc_obj
        yc_obj = np.cross(zc_obj, xc_obj) # Dobijanje desnog koordinantog sistema
        self.R_camera_object = np.column_stack((xc_obj, yc_obj, zc_obj))
        points_ = np.array([center, center + 0.02*xc_obj, center + 0.02*yc_obj, center + 0.02*zc_obj], dtype=np.float32)
        self.T_camera_object = np.eye(4, dtype=np.float32)
        self.T_camera_object[:3,:3] = self.R_camera_object
        self.T_camera_object[:3, 3] = center
        return points_
    
    def object_dimension(self, points):
        rvec_ = tvec_ = np.zeros((3,1))
        imgpts, _ = cv.projectPoints(points, rvec=rvec_, tvec=tvec_, cameraMatrix=self.K, distCoeffs=self.D)
        imgpts = imgpts.reshape(-1,2).astype(int)
        rect = cv.minAreaRect(imgpts)
        box = cv.boxPoints(rect)
        corner_idx = []
        for corner in box:
            idx = np.argmin(np.linalg.norm(imgpts - corner, axis=1))
            corner_idx.append(idx)
        corner = points[corner_idx]
        distances = np.linalg.norm(corner[1:]-corner[0], axis=1)
        distances = np.sort(distances)
        length = distances[1]
        width = distances[0]
        #rospy.loginfo(f"Dužina = {length}, širina = {width}")

        if (length >= 0.085 and length <= 0.115) and (width >= 0.020 and width <= 0.040):
            return True
        else:
            return False

    def draw_axis_camera(self, image, points, rvec , tvec):
        imgpts, _ = cv.projectPoints(
            points,
            rvec=rvec,
            tvec=tvec,
            cameraMatrix=self.K,
            distCoeffs=self.D
        )
        imgpts = imgpts.reshape(-1, 2).astype(int)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 1)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 1)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 1)

if __name__ == '__main__':
    rospy.init_node('camera_node')
    camera = Camera()
    rospy.spin()