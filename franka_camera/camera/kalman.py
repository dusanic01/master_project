#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv 
import numpy as np
from ultralytics import YOLO
from tf2_ros import Buffer, TransformListener
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Pose

class Camera:
    def __init__(self):
        self.parameters()
        self.model = YOLO("/home/etf/YoloV8/best.pt")
        self.tf_buffer = Buffer()
        #self.tf_listener = TransformListener(self.tf_buffer, queue_size=10)
        
        self.depth = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback, queue_size=10)
        self.sub = rospy.Subscriber('/camera/color/image_raw', Image, self.detection_callback, queue_size=10)
        
        self.pose_pub = rospy.Publisher('/pose', Pose, queue_size=10)
        self.pom_pose_pub = rospy.Publisher('/pom_pose', Pose, queue_size=10)
        
        self.bridge = CvBridge()
        self.pose = Pose()
        self.pom_pose = Pose()
        
        self.depth_image = None
        self.T_world_hand = None
        self.T_hand_camera = np.array([[0, -1, 0, 0.06], [1, 0, 0, 0], [0, 0, 1, 0.06], [0, 0, 0, 1]])
        
        self.kalmans = {}
        self.next_id = 0
        self.max_distance = 0.05 
    
    def init_kalman(self):
        kf = cv.KalmanFilter(6, 3)
        dt = 0.1 
        kf.transitionMatrix = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]], np.float32)

        kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]], np.float32)

        kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-7
        
        kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 0.5
        
        kf.errorCovPost = np.eye(6, dtype=np.float32)
        return kf

    def depth_callback(self, msg):
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.depth_image = depth.astype(np.float32)

    def detection_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(image, save=False)
        result = results[0]
        h, w, _  = image.shape
        
        if result.masks is not None:
            active_ids = []
            for i, m in enumerate(result.masks.data):
                mask = m.cpu().numpy()
                mask = cv.resize(mask, (w, h))
                mask = (mask * 255).astype(np.uint8)
                
                if self.depth_image is None: continue

                points_obj = self.point_cloud_obj(mask)
                points = self.ransac_algorithm(points_obj)
                if points is None: continue
                
                raw_center = points.mean(axis=0).astype(np.float32)
                obj_id = self.assign_id(raw_center)
                active_ids.append(obj_id)
                
                center = self.center_filtred(image, raw_center, obj_id)
                
                points_ = self.obj_coord_sys(points, None)
                self.draw_axis_camera(image, points_, np.zeros((3,1)), np.zeros((3,1)))
                
                if self.T_world_hand is not None:
                    T_world_camera = self.T_world_hand @ self.T_hand_camera
                    T_world_object = T_world_camera @ self.T_camera_object
                    self.publish_callback(T_world_object)
            
            self.kalmans = {k: v for k, v in self.kalmans.items() if k in active_ids}
        else:
            self.kalmans = {}

        cv.imshow("frame", image)
        cv.waitKey(1)

    def center_filtred(self, image, raw_center, obj_id):
        if obj_id not in self.kalmans:
            kf = self.init_kalman()
            kf.statePost = np.array([raw_center[0], raw_center[1], raw_center[2], 0, 0, 0], np.float32).reshape(-1, 1)
            self.kalmans[obj_id] = kf

        kf = self.kalmans[obj_id]
        kf.predict()
        estimate = kf.correct(raw_center.reshape(-1, 1))
        
        smoothed_center = estimate[0:3].flatten()
        
        Xc, Yc, Zc = smoothed_center
        u = (self.K[0,0]*Xc)/Zc + self.K[0,2]
        v = (self.K[1,1]*Yc)/Zc + self.K[1,2]
        #cv.circle(image, (int(round(u)), int(round(v))), 4, (0, 0, 255), -1)
        
        return smoothed_center

    def assign_id(self, current_center):
        if not self.kalmans:
            obj_id = self.next_id
            self.next_id += 1
            return obj_id

        distances = {}
        for kid, kf in self.kalmans.items():
            prev_pos = kf.statePost[0:3].flatten()
            distances[kid] = np.linalg.norm(current_center - prev_pos)

        nearest_id = min(distances, key=distances.get)
        if distances[nearest_id] < self.max_distance:
            return nearest_id
        else:
            obj_id = self.next_id
            self.next_id += 1
            return obj_id
    
    def point_cloud_obj(self, mask):
        ys, xs = np.where(mask > 0)
        Z = self.depth_image[ys,xs].astype(np.float32)
        valid = (Z > 0) & (np.isfinite(Z))
        xs, ys, Z = xs[valid], ys[valid], Z[valid]
        Xc = ((xs-self.K[0,2])*Z)/self.K[0,0]
        Yc = ((ys-self.K[1,2])*Z)/self.K[1,1]
        return np.stack((Xc,Yc,Z), axis=1)
    
    def obj_coord_sys(self, points, center_guess):
        mean_center = points.mean(axis=0)
        points_centered = points - mean_center
        
        cov = np.cov(points_centered.T)
        _, eig_vect = np.linalg.eigh(cov)
        
        idx = np.argsort(np.linalg.norm(eig_vect, axis=0))[::-1]
        eig_vect = eig_vect[:, idx]
        
        xc, yc, zc = eig_vect[:,0], eig_vect[:,1], eig_vect[:,2]
        
        if zc[2] > 0: zc = -zc 
        if xc[1] > 0: xc = -xc
        yc = np.cross(zc, xc)  
        
        self.R_camera_object = np.column_stack((xc, yc, zc))

        local_points = points_centered @ self.R_camera_object
        
        local_min = local_points.min(axis=0)
        local_max = local_points.max(axis=0)
        
        local_center = (local_min + local_max) / 2.0
   
        true_center = mean_center + (local_center @ self.R_camera_object.T)
        
        self.T_camera_object = np.eye(4, dtype=np.float32)
        self.T_camera_object[:3,:3] = self.R_camera_object
        self.T_camera_object[:3, 3] = true_center
        
        return np.array([true_center, true_center + 0.2*xc, true_center + 0.2*yc, true_center + 0.2*zc], dtype=np.float32)

    def ransac_algorithm(self, points_obj):
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_obj) 
        if len(pcd.points) < 3: 
            rospy.loginfo("RANSAC - too few points!")
            return None
        _, inliers = pcd.segment_plane(
            distance_threshold=2,
            ransac_n=3,
            num_iterations=1000
        )
        plane_points_1 = pcd.select_by_index(inliers)
        pcd = pcd.select_by_index(inliers, invert=True)
        if len(pcd.points) < 3: # Provjera i traženje druge ravni
            rospy.loginfo("RANSAC - too few points!")
            return None
        _, inliers = pcd.segment_plane(
            distance_threshold=2,
            ransac_n=3,
            num_iterations=1000
        )
        plane_points_2 = pcd.select_by_index(inliers)
        if len(plane_points_1.points) > len(plane_points_2.points): 
            points = np.asarray(plane_points_1.points)
        else:
            points = np.asarray(plane_points_2.points)
        return points

    def draw_axis_camera(self, image, points, rvec, tvec):
        imgpts, _ = cv.projectPoints(points, rvec, tvec, self.K, self.D)
        imgpts = imgpts.reshape(-1, 2).astype(int)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[1]), (0, 0, 255), 2)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[2]), (0, 255, 0), 2)
        cv.line(image, tuple(imgpts[0]), tuple(imgpts[3]), (255, 0, 0), 2)

    def publish_callback(self, T_world_object):
        Xw, Yw, Zw, _ = T_world_object @ np.array([0, 0, 0.0884, 1], dtype=np.float32)
        quat = R.from_matrix((self.T_hand_camera @ self.T_camera_object)[:3,:3]).as_quat()
        self.pose.position.x, self.pose.position.y, self.pose.position.z = Xw, Yw, Zw
        self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w = quat
        self.pose_pub.publish(self.pose)

    def parameters(self):
        self.K = np.array([[382.6377, 0, 321.1927], [0, 381.8234, 241.2623], [0, 0, 1]], dtype=np.float32)
        self.D = np.array([-0.056515, 0.066448, -0.000763, -0.0008797, -0.021259], dtype=np.float32)

if __name__ == '__main__':
    rospy.init_node('camera_node')
    camera = Camera()
    rospy.spin()