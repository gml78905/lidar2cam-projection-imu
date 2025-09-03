#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
os.environ["OPENCV_IO_ENABLE_GDAL"] = "0"   # GDAL 충돌 방지 (cv2 import 전)
import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from message_filters import ApproximateTimeSynchronizer, Subscriber

_LIVOX2_AVAILABLE = True
try:
    from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
except Exception:
    _LIVOX2_AVAILABLE = False


def to_numpy_T(T_list):
    T = np.array(T_list, dtype=np.float64)
    assert T.shape == (4, 4), "Extrinsic matrix must be 4x4"
    return T

def invert_T(T):
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4, dtype=np.float64)
    Ti[:3, :3] = R.T
    Ti[:3, 3]  = -R.T @ t
    return Ti

class LidarToImageProjector(object):
    def __init__(self):
        self.bridge = CvBridge()

        # Topics / params
        img_topic      = rospy.get_param("~image_topic", "/camera/color/image_raw")
        caminfo_topic  = rospy.get_param("~camera_info_topic", "/camera/color/camera_info")
        lidar_topic    = rospy.get_param("~lidar_topic", "/livox/lidar")
        self.lidar_msg_type = rospy.get_param("~lidar_msg_type", "LivoxCustom")

        # Extrinsics
        c2i_dict = rospy.get_param("~cam_to_imu")
        l2i_dict = rospy.get_param("~lidar_to_imu")
        self.T_imu_cam   = to_numpy_T(c2i_dict["T"])
        self.T_imu_lidar = to_numpy_T(l2i_dict["T"])

        # Optional quick test: flip direction if your matrices are opposite of what code expects
        flip_cam_to_imu   = bool(rospy.get_param("~flip_cam_to_imu", False))
        flip_lidar_to_imu = bool(rospy.get_param("~flip_lidar_to_imu", False))
        if flip_cam_to_imu:
            rospy.logwarn("flip_cam_to_imu=true -> inverting T_imu_cam")
            self.T_imu_cam = invert_T(self.T_imu_cam)
        if flip_lidar_to_imu:
            rospy.logwarn("flip_lidar_to_imu=true -> inverting T_imu_lidar")
            self.T_imu_lidar = invert_T(self.T_imu_lidar)

        # LiDAR -> Camera
        self.T_cam_lidar = invert_T(self.T_imu_cam).dot(self.T_imu_lidar)
        rospy.loginfo("T_cam_lidar:\n%s", self.T_cam_lidar)

        # Intrinsics (from CameraInfo OR from config)
        cam_cfg = rospy.get_param("~camera", {})
        self.use_camera_info = bool(cam_cfg.get("use_camera_info", True))
        self.K = None
        self.D = None
        self.img_size = None

        if self.use_camera_info:
            rospy.loginfo("Using CameraInfo topic for intrinsics/distortion.")
            rospy.Subscriber(caminfo_topic, CameraInfo, self._caminfo_cb, queue_size=1)
        else:
            fx, fy, cx, cy = cam_cfg["intrinsics"]
            self.K = np.array([[fx, 0,  cx],
                               [0,  fy, cy],
                               [0,  0,  1 ]], dtype=np.float64)
            D = np.array(cam_cfg.get("distortion", []), dtype=np.float64).reshape(-1)
            # pad to length 5 for OpenCV radtan [k1,k2,p1,p2,k3]
            if D.size == 4:
                D = np.r_[D, 0.0]
            elif D.size == 0:
                D = np.zeros(5, dtype=np.float64)
            self.D = D.reshape(-1, 1)
            res = cam_cfg.get("resolution", [0, 0])
            self.img_size = (int(res[0]), int(res[1]))
            rospy.loginfo("Using intrinsics from config. K=\n%s\nD=%s size=%s", self.K, self.D.ravel(), self.img_size)

        # Visualization params
        self.max_range  = float(rospy.get_param("~max_range", 60.0))
        self.point_size = int(rospy.get_param("~point_size", 2))
        self.draw_alpha = float(rospy.get_param("~draw_alpha", 0.9))

        # Publisher
        self.vis_pub = rospy.Publisher("~projected_image", Image, queue_size=1)

        # Subscribers & Sync
        self.image_sub = Subscriber(img_topic, Image)
        if self.lidar_msg_type.lower() == "pointcloud2":
            self.lidar_sub = Subscriber(lidar_topic, PointCloud2)
        else:
            if not _LIVOX2_AVAILABLE:
                raise RuntimeError("livox_ros_driver2/CustomMsg not available. "
                                   "Install livox_ros_driver2 or set ~lidar_msg_type:=PointCloud2")
            self.lidar_sub = Subscriber(lidar_topic, LivoxCustomMsg)

        self.sync = ApproximateTimeSynchronizer([self.image_sub, self.lidar_sub],
                                                queue_size=10, slop=0.07)
        self.sync.registerCallback(self.cb_sync)

        rospy.loginfo("[lidar_camera_projection] ready. Subscribed: %s, %s, %s (lidar_msg_type=%s)",
                      img_topic, caminfo_topic, lidar_topic, self.lidar_msg_type)

    def _caminfo_cb(self, msg):
        K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.D, dtype=np.float64).reshape(-1)
        if D.size == 4:
            D = np.r_[D, 0.0]
        elif D.size == 0:
            D = np.zeros(5, dtype=np.float64)
        self.K = K
        self.D = D.reshape(-1, 1)
        self.img_size = (int(msg.width), int(msg.height))

    def _points_from_pointcloud2(self, pc_msg):
        pts = []
        for p in pc2.read_points(pc_msg, skip_nans=True, field_names=("x","y","z")):
            x, y, z = float(p[0]), float(p[1]), float(p[2])
            r = (x*x + y*y + z*z) ** 0.5
            if 0.1 < r <= self.max_range:
                pts.append([x, y, z])
        return np.asarray(pts, dtype=np.float64) if pts else None

    def _points_from_livox_custom(self, custom_msg):
        pts = []
        for p in custom_msg.points:
            x = float(p.x); y = float(p.y); z = float(p.z)
            r = (x*x + y*y + z*z) ** 0.5
            if 0.1 < r <= self.max_range:
                pts.append([x, y, z])
        return np.asarray(pts, dtype=np.float64) if pts else None

    def cb_sync(self, img_msg, lidar_msg):
        if self.K is None:
            rospy.logwarn_throttle(5.0, "Intrinsics not ready yet.")
            return

        # Image
        cv_img = CvBridge().imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h, w = cv_img.shape[:2]

        # LiDAR points (Nx3)
        if isinstance(lidar_msg, PointCloud2):
            P = self._points_from_pointcloud2(lidar_msg)
        else:
            P = self._points_from_livox_custom(lidar_msg)
        if P is None or len(P) == 0:
            self.vis_pub.publish(CvBridge().cv2_to_imgmsg(cv_img, encoding="bgr8"))
            return

        # Homogeneous -> Camera frame
        P_lidar = np.hstack([P, np.ones((P.shape[0], 1), dtype=np.float64)])
        P_cam = (self.T_cam_lidar @ P_lidar.T).T
        X, Y, Z = P_cam[:, 0], P_cam[:, 1], P_cam[:, 2]
        valid = Z > 0.1
        X, Y, Z = X[valid], Y[valid], Z[valid]
        if Z.size == 0:
            self.vis_pub.publish(CvBridge().cv2_to_imgmsg(cv_img, encoding="bgr8"))
            return

        # ---- 정확 투영(왜곡 포함) ----
        objp = np.stack([X, Y, Z], axis=1).astype(np.float64).reshape(-1, 1, 3)
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)
        D = self.D if self.D is not None else np.zeros((5, 1), dtype=np.float64)
        proj, _ = cv2.projectPoints(objp, rvec, tvec, self.K, D)
        uv = proj.reshape(-1, 2)
        u = uv[:, 0].astype(np.int32)
        v = uv[:, 1].astype(np.int32)
        # --------------------------------

        in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h)
        u, v, Z = u[in_img], v[in_img], Z[in_img]
        if u.size == 0:
            self.vis_pub.publish(CvBridge().cv2_to_imgmsg(cv_img, encoding="bgr8"))
            return

        # color by depth
        z_norm = np.clip((Z / self.max_range) * 255.0, 0, 255).astype(np.uint8)
        colors = cv2.applyColorMap(z_norm.reshape(-1, 1), cv2.COLORMAP_JET).reshape(-1, 3)

        overlay = cv_img.copy()
        ps = max(1, self.point_size)
        for (uu, vv), col in zip(zip(u, v), colors):
            cv2.circle(overlay, (int(uu), int(vv)), ps, (int(col[0]), int(col[1]), int(col[2])), -1)
        blended = cv2.addWeighted(overlay, self.draw_alpha, cv_img, 1.0 - self.draw_alpha, 0)
        self.vis_pub.publish(CvBridge().cv2_to_imgmsg(blended, encoding="bgr8"))

def main():
    rospy.init_node("lidar_to_image_projector")
    LidarToImageProjector()
    rospy.spin()

if __name__ == "__main__":
    main()
