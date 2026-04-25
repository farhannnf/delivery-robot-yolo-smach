#!/usr/bin/env python3
"""
Coordinate Transformer - REVISI (Depth Image Version)
Sesuai Proposal Farhan - Persamaan 2.6, 2.7, 2.8, 2.9, 2.10
dan Transformasi TF (Persamaan 2.11, 2.12, 2.13)

PERUBAHAN DARI VERSI LAMA:
- Versi lama: subscribe ke PointCloud2 (~9.8 MB per frame)
  → Terlalu besar untuk dikirim via WiFi hotspot
  → Menyebabkan "No point cloud available"
  
- Versi baru: subscribe ke depth IMAGE (~600 KB per frame)  
  → 16x lebih kecil, bisa lewat WiFi hotspot
  → Menghitung 3D coordinates secara manual menggunakan
    rumus pinhole projection (Persamaan 2.6, 2.7, 2.8)
  → Hasilnya IDENTIK karena nodelet point cloud pun
    menggunakan rumus yang sama secara internal

ALUR TETAP SAMA:
  PIXEL (u,v) → 3D CAMERA (X,Y,Z) → MAP (X,Y,Z)

OUTPUT TOPIC TETAP SAMA:
  /detected_positions (geometry_msgs/PointStamped)
  → Tidak perlu mengubah node lain yang subscribe ke topic ini
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge


class CoordinateTransformerComplete:
    def __init__(self):
        rospy.init_node('coordinate_transformer_complete')
        
        self.bridge = CvBridge()
        
        # ============================================================
        # TF listener untuk transformasi frame (camera → map)
        # Digunakan di STEP 3 (Persamaan 2.11, 2.12, 2.13)
        # ============================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # ============================================================
        # Camera intrinsic parameters (dari CameraInfo)
        # K matrix: [fx  0 cx]
        #           [ 0 fy cy]
        #           [ 0  0  1]
        # Digunakan di STEP 2 (Persamaan 2.6, 2.7)
        # ============================================================
        self.fx = None   # Focal length X
        self.fy = None   # Focal length Y
        self.cx = None   # Principal point X
        self.cy = None   # Principal point Y
        self.camera_info_received = False
        
        # ============================================================
        # Depth image terbaru (pengganti point cloud)
        # ============================================================
        self.latest_depth = None
        
        # ============================================================
        # Parameter depth
        # ============================================================
        self.min_depth = 0.1    # meter, di bawah ini dianggap invalid
        self.max_depth = 5.0    # meter, di atas ini dianggap invalid
        self.patch_size = 5     # ukuran area sampling (5x5 pixel)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Coordinate Transformer - REVISI (Depth Image Version)")
        rospy.loginfo("Menggunakan depth image, BUKAN point cloud")
        rospy.loginfo("Bandwidth: ~600 KB/frame (vs 9.8 MB/frame point cloud)")
        rospy.loginfo("=" * 60)
        
        # ============================================================
        # SUBSCRIBER 1: Camera Info (parameter intrinsik kamera)
        # Topic ini sangat kecil, tidak membebani jaringan
        # ============================================================
        rospy.Subscriber(
            '/camera/depth_registered/camera_info', 
            CameraInfo, 
            self.camera_info_callback
        )
        
        # ============================================================
        # SUBSCRIBER 2: Depth Image (PENGGANTI POINT CLOUD)
        # 
        # PENTING: buff_size=2**24 (16 MB) diperlukan karena:
        # Default buffer rospy hanya 64 KB, sedangkan satu frame
        # depth image ~600 KB. Kalau buffer lebih kecil dari pesan,
        # rospy mengabaikan queue_size=1 dan menumpuk pesan lama.
        # Dengan buffer besar, hanya frame terbaru yang diproses.
        # ============================================================
        rospy.Subscriber(
            '/camera/depth_registered/image_raw',
            Image,
            self.depth_callback,
            queue_size=1,
            buff_size=2**24    # 16 MB buffer, mencegah lag
        )
        
        # ============================================================
        # SUBSCRIBER 3: YOLO detections (koordinat pixel)
        # Format: Point(x=u_center, y=v_center, z=confidence)
        # Topic ini sangat kecil, tidak membebani jaringan
        # ============================================================
        rospy.Subscriber(
            '/yolo/detections', 
            Point, 
            self.detection_callback
        )
        
        # ============================================================
        # PUBLISHER: Koordinat map (hasil transformasi akhir)
        # Topic ini SAMA dengan versi lama → kompatibel
        # ============================================================
        self.coord_pub = rospy.Publisher(
            '/detected_positions', 
            PointStamped, 
            queue_size=10
        )
        
        rospy.loginfo("Subscribed to: /camera/depth_registered/camera_info")
        rospy.loginfo("Subscribed to: /camera/depth_registered/image_raw")
        rospy.loginfo("Subscribed to: /yolo/detections")
        rospy.loginfo("Publishing to: /detected_positions")
        rospy.loginfo("Waiting for camera_info dan depth image...")
    
    # ================================================================
    # CALLBACK 1: Menerima parameter intrinsik kamera
    # ================================================================
    def camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.fx = msg.K[0]  # K[0] = fx
            self.fy = msg.K[4]  # K[4] = fy
            self.cx = msg.K[2]  # K[2] = cx
            self.cy = msg.K[5]  # K[5] = cy
            self.camera_info_received = True
            
            rospy.loginfo("=== Camera Intrinsic Parameters ===")
            rospy.loginfo("Focal Length: fx=%.2f, fy=%.2f", self.fx, self.fy)
            rospy.loginfo("Principal Point: cx=%.2f, cy=%.2f", self.cx, self.cy)
            rospy.loginfo("===================================")
    
    # ================================================================
    # CALLBACK 2: Menerima depth image (PENGGANTI point cloud)
    # ================================================================
    def depth_callback(self, msg):
        """
        Menyimpan depth image terbaru.
        Format depth image dari ASUS Xtion:
        - Tipe: 16UC1 (16-bit unsigned, 1 channel)
        - Satuan: milimeter (mm)
        - Nilai 0 = tidak ada data depth (invalid)
        """
        try:
            # Konversi ROS Image → numpy array
            # 'passthrough' = pertahankan format asli (16UC1)
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough'
            )
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Gagal konversi depth image: %s" % str(e))
    
    # ================================================================
    # FUNGSI BANTU: Ambil depth di pixel (u,v) dengan patch sampling
    # ================================================================
    def get_depth_at_pixel(self, u, v):
        """
        Mengambil nilai depth di sekitar pixel (u,v) menggunakan
        median dari patch kecil (5x5 pixel). Median digunakan
        karena lebih robust terhadap noise dibanding mengambil
        satu pixel saja.
        
        Return: depth dalam METER, atau None jika invalid
        """
        if self.latest_depth is None:
            return None
        
        h, w = self.latest_depth.shape[:2]
        
        # Batas area patch 5x5 di sekitar pixel (u,v)
        half = self.patch_size // 2
        v_start = max(0, v - half)
        v_end   = min(h, v + half + 1)
        u_start = max(0, u - half)
        u_end   = min(w, u + half + 1)
        
        # Ambil patch dari depth image
        patch = self.latest_depth[v_start:v_end, u_start:u_end].astype(np.float64)
        
        # Konversi satuan:
        # ASUS Xtion depth image format 16UC1 → satuan milimeter
        # Kita konversi ke meter
        if self.latest_depth.dtype == np.uint16:
            patch = patch / 1000.0          # mm → meter
            valid = patch[patch > 0]         # 0 mm = invalid
        else:
            # Format 32FC1 → sudah dalam meter, NaN = invalid
            valid = patch[~np.isnan(patch) & (patch > 0)]
        
        # Filter depth yang di luar jangkauan kamera
        # ASUS Xtion: jangkauan efektif 0.8 - 3.5 meter
        valid = valid[(valid >= self.min_depth) & (valid <= self.max_depth)]
        
        if len(valid) == 0:
            return None
        
        # Gunakan MEDIAN (bukan mean) → lebih tahan terhadap outlier
        return float(np.median(valid))
    
    # ================================================================
    # CALLBACK 3: Proses deteksi YOLO → hitung koordinat 3D → transform ke map
    # ================================================================
    def detection_callback(self, detection):
        """
        Main function: Transform dari pixel ke map
        Alur: PIXEL (u,v) → 3D CAMERA (X,Y,Z) → MAP (X,Y,Z)
        
        STEP 1: Ambil koordinat pixel dari YOLO (Persamaan 2.9, 2.10)
        STEP 2: Hitung koordinat 3D di camera frame (Persamaan 2.6, 2.7, 2.8)
        STEP 3: Transform ke map frame menggunakan TF (Persamaan 2.11, 2.12, 2.13)
        STEP 4: Publish hasil
        """
        
        # === CEK PRASYARAT ===
        if not self.camera_info_received:
            rospy.logwarn_throttle(5.0, "Camera info belum diterima")
            return
        
        if self.latest_depth is None:
            rospy.logwarn_throttle(5.0, "Depth image belum diterima")
            return
        
        # ========================================================
        # STEP 1: Ambil koordinat pixel dari YOLO
        # u_center = (x_min + x_max) / 2  → Persamaan 2.9
        # v_center = (y_min + y_max) / 2  → Persamaan 2.10
        # ========================================================
        u = int(detection.x)       # u_center (horizontal pixel)
        v = int(detection.y)       # v_center (vertical pixel)
        confidence = detection.z   # confidence score YOLO
        
        rospy.loginfo(">>> Deteksi YOLO: pixel (%d, %d) conf=%.2f", u, v, confidence)
        
        try:
            # ========================================================
            # STEP 2: Hitung koordinat 3D dalam camera frame
            # Menggunakan Persamaan 2.6, 2.7, 2.8 dari proposal
            #
            # Z = depth (meter)                    → Persamaan 2.8
            # X = (u - cx) * Z / fx                → Persamaan 2.6
            # Y = (v - cy) * Z / fy                → Persamaan 2.7
            #
            # CATATAN: Rumus ini IDENTIK dengan yang digunakan oleh
            # nodelet depth_image_proc/point_cloud_xyzrgb secara internal.
            # Jadi hasilnya sama persis dengan query point cloud.
            # ========================================================
            Z = self.get_depth_at_pixel(u, v)
            
            if Z is None:
                rospy.logwarn("Depth tidak valid di pixel (%d, %d)", u, v)
                return
            
            # Hitung X dan Y menggunakan rumus pinhole projection
            X_cam = (u - self.cx) * Z / self.fx   # Persamaan 2.6
            Y_cam = (v - self.cy) * Z / self.fy   # Persamaan 2.7
            
            rospy.loginfo(">>> Camera frame: X=%.3f, Y=%.3f, Z=%.3f (meter)", 
                         X_cam, Y_cam, Z)
            
            # ========================================================
            # STEP 3: Transform ke MAP frame menggunakan TF
            # Implementasi Persamaan 2.11 (Matriks Transformasi Homogen)
            #         Persamaan 2.12 (Transformasi titik A → B)
            #         Persamaan 2.13 (Camera frame → map frame)
            # ========================================================
            
            # Buat PointStamped di camera frame
            point_cam = PointStamped()
            point_cam.header.frame_id = "camera_depth_optical_frame"
            point_cam.header.stamp = rospy.Time(0)  # Gunakan transform terbaru
            point_cam.point.x = X_cam
            point_cam.point.y = Y_cam
            point_cam.point.z = Z
            
            try:
                # Lookup transform: camera_frame → map_frame
                point_map = self.tf_buffer.transform(
                    point_cam, 
                    "map",               # Target frame
                    rospy.Duration(1.0)   # Timeout 1 detik
                )
                
                rospy.loginfo(">>> Map frame: X=%.3f, Y=%.3f, Z=%.3f", 
                             point_map.point.x, 
                             point_map.point.y,
                             point_map.point.z)
                
                # ====================================================
                # STEP 4: Publish hasil transformasi
                # ====================================================
                self.coord_pub.publish(point_map)
                
                rospy.loginfo("=" * 60)
                rospy.loginfo("TRANSFORMASI SELESAI:")
                rospy.loginfo("  Pixel     : (%d, %d)", u, v)
                rospy.loginfo("  Confidence: %.2f", confidence)
                rospy.loginfo("  Depth     : %.3f meter", Z)
                rospy.loginfo("  Camera    : (%.3f, %.3f, %.3f)", X_cam, Y_cam, Z)
                rospy.loginfo("  Map       : (%.3f, %.3f, %.3f)", 
                             point_map.point.x, 
                             point_map.point.y, 
                             point_map.point.z)
                rospy.loginfo("=" * 60)
                
            except (tf2_ros.LookupException, 
                    tf2_ros.ConnectivityException, 
                    tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF transform gagal: %s", str(e))
                rospy.logerr("Periksa TF tree: rosrun tf view_frames")
                
        except Exception as e:
            rospy.logerr("Error di detection_callback: %s", str(e))


if __name__ == '__main__':
    try:
        transformer = CoordinateTransformerComplete()
        rospy.loginfo("Coordinate Transformer ready. Menunggu deteksi YOLO...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
