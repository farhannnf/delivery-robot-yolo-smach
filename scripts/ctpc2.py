#!/usr/bin/env python3
"""
coordinate_transformer_pc.py (Point Cloud Version)
====================================================
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

Script ini adalah ALTERNATIF dari coordinate_transformer.py yang sudah ada.
Perbedaan utama: script ini menggunakan PointCloud2 untuk mendapatkan
koordinat 3D, bukan depth image + rumus pinhole projection.

PENDEKATAN INI BERASAL DARI KODE ASLI PAK YANI (detect_ros_TF.py).
Dalam kode Pak Yani, koordinat 3D diperoleh dengan cara:
    pc2.read_points(point_cloud, uvs=[(x_center, y_center)])
yang langsung mengembalikan (x, y, z) dalam frame kamera tanpa
perlu menghitung rumus pinhole projection secara manual.

TUJUAN SCRIPT INI:
  - Sebagai eksperimen untuk membuktikan bahwa hasil transformasi
    dari pendekatan point cloud dan pendekatan depth image KONSISTEN
    (menghasilkan koordinat map yang sama atau sangat mendekati)
  - Sebagai cadangan kalau depth image approach mengalami masalah
  - Sebagai bahan dokumentasi di Bab IV bahwa kedua pendekatan
    telah diuji dan dibandingkan

PERBEDAAN DENGAN coordinate_transformer.py:
  ┌──────────────────────┬──────────────────────────────┐
  │ coordinate_transformer│ coordinate_transformer_pc    │
  │ (yang sudah ada)      │ (script ini)                 │
  ├──────────────────────┼──────────────────────────────┤
  │ Subscribe depth image │ Subscribe PointCloud2        │
  │ ~600 KB/frame         │ ~9.8 MB/frame                │
  │ Hitung manual:        │ Query langsung:              │
  │   X = (u-cx)*Z/fx     │   pc2.read_points(uvs=[..]) │
  │   Y = (v-cy)*Z/fy     │                              │
  │ Perlu camera_info     │ Tidak perlu camera_info      │
  │ Publish ke:           │ Publish ke:                  │
  │   /detected_positions │   /detected_positions_pc     │
  └──────────────────────┴──────────────────────────────┘

TOPIC OUTPUT BERBEDA agar kedua pipeline bisa berjalan bersamaan:
  - coordinate_transformer.py   → /detected_positions    (depth image)
  - coordinate_transformer_pc.py → /detected_positions_pc (point cloud)
  scanning_mode.py subscribe ke /detected_positions (pipeline utama)
  Untuk perbandingan, bisa subscribe ke kedua topic secara manual

CATATAN PENTING TENTANG BANDWIDTH:
  PointCloud2 berukuran ~9.8 MB per frame pada resolusi 640x480.
  Kalau kamera di robot dan script ini di Remote PC via WiFi,
  bandwidth mungkin tidak cukup dan menyebabkan lag parah.
  Script ini PALING COCOK dijalankan di komputer yang SAMA dengan
  kamera (misalnya langsung di Jetson Xavier) atau via koneksi
  ethernet kabel. Untuk WiFi, tetap gunakan coordinate_transformer.py.

CARA MENJALANKAN:
  1. Pastikan sudah ada point cloud topic yang aktif:
     rostopic list | grep points
     (biasanya /camera/depth_registered/points atau /camera/depth/points)
  2. Jalankan:
     rosrun ta2_farhan coordinate_transformer_pc.py
  3. Untuk menjalankan BERSAMAAN dengan versi depth image:
     Terminal 1: rosrun ta2_farhan coordinate_transformer.py
     Terminal 2: rosrun ta2_farhan coordinate_transformer_pc.py
     Lalu bandingkan output di terminal atau:
       rostopic echo /detected_positions      (depth image)
       rostopic echo /detected_positions_pc   (point cloud)

REFERENSI:
  - Kode asli Pak Yani: detect_ros_TF.py (baris 211-214)
  - Proposal: Persamaan 2.11, 2.12, 2.13 (transformasi TF)
  - Bimbingan 12 Desember 2025: "ngambil dari bounding box-nya
    point cloud, nanti kan tengahnya itu ditransformasi ke dari
    dua dimensi ke tiga dimensi"
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
import sensor_msgs.point_cloud2 as pc2


class CoordinateTransformerPointCloud:
    def __init__(self):
        rospy.init_node('coordinate_transformer_pc')

        # ==============================================================
        # TF2 BUFFER DAN LISTENER
        # ==============================================================
        # Menggunakan tf2_ros (bukan tf seperti di kode Pak Yani)
        # karena tf2 lebih modern, thread-safe, dan sudah dipakai
        # di seluruh pipeline kita yang lain.
        #
        # Perbedaan dengan kode Pak Yani:
        #   Pak Yani: self.tf_listener = tf.TransformListener()
        #   Kita:     self.tf_buffer + tf2_ros.TransformListener
        # Fungsionalitasnya sama — keduanya lookup transform dari
        # TF tree yang di-broadcast oleh AMCL dan odometry.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ==============================================================
        # POINT CLOUD BUFFER
        # ==============================================================
        # Menyimpan PointCloud2 terbaru. Setiap kali point cloud baru
        # datang dari kamera, buffer ini di-update.
        #
        # Ini PERSIS sama dengan yang dilakukan Pak Yani:
        #   def process_pc_msg(self, pc_msg):
        #       self.current_pc = pc_msg
        self.current_pc = None

        # ==============================================================
        # PARAMETER
        # ==============================================================
        # Topic point cloud — sesuaikan dengan output driver kamera.
        # Untuk ASUS Xtion dengan OpenNI2, biasanya salah satu dari:
        #   /camera/depth_registered/points (point cloud yang sudah
        #       ter-register dengan RGB, koordinat sesuai frame RGB)
        #   /camera/depth/points (point cloud dalam frame depth saja)
        #
        # Kita pakai depth_registered/points karena ini yang koordinat
        # pixelnya sesuai dengan output YOLO (yang bekerja di RGB).
        self.pc_topic = rospy.get_param(
            '~point_cloud_topic',
            '/camera/depth_registered/points'
        )

        # Frame name kamera depth — ini yang akan digunakan sebagai
        # source frame saat transformasi TF ke map.
        #
        # PERBEDAAN PENTING dengan kode Pak Yani:
        #   Pak Yani menggunakan 'head_rgbd_sensor_link' karena
        #   itu nama frame di robot HSR yang beliau pakai.
        #   Kita menggunakan frame dari ASUS Xtion yang ter-register
        #   dengan OpenNI2: 'camera_rgb_optical_frame'
        #
        # Point cloud dari /camera/depth_registered/points sudah
        # di-register ke frame RGB, sehingga koordinat 3D-nya
        # berada di frame 'camera_rgb_optical_frame' (bukan
        # 'camera_depth_optical_frame'). Ini penting karena kalau
        # frame-nya salah, transformasi ke map akan geser.
        self.camera_frame = rospy.get_param(
            '~camera_frame',
            'camera_rgb_optical_frame'
        )

        rospy.loginfo("=" * 60)
        rospy.loginfo("Coordinate Transformer - POINT CLOUD Version")
        rospy.loginfo("Menggunakan PointCloud2, bukan depth image")
        rospy.loginfo("Pendekatan dari kode asli Pak Yani (detect_ros_TF.py)")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Point cloud topic : %s", self.pc_topic)
        rospy.loginfo("Camera frame       : %s", self.camera_frame)
        rospy.loginfo("=" * 60)

        # ==============================================================
        # SUBSCRIBER 1: PointCloud2
        # ==============================================================
        # Subscribe ke point cloud topic.
        #
        # PENTING: buff_size harus BESAR karena satu pesan PointCloud2
        # berukuran ~9.8 MB. Kalau buffer terlalu kecil, ROS akan
        # menumpuk pesan lama dan callback menerima data yang sudah
        # basi (tidak sinkron dengan frame YOLO yang sedang diproses).
        # Dengan buff_size=2**26 (64 MB) dan queue_size=1, hanya
        # point cloud terbaru yang disimpan.
        rospy.Subscriber(
            self.pc_topic,
            PointCloud2,
            self.pc_callback,
            queue_size=1,
            buff_size=2**26  # 64 MB buffer untuk PointCloud2
        )

        # ==============================================================
        # SUBSCRIBER 2: YOLO detections (koordinat pixel centroid)
        # ==============================================================
        # Subscribe ke topic yang SAMA dengan coordinate_transformer.py
        # karena input-nya sama: centroid pixel dari yolo_hybrid.py.
        # Ini tidak menyebabkan konflik — di ROS, banyak node boleh
        # subscribe ke topic yang sama secara bersamaan.
        rospy.Subscriber(
            '/yolo/detections',
            Point,
            self.detection_callback
        )

        # ==============================================================
        # PUBLISHER: Koordinat map (TOPIC BERBEDA dari versi depth image)
        # ==============================================================
        # Publish ke /detected_positions_pc (bukan /detected_positions)
        # agar kedua pipeline bisa berjalan bersamaan tanpa saling
        # menimpa data.
        self.coord_pub = rospy.Publisher(
            '/detected_positions_pc',
            PointStamped,
            queue_size=10
        )

        rospy.loginfo("Subscribed to: %s", self.pc_topic)
        rospy.loginfo("Subscribed to: /yolo/detections")
        rospy.loginfo("Publishing to: /detected_positions_pc")
        rospy.loginfo("Waiting for point cloud data...")

    # ==================================================================
    # CALLBACK 1: Menerima dan menyimpan PointCloud2 terbaru
    # ==================================================================
    def pc_callback(self, msg):
        """
        Menyimpan point cloud terbaru ke buffer.

        Ini IDENTIK dengan kode Pak Yani:
            def process_pc_msg(self, pc_msg):
                self.current_pc = pc_msg

        Point cloud di-update terus-menerus oleh callback ini,
        dan detection_callback menggunakan point cloud terbaru
        yang tersedia saat deteksi YOLO masuk.
        """
        self.current_pc = msg

    # ==================================================================
    # CALLBACK 2: Proses deteksi YOLO → query point cloud → transform
    # ==================================================================
    def detection_callback(self, detection):
        """
        Callback utama — dipanggil setiap kali yolo_hybrid.py
        mempublish centroid pixel ke /yolo/detections.

        Alur:
        1. Ambil koordinat pixel (u, v) dari deteksi YOLO
        2. Query PointCloud2 untuk mendapatkan koordinat 3D
           di pixel tersebut (pendekatan Pak Yani)
        3. Transform dari camera frame ke map frame via TF
        4. Publish hasil ke /detected_positions_pc
        """

        # === CEK PRASYARAT ===
        if self.current_pc is None:
            rospy.logwarn_throttle(5.0,
                "Point cloud belum diterima dari %s", self.pc_topic)
            return

        # ===========================================================
        # STEP 1: Ambil koordinat pixel dari YOLO
        # ===========================================================
        # Format pesan Point dari yolo_hybrid.py:
        #   x = u_center (horizontal pixel)
        #   y = v_center (vertical pixel)
        #   z = confidence score YOLO
        u = int(detection.x)
        v = int(detection.y)
        confidence = detection.z

        rospy.loginfo("[PC] Deteksi YOLO: pixel (%d, %d) conf=%.2f",
                      u, v, confidence)

        # ===========================================================
        # STEP 2: Query PointCloud2 — PENDEKATAN PAK YANI
        # ===========================================================
        # Ini adalah bagian yang BERBEDA dari coordinate_transformer.py.
        #
        # Alih-alih:
        #   Z = depth_image[v, u] / 1000.0
        #   X = (u - cx) * Z / fx
        #   Y = (v - cy) * Z / fy
        #
        # Kita langsung query point cloud:
        #   pc2.read_points(cloud, uvs=[(u, v)])
        #   → langsung dapat (x, y, z) dalam frame kamera
        #
        # Ini PERSIS yang dilakukan Pak Yani di detect_ros_TF.py:
        #   points_list = list(pc2.read_points(self.current_pc,
        #                      skip_nans=True,
        #                      field_names=('x', 'y', 'z'),
        #                      uvs=[(x_center, y_center)]))
        #
        # Parameter penjelasan:
        #   self.current_pc = PointCloud2 message terbaru
        #   skip_nans=True  = abaikan titik yang tidak punya data depth
        #                     (NaN biasanya terjadi di area yang terlalu
        #                      dekat, terlalu jauh, atau permukaan
        #                      reflektif yang tidak terbaca sensor IR)
        #   field_names=('x','y','z') = kita hanya butuh koordinat 3D,
        #                     bukan warna RGB atau data lainnya
        #   uvs=[(u, v)]    = koordinat pixel yang ingin di-query;
        #                     fungsi ini akan mencari titik 3D yang
        #                     berkorespondensi dengan pixel (u, v)
        #                     di gambar
        try:
            points_list = list(pc2.read_points(
                self.current_pc,
                skip_nans=True,
                field_names=('x', 'y', 'z'),
                uvs=[(u, v)]
            ))
        except Exception as e:
            rospy.logerr("[PC] Gagal query point cloud: %s", str(e))
            return

        # Cek apakah query mengembalikan hasil
        # Kalau points_list kosong, berarti pixel (u, v) tidak punya
        # data 3D yang valid di point cloud (NaN atau out of range)
        if not points_list:
            rospy.logwarn("[PC] Tidak ada titik 3D valid di pixel (%d, %d)",
                          u, v)
            return

        # Ambil koordinat 3D — hanya ada satu titik karena kita
        # query satu pixel saja
        point_x, point_y, point_z = points_list[0]

        # Validasi depth range (sama seperti di coordinate_transformer.py)
        # ASUS Xtion: jangkauan efektif 0.8 - 3.5 meter
        if point_z < 0.1 or point_z > 5.0:
            rospy.logwarn("[PC] Depth di luar range: %.3f meter", point_z)
            return

        rospy.loginfo("[PC] Camera frame: X=%.3f, Y=%.3f, Z=%.3f (meter)",
                      point_x, point_y, point_z)

        # ===========================================================
        # STEP 3: Transform ke MAP frame menggunakan TF
        # ===========================================================
        # Buat PointStamped di camera frame.
        #
        # PERBEDAAN dengan kode Pak Yani:
        #   Pak Yani: point.header.frame_id = "head_rgbd_sensor_link"
        #   Kita:     point.header.frame_id = self.camera_frame
        #             (default: "camera_rgb_optical_frame")
        #
        # "head_rgbd_sensor_link" adalah nama frame di robot HSR
        # milik Pak Yani. Di robot kita dengan ASUS Xtion + OpenNI2,
        # point cloud dari /camera/depth_registered/points berada
        # di frame "camera_rgb_optical_frame" karena sudah ter-register
        # (aligned) dengan gambar RGB.
        #
        # PERBEDAAN API dengan kode Pak Yani:
        #   Pak Yani (tf1): self.tf_listener.transformPoint('map', point)
        #   Kita (tf2):     self.tf_buffer.transform(point, 'map')
        # Fungsionalitasnya identik — keduanya melakukan lookup
        # transform dari source frame ke target frame dan menerapkan
        # matriks transformasi ke koordinat titik.
        point_cam = PointStamped()
        point_cam.header.frame_id = self.camera_frame
        point_cam.header.stamp = rospy.Time(0)  # Transform terbaru
        point_cam.point.x = point_x
        point_cam.point.y = point_y
        point_cam.point.z = point_z

        try:
            # Transform ke map frame
            point_map = self.tf_buffer.transform(
                point_cam,
                "map",
                rospy.Duration(1.0)
            )

            rospy.loginfo("[PC] Map frame: X=%.3f, Y=%.3f, Z=%.3f",
                          point_map.point.x,
                          point_map.point.y,
                          point_map.point.z)

            # ===========================================================
            # STEP 4: Publish hasil transformasi
            # ===========================================================
            self.coord_pub.publish(point_map)

            rospy.loginfo("=" * 60)
            rospy.loginfo("[PC] TRANSFORMASI SELESAI:")
            rospy.loginfo("  Pixel     : (%d, %d)", u, v)
            rospy.loginfo("  Confidence: %.2f", confidence)
            rospy.loginfo("  Camera    : (%.3f, %.3f, %.3f)",
                          point_x, point_y, point_z)
            rospy.loginfo("  Map       : (%.3f, %.3f, %.3f)",
                          point_map.point.x,
                          point_map.point.y,
                          point_map.point.z)
            rospy.loginfo("=" * 60)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr("[PC] TF transform gagal: %s", str(e))
            rospy.logerr("[PC] Periksa TF tree: rosrun tf view_frames")


if __name__ == '__main__':
    try:
        transformer = CoordinateTransformerPointCloud()
        rospy.loginfo("[PC] Point Cloud Transformer ready. "
                      "Menunggu deteksi YOLO...")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
