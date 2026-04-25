#!/usr/bin/env python3
"""
Image Capture Node untuk Pengumpulan Dataset YOLO
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

Fungsi:
- Subscribe ke /camera/rgb/image_raw (ASUS Xtion)
- Simpan gambar setiap N detik ke folder yang ditentukan
- Tekan Ctrl+C untuk berhenti

Cara pakai:
1. Jalankan kamera:    roslaunch openni2_launch openni2.launch
2. Jalankan node ini:  python3 image_capture_node.py _save_dir:=/home/farhan/dataset_xtion/Pak_Yani _interval:=3.0
3. Arahkan kamera ke orang yang ingin di-capture
4. Tekan Ctrl+C kalau sudah cukup

Parameter ROS:
- ~save_dir  : folder tujuan simpan gambar (WAJIB diisi)
- ~interval  : jeda antar capture dalam detik (default: 3.0)
"""

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
from datetime import datetime


class ImageCaptureNode:
    def __init__(self):
        rospy.init_node('image_capture_node')
        
        # Parameter
        self.save_dir = rospy.get_param('~save_dir', '')
        self.interval = rospy.get_param('~interval', 3.0)
        
        # Validasi save_dir
        if not self.save_dir:
            rospy.logerr("Parameter ~save_dir HARUS diisi!")
            rospy.logerr("Contoh: _save_dir:=/home/farhan/dataset_xtion/Pak_Yani")
            rospy.signal_shutdown("save_dir not set")
            return
        
        # Buat folder kalau belum ada
        os.makedirs(self.save_dir, exist_ok=True)
        
        self.bridge = CvBridge()
        self.last_capture_time = rospy.Time.now()
        self.count = 0
        
        # Hitung gambar yang sudah ada di folder (untuk lanjutkan numbering)
        existing = [f for f in os.listdir(self.save_dir) if f.endswith('.jpg')]
        self.count = len(existing)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("IMAGE CAPTURE NODE - Dataset Collection")
        rospy.loginfo("Save directory : %s", self.save_dir)
        rospy.loginfo("Capture interval: %.1f detik", self.interval)
        rospy.loginfo("Existing images : %d", self.count)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Capturing... Tekan Ctrl+C untuk berhenti.")
        
        # Subscribe ke kamera
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback, queue_size=1)
    
    def callback(self, msg):
        # Cek interval waktu
        now = rospy.Time.now()
        elapsed = (now - self.last_capture_time).to_sec()
        
        if elapsed < self.interval:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Nama file: nomor urut + timestamp
            self.count += 1
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"img_{self.count:04d}_{timestamp}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            cv2.imwrite(filepath, cv_image)
            self.last_capture_time = now
            
            rospy.loginfo("[CAPTURED] #%d: %s", self.count, filename)
            
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: %s", str(e))


if __name__ == '__main__':
    try:
        node = ImageCaptureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    
    rospy.loginfo("Capture selesai. Total gambar: tersimpan di folder.")
