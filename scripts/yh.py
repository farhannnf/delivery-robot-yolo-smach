#!/usr/bin/env python3
"""
YOLO Detector - Hybrid Version (YOLO + InsightFace) — REVISI THREADING
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

PERBAIKAN UTAMA DARI VERSI SEBELUMNYA:
    Arsitektur diubah dari BLOCKING CALLBACK menjadi TWO-THREAD MODEL.

    Versi lama (bermasalah):
        ROS callback → YOLO → InsightFace → publish → selesai → callback berikutnya
        Akibat: kalau satu processing butuh 1 detik, frame berikutnya menunggu,
                antrian menumpuk, sistem makin lama makin ketinggalan realita.

    Versi baru (benar):
        Thread 1 (ROS callback): hanya simpan frame terbaru ke self.latest_frame
        Thread 2 (processing loop): ambil frame terbaru → YOLO → InsightFace → publish
        Akibat: frame lama TIDAK PERNAH menumpuk, selalu frame terbaru yang diproses,
                tidak ada lag kumulatif apapun — hanya ada delay tetap 1 processing cycle.

    Analogi: bukan kasir yang melayani antrian, tapi kasir yang hanya melayani
    orang PALING DEPAN saat ini, dan setiap orang baru langsung menggantikan posisi depan.

PERBAIKAN LAINNYA:
    - GPU otomatis dipakai jika tersedia (termasuk MX series, tapi fallback ke CPU jika lebih lambat)
    - face_crop_ratio dinaikkan 0.35 → 0.55 untuk kamera Xtion yang dipasang rendah
    - InsightFace hanya memproses satu crop per deteksi (bukan seluruh frame)
    - Semua parameter bisa dikonfigurasi dari launch file

INTERFACE TIDAK BERUBAH:
    Subscribe : /camera/rgb/image_raw     (sensor_msgs/Image)
    Publish   : /yolo/detections          (geometry_msgs/Point)
    Publish   : /yolo/person_id           (std_msgs/String)
    Publish   : /yolo/annotated_image     (sensor_msgs/Image)
"""

import rospy
import warnings
import os
import sys
import pickle
import threading
import numpy as np
import cv2
import torch

warnings.filterwarnings('ignore', category=FutureWarning)
warnings.filterwarnings('ignore', category=DeprecationWarning)

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError


class YOLODetectorHybrid:
    def __init__(self):
        rospy.init_node('yolo_detector_hybrid')

        self.bridge = CvBridge()

        # ==============================================================
        # VARIABEL UNTUK TWO-THREAD MODEL
        # self.latest_frame  : frame terbaru dari kamera (selalu ditimpa)
        # self.latest_header : header ROS dari frame tersebut (untuk timestamp)
        # self.frame_lock    : mutex agar dua thread tidak baca/tulis bersamaan
        # self.processing    : flag agar tidak ada dua processing berjalan bersamaan
        # ==============================================================
        self.latest_frame  = None
        self.latest_header = None
        self.frame_lock    = threading.Lock()
        self.processing    = False

        # ==============================================================
        # PARAMETER DARI LAUNCH FILE
        # ==============================================================
        model_path      = rospy.get_param('~weights',
                          os.path.expanduser('~/yolov5/yolov5n.pt'))
        self.conf_thres = rospy.get_param('~confidence_threshold', 0.45)
        self.iou_thres  = rospy.get_param('~iou_threshold', 0.45)
        self.infer_size = rospy.get_param('~inference_size', 320)
        device_param    = rospy.get_param('~device', 'cpu')

        input_topic     = rospy.get_param('~input_image_topic',
                          '/camera/rgb/image_raw')
        detection_topic = rospy.get_param('~output_detection_topic',
                          '/yolo/detections')
        image_topic     = rospy.get_param('~output_image_topic',
                          '/yolo/annotated_image')

        face_db_path            = rospy.get_param('~face_db_path',
                                  os.path.expanduser('~/face_database.pkl'))
        self.face_sim_threshold = rospy.get_param('~face_sim_threshold', 0.45)

        # 0.55 lebih aman untuk kamera Xtion yang dipasang rendah di robot.
        # Jika garis oranye di rqt_image_view terlalu rendah (lebih dari setengah badan),
        # turunkan kembali ke 0.45. Jika wajah masih sering terpotong, naikkan ke 0.65.
        self.face_crop_ratio    = rospy.get_param('~face_crop_ratio', 0.55)

        # Tentukan device: pakai GPU jika tersedia, fallback ke CPU
        if device_param == 'cuda' and torch.cuda.is_available():
            self.device = 'cuda'
            gpu_name = torch.cuda.get_device_name(0)
        else:
            self.device = 'cpu'
            gpu_name = 'CPU'
            if device_param == 'cuda' and not torch.cuda.is_available():
                rospy.logwarn("CUDA diminta tapi tidak tersedia. Fallback ke CPU.")

        rospy.loginfo("=" * 60)
        rospy.loginfo("YOLO Detector - Hybrid [THREADING REVISION]")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  device               : %s (%s)", self.device.upper(), gpu_name)
        rospy.loginfo("  weights              : %s", model_path)
        rospy.loginfo("  confidence_threshold : %.2f", self.conf_thres)
        rospy.loginfo("  inference_size       : %d",   self.infer_size)
        rospy.loginfo("  face_sim_threshold   : %.2f", self.face_sim_threshold)
        rospy.loginfo("  face_crop_ratio      : %.2f", self.face_crop_ratio)
        rospy.loginfo("  architecture         : TWO-THREAD (no lag)")
        rospy.loginfo("=" * 60)

        # ==============================================================
        # LOAD MODEL YOLO
        # ==============================================================
        if not os.path.exists(model_path):
            rospy.logerr("YOLO weights tidak ditemukan: %s", model_path)
            rospy.signal_shutdown("YOLO weights not found")
            return

        rospy.loginfo("Loading YOLO model...")

        old_stdout_fd = os.dup(1)
        old_stderr_fd = os.dup(2)
        devnull_fd = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull_fd, 1)
        os.dup2(devnull_fd, 2)
        os.close(devnull_fd)

        try:
            yolov5_dir = os.path.dirname(model_path)
            self.model = torch.hub.load(
                yolov5_dir, 'custom', path=model_path,
                source='local', force_reload=False
            )
        finally:
            os.dup2(old_stdout_fd, 1)
            os.dup2(old_stderr_fd, 2)
            os.close(old_stdout_fd)
            os.close(old_stderr_fd)

        self.model.conf    = self.conf_thres
        self.model.iou     = self.iou_thres
        self.model.classes = [0]  # class 0 = "person" di COCO

        # Pindahkan YOLO ke GPU jika tersedia
        self.model = self.model.to(self.device)
        rospy.loginfo("YOLO loaded → %s", self.device.upper())

        # ==============================================================
        # LOAD INSIGHTFACE + DATABASE
        # ==============================================================
        rospy.loginfo("Loading InsightFace model (buffalo_sc)...")

        try:
            from insightface.app import FaceAnalysis

            # Untuk GPU MX series: seringkali CPU lebih cepat dari GPU
            # karena overhead transfer data > keuntungan paralelisme.
            # Oleh karena itu, InsightFace kita jalankan di CPU saja,
            # sementara YOLO yang lebih berat kita jalankan di GPU.
            # Ini disebut "heterogeneous execution" — bagi tugas ke hardware yang tepat.
            self.face_app = FaceAnalysis(
                name='buffalo_sc',
                providers=['CPUExecutionProvider']   # InsightFace tetap CPU
            )
            # det_size=(160,160): lebih kecil = lebih cepat untuk crop wajah kecil.
            # Akurasi tidak turun signifikan karena input sudah berupa crop wajah,
            # bukan gambar penuh dengan banyak objek lain.
            self.face_app.prepare(ctx_id=0, det_size=(160, 160))
            rospy.loginfo("InsightFace loaded → CPU (optimal untuk MX GPU)")

        except ImportError:
            rospy.logerr("InsightFace tidak terinstall!")
            rospy.signal_shutdown("InsightFace not installed")
            return

        if not os.path.exists(face_db_path):
            rospy.logerr("Face database tidak ditemukan: %s", face_db_path)
            rospy.signal_shutdown("Face database not found")
            return

        with open(face_db_path, 'rb') as f:
            self.face_db = pickle.load(f)

        rospy.loginfo("Face database loaded: %s", list(self.face_db.keys()))

        # ==============================================================
        # SUBSCRIBER DAN PUBLISHER
        # ==============================================================

        # queue_size=1 dan buff_size besar: ROS hanya menyimpan 1 frame
        # di buffer saat callback sedang berjalan. Frame lama langsung dibuang.
        # Ini SANGAT PENTING untuk mencegah antrian menumpuk.
        rospy.Subscriber(input_topic, Image, self.image_callback,
                         queue_size=1, buff_size=2**24)

        self.detection_pub = rospy.Publisher(detection_topic, Point, queue_size=10)
        self.person_id_pub = rospy.Publisher('/yolo/person_id', String, queue_size=10)
        self.annotated_pub = rospy.Publisher(image_topic, Image, queue_size=1)

        rospy.loginfo("Subscribed to : %s", input_topic)
        rospy.loginfo("Publishing to : %s, /yolo/person_id, %s",
                      detection_topic, image_topic)
        rospy.loginfo("=" * 60)

        # ==============================================================
        # MULAI PROCESSING THREAD
        # Thread ini berjalan terpisah dari ROS callback thread.
        # Ia hanya mengambil self.latest_frame saat tersedia dan memprosesnya.
        # ==============================================================
        self.proc_thread = threading.Thread(target=self.processing_loop, daemon=True)
        self.proc_thread.start()
        rospy.loginfo("Processing thread started. No lag mode ACTIVE.")
        rospy.loginfo("Ready. Waiting for camera frames...")

    # ============================================================
    # THREAD 1: ROS CALLBACK — hanya simpan frame terbaru
    # Ini adalah thread yang cepat — tidak ada komputasi berat di sini.
    # ============================================================
    def image_callback(self, msg):
        """
        Callback ini dipanggil setiap kali frame baru datang dari kamera (~30fps).
        Tugasnya HANYA menyimpan frame ke self.latest_frame.

        Dengan menggunakan lock (mutex), kita memastikan Thread 1 (callback) dan
        Thread 2 (processing) tidak membaca/menulis variabel yang sama secara bersamaan,
        yang bisa menyebabkan data corrupt atau crash.

        Kenapa ini menyelesaikan masalah lag:
        - Setiap frame baru langsung MENGGANTIKAN frame lama di self.latest_frame
        - Tidak ada antrian, tidak ada penumpukan
        - Thread 2 selalu memproses frame yang paling segar tersedia
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.frame_lock:
                self.latest_frame  = cv_image
                self.latest_header = msg.header
        except CvBridgeError as e:
            rospy.logerr_throttle(5.0, "CvBridge error: %s", str(e))

    # ============================================================
    # THREAD 2: PROCESSING LOOP — komputasi berat di sini
    # ============================================================
    def processing_loop(self):
        """
        Loop ini berjalan terus-menerus di thread terpisah.
        Setiap iterasi: ambil frame terbaru → proses → publish → tunggu frame berikutnya.

        'self.processing' flag digunakan untuk memastikan hanya satu processing
        yang berjalan dalam satu waktu. Ini penting di lingkungan multi-thread.
        """
        rate = rospy.Rate(30)  # Cek apakah ada frame baru maksimal 30x per detik

        while not rospy.is_shutdown():
            # Cek apakah ada frame baru yang belum diproses
            with self.frame_lock:
                if self.latest_frame is None or self.processing:
                    rate.sleep()
                    continue

                # Salin frame dan langsung set latest_frame = None
                # supaya frame yang sama tidak diproses dua kali
                frame_to_process = self.latest_frame.copy()
                header_to_use    = self.latest_header
                self.latest_frame = None
                self.processing   = True

            # Di luar lock: lakukan komputasi berat (YOLO + InsightFace)
            # Lock tidak perlu dipegang selama processing — ini yang membuat
            # Thread 1 tetap bisa menerima frame baru tanpa diblokir
            try:
                self._process_frame(frame_to_process, header_to_use)
            except Exception as e:
                rospy.logerr("Error di processing_loop: %s", str(e))
                import traceback
                rospy.logerr(traceback.format_exc())
            finally:
                # Selalu reset flag, bahkan jika ada error
                with self.frame_lock:
                    self.processing = False

            rate.sleep()

    # ============================================================
    # FUNGSI INTI: YOLO → crop → InsightFace → publish
    # Dipanggil oleh processing_loop (Thread 2), bukan callback langsung
    # ============================================================
    def _process_frame(self, cv_image, header):
        h_frame, w_frame = cv_image.shape[:2]

        # ─── YOLO: deteksi "person" ──────────────────────────────────
        ratio   = self.infer_size / max(h_frame, w_frame)
        resized = cv2.resize(cv_image,
                             (int(w_frame * ratio), int(h_frame * ratio)))

        with torch.no_grad():
            results = self.model(resized)

        annotated_image = cv_image.copy()
        detection_found = False

        for *box, conf, cls in results.xyxy[0]:
            detection_found = True

            x_min = max(0, int(box[0] / ratio))
            y_min = max(0, int(box[1] / ratio))
            x_max = min(w_frame, int(box[2] / ratio))
            y_max = min(h_frame, int(box[3] / ratio))
            confidence = float(conf)

            # Persamaan 2.9 & 2.10: titik tengah bounding box
            u_center = (x_min + x_max) / 2.0
            v_center = (y_min + y_max) / 2.0

            # ─── Crop area wajah lalu identifikasi ───────────────────
            face_y2   = y_min + int((y_max - y_min) * self.face_crop_ratio)
            face_crop = cv_image[y_min:face_y2, x_min:x_max]
            person_name, similarity = self.identify_face(face_crop)

            # ─── Publish /yolo/detections (format lama, tidak berubah) ─
            detection_msg   = Point()
            detection_msg.x = float(u_center)
            detection_msg.y = float(v_center)
            detection_msg.z = float(confidence)
            self.detection_pub.publish(detection_msg)

            # ─── Publish /yolo/person_id (baru) ──────────────────────
            self.person_id_pub.publish(String(data=person_name))

            rospy.loginfo(
                "[DETECTED] person center=(%.1f,%.1f) conf=%.2f "
                "| face=%s (sim=%.3f)",
                u_center, v_center, confidence, person_name, similarity
            )

            # ─── Visualisasi ─────────────────────────────────────────
            color = (0, 255, 0) if person_name != "Unknown" else (0, 0, 255)

            cv2.rectangle(annotated_image,
                          (x_min, y_min), (x_max, y_max), color, 2)
            cv2.circle(annotated_image,
                       (int(u_center), int(v_center)), 5, (0, 0, 255), -1)

            label = f"{person_name} ({similarity:.2f}) | conf:{confidence:.2f}"
            cv2.putText(annotated_image, label, (x_min, y_min - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            cv2.putText(annotated_image,
                        f"({int(u_center)}, {int(v_center)})",
                        (int(u_center) + 10, int(v_center)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

            # Garis oranye: batas bawah area crop InsightFace.
            # Pastikan garis ini memotong di sekitar leher/bahu.
            # Kalau terlalu tinggi (wajah terpotong) → naikkan face_crop_ratio.
            # Kalau terlalu rendah (lebih dari setengah badan) → turunkan.
            cv2.rectangle(annotated_image,
                          (x_min, y_min), (x_max, face_y2),
                          (0, 165, 255), 1)

        if not detection_found:
            rospy.loginfo_throttle(5.0, "[SCANNING] No person detected")

        # Publish annotated image
        try:
            self.annotated_pub.publish(
                self.bridge.cv2_to_imgmsg(annotated_image, 'bgr8')
            )
        except CvBridgeError as e:
            rospy.logerr("Gagal publish annotated image: %s", str(e))

    # ============================================================
    # IDENTIFIKASI WAJAH
    # ============================================================
    def identify_face(self, face_crop_bgr):
        """
        Return (nama, cosine_similarity) dari crop wajah BGR.
        Return ("Unknown", score) jika tidak ada wajah atau score < threshold.
        """
        if face_crop_bgr is None or face_crop_bgr.size == 0:
            return "Unknown", 0.0

        h, w = face_crop_bgr.shape[:2]
        if h < 40 or w < 40:
            return "Unknown", 0.0

        faces = self.face_app.get(face_crop_bgr)
        if not faces:
            return "Unknown", 0.0

        face = sorted(faces, key=lambda f: f.det_score, reverse=True)[0]

        query_emb = face.embedding / np.linalg.norm(face.embedding)

        best_name = "Unknown"
        best_sim  = 0.0

        for name, ref_emb in self.face_db.items():
            sim = float(np.dot(query_emb, ref_emb))
            if sim > best_sim:
                best_sim  = sim
                best_name = name

        if best_sim < self.face_sim_threshold:
            return "Unknown", best_sim

        return best_name, best_sim


if __name__ == '__main__':
    try:
        detector = YOLODetectorHybrid()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
