#!/usr/bin/env python3
"""
live_person_marker.py — Visualisasi Real-Time Orang di Map (RViz)
=================================================================
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

Requirement dari Pak Yani (meeting terakhir):
  "Kalau ada orang di situ, langsung orangnya muncul di map."

Node ini menampilkan marker real-time di RViz setiap kali YOLO
mendeteksi orang dan coordinate_transformer menghitung posisinya
di map. Marker muncul langsung saat deteksi terjadi dan hilang
otomatis setelah 2 detik jika tidak ada deteksi baru.

PERBEDAAN DENGAN db_visualizer.py:
  db_visualizer.py → marker PERMANEN berdasarkan position_database.yaml
                     (untuk referensi posisi yang sudah tersimpan)
  live_person_marker.py → marker SEMENTARA berdasarkan deteksi real-time
                          (untuk menunjukkan "ada orang di sini sekarang")

KEDUA NODE BISA BERJALAN BERSAMAAN tanpa konflik karena:
  - Publish ke topic berbeda (/live_person_markers vs /database_markers)
  - Menggunakan namespace marker berbeda ("live_person" vs "person_body")
  - Warna marker berbeda (hijau untuk live, biru untuk database)

SUBSCRIBE:
  /detected_positions  (PointStamped) — koordinat manusia di map
  /yolo/person_id      (String)       — nama orang dari InsightFace

PUBLISH:
  /live_person_markers (MarkerArray)  — marker real-time untuk RViz

CARA MELIHAT DI RVIZ:
  Add → By topic → /live_person_markers → MarkerArray → OK

CARA MENJALANKAN:
  rosrun ta2_farhan live_person_marker.py
"""

import rospy
import threading
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class LivePersonMarker:
    def __init__(self):
        rospy.init_node('live_person_marker')

        # ==============================================================
        # BUFFER NAMA TERBARU (Temporal Pairing)
        # ==============================================================
        # Mekanisme yang sama dengan scanning_mode.py:
        # nama dari /yolo/person_id disimpan di buffer,
        # dan digunakan saat /detected_positions masuk.
        self.latest_person_name = "Unknown"
        self.latest_person_stamp = rospy.Time(0)
        self.name_max_age = rospy.Duration(2.0)
        self.lock = threading.Lock()

        # ==============================================================
        # PARAMETER
        # ==============================================================
        # Lifetime marker dalam detik. Setelah waktu ini, marker
        # otomatis hilang dari RViz jika tidak di-refresh.
        # 2 detik cukup untuk memberikan kesan "real-time" tanpa
        # marker yang menumpuk saat tidak ada deteksi.
        self.marker_lifetime = rospy.get_param('~marker_lifetime', 2.0)

        # Topic output — BERBEDA dari db_visualizer.py
        marker_topic = rospy.get_param('~marker_topic',
                                        '/live_person_markers')

        # ==============================================================
        # SUBSCRIBER
        # ==============================================================
        rospy.Subscriber('/yolo/person_id', String,
                         self.person_id_callback)
        rospy.Subscriber('/detected_positions', PointStamped,
                         self.position_callback)

        # ==============================================================
        # PUBLISHER
        # ==============================================================
        self.marker_pub = rospy.Publisher(
            marker_topic, MarkerArray, queue_size=10)

        # Counter untuk marker ID unik
        self.marker_id_counter = 0

        rospy.loginfo("=" * 60)
        rospy.loginfo("Live Person Marker — Real-Time Visualization")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  Marker lifetime  : %.1f detik", self.marker_lifetime)
        rospy.loginfo("  Publish topic    : %s", marker_topic)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Di RViz: Add -> MarkerArray -> topic: %s",
                       marker_topic)
        rospy.loginfo("Menunggu deteksi dari YOLO + coordinate_transformer...")

    # ==================================================================
    # CALLBACK 1: Menerima nama orang dari InsightFace
    # ==================================================================
    def person_id_callback(self, msg):
        with self.lock:
            self.latest_person_name = msg.data.strip()
            self.latest_person_stamp = rospy.Time.now()

    # ==================================================================
    # CALLBACK 2: Menerima koordinat map — langsung publish marker
    # ==================================================================
    def position_callback(self, msg):
        # Ambil nama dari buffer
        with self.lock:
            person_name = self.latest_person_name
            name_stamp = self.latest_person_stamp

        # Validasi: abaikan Unknown
        if person_name.lower() == "unknown" or person_name == "":
            return

        # Validasi: cek freshness nama
        age = rospy.Time.now() - name_stamp
        if age > self.name_max_age:
            return

        # Koordinat orang di map
        px = msg.point.x
        py = msg.point.y

        # Buat dan publish marker langsung
        marker_array = self._create_person_markers(person_name, px, py)
        self.marker_pub.publish(marker_array)

        rospy.loginfo_throttle(2.0,
            "[LIVE] %s terdeteksi di map (%.2f, %.2f)", person_name, px, py)

    # ==================================================================
    # BUAT MARKER UNTUK SATU ORANG
    # ==================================================================
    def _create_person_markers(self, name, x, y):
        """
        Membuat 3 marker untuk satu orang (sama seperti db_visualizer):
          1. CYLINDER  → badan/torso (warna hijau, bukan biru)
          2. SPHERE    → kepala
          3. TEXT      → nama orang

        Perbedaan dengan db_visualizer.py:
          - Warna HIJAU (bukan biru) agar bisa dibedakan secara visual
          - Lifetime TERBATAS (2 detik, bukan permanen)
          - Marker ID menggunakan counter yang terus naik agar
            marker lama otomatis digantikan oleh marker baru
            tanpa penumpukan

        Lifetime 2 detik berarti: kalau YOLO berhenti mendeteksi
        orang tersebut (misalnya orang pergi atau robot berputar
        ke arah lain), marker hilang sendiri setelah 2 detik.
        Kalau YOLO terus mendeteksi, callback terus dipanggil
        dan marker terus di-refresh sehingga tetap terlihat.
        """
        marker_array = MarkerArray()
        now = rospy.Time.now()
        lifetime = rospy.Duration(self.marker_lifetime)

        # Gunakan hash nama sebagai base ID agar marker untuk
        # orang yang sama selalu menimpa marker sebelumnya
        # (bukan membuat marker baru yang menumpuk)
        base_id = hash(name) % 10000

        # --- MARKER 1: Badan (CYLINDER) ---
        body = Marker()
        body.header.frame_id = "map"
        body.header.stamp = now
        body.ns = "live_person_body"
        body.id = base_id * 3
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.05
        body.pose.orientation.w = 1.0
        body.scale.x = 0.5    # diameter badan
        body.scale.y = 0.5
        body.scale.z = 0.05   # tipis (terlihat seperti lingkaran)
        # Warna HIJAU semi-transparan — berbeda dari biru db_visualizer
        body.color.r = 0.0
        body.color.g = 0.9
        body.color.b = 0.2
        body.color.a = 0.7
        body.lifetime = lifetime
        marker_array.markers.append(body)

        # --- MARKER 2: Kepala (SPHERE) ---
        head = Marker()
        head.header.frame_id = "map"
        head.header.stamp = now
        head.ns = "live_person_head"
        head.id = base_id * 3 + 1
        head.type = Marker.SPHERE
        head.action = Marker.ADD
        head.pose.position.x = x
        head.pose.position.y = y
        head.pose.position.z = 0.3
        head.pose.orientation.w = 1.0
        head.scale.x = 0.25   # diameter kepala
        head.scale.y = 0.25
        head.scale.z = 0.25
        # Warna hijau tua
        head.color.r = 0.0
        head.color.g = 0.7
        head.color.b = 0.1
        head.color.a = 0.9
        head.lifetime = lifetime
        marker_array.markers.append(head)

        # --- MARKER 3: Nama (TEXT) ---
        text = Marker()
        text.header.frame_id = "map"
        text.header.stamp = now
        text.ns = "live_person_name"
        text.id = base_id * 3 + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = x
        text.pose.position.y = y
        text.pose.position.z = 0.7   # mengambang di atas kepala
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2    # ukuran teks sedikit lebih besar
        # Warna kuning agar kontras dan mudah terbaca
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.text = name
        text.lifetime = lifetime
        marker_array.markers.append(text)

        return marker_array


if __name__ == '__main__':
    try:
        node = LivePersonMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
