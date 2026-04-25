#!/usr/bin/env python3
"""
scanning_mode.py — REVISI: Observasi Dulu, Baru Simpan
========================================================
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

PERUBAHAN DARI VERSI SEBELUMNYA (Meeting 13 April 2026):
  Versi lama:
    Robot mendeteksi orang → langsung simpan ke database.
    Masalah: false detection, posisi belum stabil (goyang-goyang),
    robot masih jauh tapi sudah capture.

  Versi baru (arahan Pak Yani, referensi disertasi beliau):
    Robot mendeteksi orang → mulai periode observasi (5 detik)
    → kumpulkan data hit (berapa kali terdeteksi, posisi setiap hit)
    → setelah observasi selesai, cek apakah jumlah hit cukup
    → kalau cukup, simpan MEDIAN posisi ke database
    → kalau tidak cukup (false detection), abaikan

  Konsep dari paper "Online Object-Oriented Semantic Mapping":
    "Our system counts the number of times an object was
    initialized or updated (hit) or not (miss). The higher
    the likelihood, the higher the confidence that the object
    really exists."

PARAMETER YANG BISA DIATUR:
  observation_duration : berapa detik robot mengobservasi (default: 5)
  min_hits             : minimal berapa kali terdeteksi (default: 3)

SUBSCRIBE:
  /detected_positions  (PointStamped) — koordinat manusia di map
  /yolo/person_id      (String)       — nama orang dari InsightFace

PUBLISH:
  /scanning/status     (String)       — status scanning untuk monitoring
"""

import rospy
import yaml
import os
import threading
import math
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String


class ScanningMode:
    def __init__(self):
        rospy.init_node('scanning_mode')

        # ==============================================================
        # PATH KE DATABASE YAML
        # ==============================================================
        self.yaml_path = rospy.get_param(
            '~database_path',
            os.path.expanduser(
                '~/catkin_ws/src/ta2_farhan/config/position_database.yaml'
            )
        )

        # ==============================================================
        # PARAMETER OBSERVASI (Arahan Pak Yani)
        # ==============================================================
        # observation_duration: berapa detik robot mengobservasi orang
        # sebelum memutuskan apakah deteksi valid atau tidak.
        # Pak Yani: "dia harus diobservasi beberapa detik, baru dia keluar"
        self.observation_duration = rospy.get_param(
            '~observation_duration', 8.0)

        # min_hits: minimal berapa kali orang terdeteksi selama
        # periode observasi agar dianggap valid.
        # Pak Yani: "ada threshold-nya"
        self.min_hits = rospy.get_param('~min_hits', 5)

        # ==============================================================
        # TF BUFFER UNTUK LOOKUP POSISI ROBOT
        # ==============================================================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ==============================================================
        # LOGIKA DETECT-ONCE (Arahan Pak Yani, 26 Feb 2026)
        # ==============================================================
        self.detected_persons = set()

        # ==============================================================
        # BUFFER NAMA TERBARU (Temporal Pairing)
        # ==============================================================
        self.latest_person_name = "Unknown"
        self.latest_person_stamp = rospy.Time(0)
        self.name_max_age = rospy.Duration(2.0)

        self.lock = threading.Lock()

        # ==============================================================
        # OBSERVATION TRACKER
        # ==============================================================
        # Dictionary yang menyimpan data observasi untuk setiap orang
        # yang sedang dalam periode observasi.
        # Format: {
        #   "farhan": {
        #     "start_time": rospy.Time,
        #     "hits": int,
        #     "robot_positions": [(x, y, theta), ...],
        #     "person_positions": [(px, py), ...],
        #     "display_name": "Farhan"
        #   }
        # }
        self.observations = {}

        # ==============================================================
        # INISIALISASI: BACA DATABASE YANG SUDAH ADA
        # ==============================================================
        self._load_existing_names()

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
        self.status_pub = rospy.Publisher(
            '/scanning/status', String, queue_size=10)

        # ==============================================================
        # TIMER: Cek apakah ada observasi yang sudah selesai
        # ==============================================================
        # Setiap 0.5 detik, timer ini mengecek apakah ada orang
        # yang periode observasinya sudah habis. Kalau sudah,
        # evaluasi apakah hit cukup, lalu simpan atau abaikan.
        rospy.Timer(rospy.Duration(0.5), self._check_observations)

        # ==============================================================
        # LOG STARTUP
        # ==============================================================
        rospy.loginfo("=" * 60)
        rospy.loginfo("SCANNING MODE — Observation-Based Database Builder")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Database path       : %s", self.yaml_path)
        rospy.loginfo("Observation duration : %.1f detik", self.observation_duration)
        rospy.loginfo("Min hits required    : %d", self.min_hits)
        rospy.loginfo("Already in DB        : %s",
                       ", ".join(self.detected_persons) if self.detected_persons
                       else "(kosong)")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Siap menerima deteksi. Arahkan robot ke depan orang.")

    # ==================================================================
    # INISIALISASI: Baca nama-nama yang sudah ada di database YAML
    # ==================================================================
    def _load_existing_names(self):
        if not os.path.exists(self.yaml_path):
            rospy.logwarn("Database belum ada: %s", self.yaml_path)
            return

        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)

            if data and 'positions' in data and data['positions']:
                for entry in data['positions']:
                    name = entry.get('name', '')
                    if name and name.lower() != 'origin':
                        self.detected_persons.add(name.lower())
                        rospy.loginfo("  Sudah di DB: %s (%.2f, %.2f)",
                                      name, entry.get('x', 0),
                                      entry.get('y', 0))
        except Exception as e:
            rospy.logwarn("Gagal baca database existing: %s", str(e))

    # ==================================================================
    # LOOKUP POSISI ROBOT SAAT INI DARI TF
    # ==================================================================
    def _get_robot_position(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rospy.Time(0), rospy.Duration(1.0))

            robot_x = trans.transform.translation.x
            robot_y = trans.transform.translation.y

            qz = trans.transform.rotation.z
            qw = trans.transform.rotation.w
            theta = 2.0 * math.atan2(qz, qw)

            return (robot_x, robot_y, theta)

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Gagal lookup posisi robot: %s", str(e))
            return None

    # ==================================================================
    # CALLBACK 1: Menerima nama orang dari InsightFace
    # ==================================================================
    def person_id_callback(self, msg):
        with self.lock:
            self.latest_person_name = msg.data.strip()
            self.latest_person_stamp = rospy.Time.now()

    # ==================================================================
    # CALLBACK 2: Menerima koordinat map — TRIGGER OBSERVASI
    # ==================================================================
    def position_callback(self, msg):
        with self.lock:
            person_name = self.latest_person_name
            name_stamp = self.latest_person_stamp

        # VALIDASI 1: Abaikan Unknown
        if person_name.lower() == "unknown" or person_name == "":
            return

        # VALIDASI 2: Cek freshness nama
        age = rospy.Time.now() - name_stamp
        if age > self.name_max_age:
            return

        # VALIDASI 3: Sudah ada di database (detect-once)
        if person_name.lower() in self.detected_persons:
            rospy.loginfo_throttle(3.0,
                "[SCANNING] '%s' sudah ada di database — skip.",
                person_name)
            return

        # VALIDASI 4: Sudah tersimpan (selesai observasi)
        # Cek lagi setelah lock untuk menghindari race condition
        if person_name.lower() in self.detected_persons:
            return

        # ===========================================================
        # LOOKUP POSISI ROBOT
        # ===========================================================
        robot_pos = self._get_robot_position()
        if robot_pos is None:
            return

        robot_x, robot_y, robot_theta = robot_pos
        person_x = msg.point.x
        person_y = msg.point.y

        key = person_name.lower()

        # ===========================================================
        # LOGIKA OBSERVASI (PERUBAHAN UTAMA)
        # ===========================================================
        # Kalau orang ini BELUM dalam observasi, mulai observasi baru.
        # Kalau SUDAH dalam observasi, tambahkan hit.
        with self.lock:
            if key not in self.observations:
                # Mulai observasi baru
                self.observations[key] = {
                    'start_time': rospy.Time.now(),
                    'hits': 1,
                    'robot_positions': [(robot_x, robot_y, robot_theta)],
                    'person_positions': [(person_x, person_y)],
                    'display_name': person_name
                }
                rospy.loginfo("[OBSERVING] Mulai observasi '%s' "
                              "(%.1f detik, min %d hits)...",
                              person_name, self.observation_duration,
                              self.min_hits)
            else:
                # Tambahkan hit ke observasi yang sedang berjalan
                obs = self.observations[key]
                obs['hits'] += 1
                obs['robot_positions'].append(
                    (robot_x, robot_y, robot_theta))
                obs['person_positions'].append(
                    (person_x, person_y))

                elapsed = (rospy.Time.now() - obs['start_time']).to_sec()
                rospy.loginfo_throttle(1.0,
                    "[OBSERVING] '%s' hit=%d (elapsed=%.1f/%.1f detik)",
                    person_name, obs['hits'], elapsed,
                    self.observation_duration)

    # ==================================================================
    # TIMER CALLBACK: Cek apakah ada observasi yang sudah selesai
    # ==================================================================
    def _check_observations(self, event):
        now = rospy.Time.now()
        finished = []

        with self.lock:
            for key, obs in self.observations.items():
                elapsed = (now - obs['start_time']).to_sec()
                if elapsed >= self.observation_duration:
                    finished.append((key, obs))

            # Hapus observasi yang sudah selesai dari tracker
            for key, _ in finished:
                del self.observations[key]

        # Proses setiap observasi yang selesai (di luar lock)
        for key, obs in finished:
            self._finalize_observation(key, obs)

    # ==================================================================
    # FINALISASI OBSERVASI: Evaluasi hit count, hitung median, simpan
    # ==================================================================
    def _finalize_observation(self, key, obs):
        name = obs['display_name']
        hits = obs['hits']

        rospy.loginfo("=" * 60)
        rospy.loginfo("[EVALUATE] Observasi '%s' selesai.", name)
        rospy.loginfo("  Total hits: %d (minimum: %d)", hits, self.min_hits)

        # Cek apakah hit cukup
        if hits < self.min_hits:
            rospy.logwarn("  DITOLAK: hits tidak cukup "
                          "(%d < %d) — kemungkinan false detection.",
                          hits, self.min_hits)
            rospy.logwarn("  '%s' TIDAK disimpan ke database.", name)
            rospy.loginfo("=" * 60)

            status_msg = String()
            status_msg.data = "REJECTED: %s (hits=%d < min=%d)" % (
                name, hits, self.min_hits)
            self.status_pub.publish(status_msg)
            return

        # ===========================================================
        # HITUNG MEDIAN POSISI
        # ===========================================================
        # Menggunakan median dari semua posisi yang dikumpulkan
        # selama observasi. Median lebih robust terhadap outlier
        # dibanding mean — kalau ada 1-2 deteksi yang posisinya
        # jauh (false detection sesaat), median tidak terpengaruh.
        #
        # Pak Yani: "confidence-nya bisa jadi yang lebih lama di sini,
        # karena intersection-nya mungkin lebih dominant yang ada di sini"
        robot_positions = np.array(obs['robot_positions'])
        person_positions = np.array(obs['person_positions'])

        median_robot_x = float(np.median(robot_positions[:, 0]))
        median_robot_y = float(np.median(robot_positions[:, 1]))
        median_robot_theta = float(np.median(robot_positions[:, 2]))
        median_person_x = float(np.median(person_positions[:, 0]))
        median_person_y = float(np.median(person_positions[:, 1]))

        # Hitung jarak robot-manusia
        distance = math.sqrt(
            (median_robot_x - median_person_x)**2 +
            (median_robot_y - median_person_y)**2
        )

        # ===========================================================
        # SIMPAN KE DATABASE
        # ===========================================================
        new_entry = {
            'name': name,
            'x': round(median_robot_x, 4),
            'y': round(median_robot_y, 4),
            'theta': round(median_robot_theta, 4),
            'person_x': round(median_person_x, 4),
            'person_y': round(median_person_y, 4)
        }

        success = self._save_to_yaml(new_entry)

        if success:
            self.detected_persons.add(key)

            rospy.loginfo("  DITERIMA: '%s' valid (%d hits).", name, hits)
            rospy.loginfo("  Posisi ROBOT (median)   : (%.4f, %.4f) "
                          "theta=%.4f",
                          median_robot_x, median_robot_y,
                          median_robot_theta)
            rospy.loginfo("  Posisi MANUSIA (median) : (%.4f, %.4f)",
                          median_person_x, median_person_y)
            rospy.loginfo("  Jarak robot<->manusia   : %.3f meter", distance)
            rospy.loginfo("  Total di DB             : %d orang",
                          len(self.detected_persons))
            rospy.loginfo("=" * 60)

            status_msg = String()
            status_msg.data = ("SAVED: %s | hits=%d | robot:(%.2f,%.2f) | "
                               "person:(%.2f,%.2f) | dist:%.2fm" % (
                                name, hits, median_robot_x, median_robot_y,
                                median_person_x, median_person_y, distance))
            self.status_pub.publish(status_msg)
        else:
            rospy.logerr("  GAGAL menyimpan '%s' ke database!", name)
            rospy.loginfo("=" * 60)

    # ==================================================================
    # FUNGSI PENYIMPANAN KE YAML
    # ==================================================================
    def _save_to_yaml(self, new_entry):
        try:
            existing_data = {'positions': []}
            if os.path.exists(self.yaml_path):
                with open(self.yaml_path, 'r') as f:
                    loaded = yaml.safe_load(f)
                    if (loaded and 'positions' in loaded and
                            isinstance(loaded['positions'], list)):
                        existing_data = loaded
                    else:
                        existing_data = {'positions': []}

            # Cek duplikasi
            for entry in existing_data['positions']:
                if entry.get('name', '').lower() == new_entry['name'].lower():
                    rospy.logwarn("Duplikasi di YAML: %s — skip.",
                                 new_entry['name'])
                    return False

            existing_data['positions'].append(new_entry)

            yaml_dir = os.path.dirname(self.yaml_path)
            if yaml_dir and not os.path.exists(yaml_dir):
                os.makedirs(yaml_dir)

            with open(self.yaml_path, 'w') as f:
                yaml.dump(
                    existing_data, f,
                    default_flow_style=False,
                    allow_unicode=True,
                    sort_keys=False
                )

            return True

        except Exception as e:
            rospy.logerr("Error menulis ke YAML: %s", str(e))
            return False


if __name__ == '__main__':
    try:
        scanner = ScanningMode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Scanning mode dihentikan.")
    except Exception as e:
        rospy.logerr("Fatal error: %s", str(e))
