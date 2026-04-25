#!/usr/bin/env python3
"""
db_visualizer.py — Hybrid Database Visualizer (Robot + Person Position)
========================================================================
Tugas Akhir 2 - Farhan Firmansyah (1102220025)

PERUBAHAN DARI VERSI SEBELUMNYA:
  Versi lama: Hanya menampilkan posisi ROBOT (field x, y) sebagai marker.
  Versi baru: Menampilkan DUA posisi per entry database:
    1. Posisi ROBOT (x, y) — ditampilkan sebagai ARROW berwarna BIRU
       menunjukkan dimana robot berdiri saat scanning dan kemana
       state machine akan mengirim robot saat delivery.
    2. Posisi PERSON (person_x, person_y) — ditampilkan sebagai
       siluet manusia (kepala + badan + kaki) berwarna HIJAU
       menunjukkan dimana orang tersebut berada di map.

  Entry Origin hanya memiliki posisi robot (tidak ada person_x/y),
  jadi Origin hanya ditampilkan sebagai ARROW saja.

SUBSCRIBE: (tidak ada — membaca langsung dari YAML)
PUBLISH:   /database_markers (MarkerArray)

CARA MENJALANKAN:
  rosrun ta2_farhan db_visualizer.py
"""

import rospy
import yaml
import os
import math
from visualization_msgs.msg import Marker, MarkerArray


class DatabaseVisualizer:
    def __init__(self):
        rospy.init_node('database_visualizer')

        self.yaml_path = rospy.get_param(
            '~database_path',
            os.path.expanduser(
                '~/catkin_ws/src/ta2_farhan/config/position_database.yaml'
            )
        )

        self.marker_pub = rospy.Publisher(
            '/database_markers', MarkerArray, queue_size=10)

        self.rate = rospy.get_param('~update_rate', 1.0)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Database Visualizer — Hybrid (Robot + Person)")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Database path : %s", self.yaml_path)
        rospy.loginfo("Topic         : /database_markers")
        rospy.loginfo("Rate          : %.1f Hz", self.rate)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Di RViz: Add -> MarkerArray -> topic: /database_markers")

    def run(self):
        rate = rospy.Rate(self.rate)

        while not rospy.is_shutdown():
            entries = self._read_database()
            if entries is not None:
                marker_array = self._create_all_markers(entries)
                self.marker_pub.publish(marker_array)
                rospy.loginfo_throttle(10.0,
                    "Database loaded: %d entries", len(entries))
            else:
                rospy.logwarn_throttle(5.0,
                    "Database kosong atau gagal dibaca.")
            rate.sleep()

    def _read_database(self):
        if not os.path.exists(self.yaml_path):
            return None
        try:
            with open(self.yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            if data and 'positions' in data and isinstance(data['positions'], list):
                return data['positions']
            return None
        except Exception as e:
            rospy.logerr_throttle(5.0,
                "Gagal membaca database: %s", str(e))
            return None

    def _create_all_markers(self, entries):
        marker_array = MarkerArray()
        now = rospy.Time.now()

        for idx, entry in enumerate(entries):
            name = entry.get('name', '')
            if not name:
                continue

            robot_x = entry.get('x', 0)
            robot_y = entry.get('y', 0)
            robot_theta = entry.get('theta', 0)
            person_x = entry.get('person_x', None)
            person_y = entry.get('person_y', None)

            is_origin = (name.lower() == 'origin')

            # ============================================
            # MARKER POSISI ROBOT — ARROW biru
            # ============================================
            # Arrow menunjukkan posisi dan orientasi robot
            # saat scanning. Ini juga adalah goal navigasi
            # yang akan digunakan oleh state machine.
            arrow = Marker()
            arrow.header.frame_id = "map"
            arrow.header.stamp = now
            arrow.ns = "robot_position"
            arrow.id = idx * 10
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = robot_x
            arrow.pose.position.y = robot_y
            arrow.pose.position.z = 0.05
            arrow.pose.orientation.z = math.sin(robot_theta / 2.0)
            arrow.pose.orientation.w = math.cos(robot_theta / 2.0)
            arrow.scale.x = 0.4   # panjang arrow
            arrow.scale.y = 0.08  # lebar shaft
            arrow.scale.z = 0.08  # tinggi shaft

            if is_origin:
                # Origin: warna merah
                arrow.color.r = 1.0
                arrow.color.g = 0.0
                arrow.color.b = 0.0
                arrow.color.a = 1.0
            else:
                # Robot position: warna biru
                arrow.color.r = 0.2
                arrow.color.g = 0.4
                arrow.color.b = 1.0
                arrow.color.a = 0.8

            marker_array.markers.append(arrow)

            # Label untuk robot position
            robot_label = Marker()
            robot_label.header.frame_id = "map"
            robot_label.header.stamp = now
            robot_label.ns = "robot_label"
            robot_label.id = idx * 10 + 1
            robot_label.type = Marker.TEXT_VIEW_FACING
            robot_label.action = Marker.ADD
            robot_label.pose.position.x = robot_x
            robot_label.pose.position.y = robot_y
            robot_label.pose.position.z = 0.3
            robot_label.pose.orientation.w = 1.0
            robot_label.scale.z = 0.15

            if is_origin:
                robot_label.text = "Origin"
                robot_label.color.r = 1.0
                robot_label.color.g = 0.3
                robot_label.color.b = 0.3
            else:
                robot_label.text = "[R] %s" % name
                robot_label.color.r = 0.4
                robot_label.color.g = 0.6
                robot_label.color.b = 1.0

            robot_label.color.a = 1.0
            marker_array.markers.append(robot_label)

            # ============================================
            # MARKER POSISI PERSON — siluet manusia hijau
            # ============================================
            # Hanya ditampilkan kalau person_x dan person_y ada
            # (Origin tidak punya field ini)
            if person_x is not None and person_y is not None:
                # --- Kepala (SPHERE) ---
                head = Marker()
                head.header.frame_id = "map"
                head.header.stamp = now
                head.ns = "person_head"
                head.id = idx * 10 + 2
                head.type = Marker.SPHERE
                head.action = Marker.ADD
                head.pose.position.x = person_x
                head.pose.position.y = person_y
                head.pose.position.z = 0.55
                head.pose.orientation.w = 1.0
                head.scale.x = 0.2
                head.scale.y = 0.2
                head.scale.z = 0.2
                head.color.r = 0.1
                head.color.g = 0.9
                head.color.b = 0.2
                head.color.a = 0.9
                marker_array.markers.append(head)

                # --- Leher (CYLINDER kecil) ---
                neck = Marker()
                neck.header.frame_id = "map"
                neck.header.stamp = now
                neck.ns = "person_neck"
                neck.id = idx * 10 + 3
                neck.type = Marker.CYLINDER
                neck.action = Marker.ADD
                neck.pose.position.x = person_x
                neck.pose.position.y = person_y
                neck.pose.position.z = 0.42
                neck.pose.orientation.w = 1.0
                neck.scale.x = 0.06
                neck.scale.y = 0.06
                neck.scale.z = 0.06
                neck.color.r = 0.1
                neck.color.g = 0.8
                neck.color.b = 0.2
                neck.color.a = 0.9
                marker_array.markers.append(neck)

                # --- Badan (CYLINDER) ---
                body = Marker()
                body.header.frame_id = "map"
                body.header.stamp = now
                body.ns = "person_body"
                body.id = idx * 10 + 4
                body.type = Marker.CYLINDER
                body.action = Marker.ADD
                body.pose.position.x = person_x
                body.pose.position.y = person_y
                body.pose.position.z = 0.25
                body.pose.orientation.w = 1.0
                body.scale.x = 0.25
                body.scale.y = 0.15
                body.scale.z = 0.3
                body.color.r = 0.1
                body.color.g = 0.85
                body.color.b = 0.2
                body.color.a = 0.85
                marker_array.markers.append(body)

                # --- Kaki kiri (CYLINDER) ---
                leg_l = Marker()
                leg_l.header.frame_id = "map"
                leg_l.header.stamp = now
                leg_l.ns = "person_leg_l"
                leg_l.id = idx * 10 + 5
                leg_l.type = Marker.CYLINDER
                leg_l.action = Marker.ADD
                leg_l.pose.position.x = person_x - 0.05
                leg_l.pose.position.y = person_y
                leg_l.pose.position.z = 0.05
                leg_l.pose.orientation.w = 1.0
                leg_l.scale.x = 0.07
                leg_l.scale.y = 0.07
                leg_l.scale.z = 0.1
                leg_l.color.r = 0.1
                leg_l.color.g = 0.7
                leg_l.color.b = 0.2
                leg_l.color.a = 0.85
                marker_array.markers.append(leg_l)

                # --- Kaki kanan (CYLINDER) ---
                leg_r = Marker()
                leg_r.header.frame_id = "map"
                leg_r.header.stamp = now
                leg_r.ns = "person_leg_r"
                leg_r.id = idx * 10 + 6
                leg_r.type = Marker.CYLINDER
                leg_r.action = Marker.ADD
                leg_r.pose.position.x = person_x + 0.05
                leg_r.pose.position.y = person_y
                leg_r.pose.position.z = 0.05
                leg_r.pose.orientation.w = 1.0
                leg_r.scale.x = 0.07
                leg_r.scale.y = 0.07
                leg_r.scale.z = 0.1
                leg_r.color.r = 0.1
                leg_r.color.g = 0.7
                leg_r.color.b = 0.2
                leg_r.color.a = 0.85
                marker_array.markers.append(leg_r)

                # --- Nama person (TEXT di atas kepala) ---
                person_label = Marker()
                person_label.header.frame_id = "map"
                person_label.header.stamp = now
                person_label.ns = "person_label"
                person_label.id = idx * 10 + 7
                person_label.type = Marker.TEXT_VIEW_FACING
                person_label.action = Marker.ADD
                person_label.pose.position.x = person_x
                person_label.pose.position.y = person_y
                person_label.pose.position.z = 0.75
                person_label.pose.orientation.w = 1.0
                person_label.scale.z = 0.18
                person_label.color.r = 1.0
                person_label.color.g = 1.0
                person_label.color.b = 0.0
                person_label.color.a = 1.0
                person_label.text = name
                marker_array.markers.append(person_label)

                # --- Garis penghubung robot → person (LINE_STRIP) ---
                # Garis tipis putih yang menghubungkan posisi robot
                # dengan posisi person, menunjukkan hubungan antara
                # kedua marker secara visual.
                connector = Marker()
                connector.header.frame_id = "map"
                connector.header.stamp = now
                connector.ns = "connector"
                connector.id = idx * 10 + 8
                connector.type = Marker.LINE_STRIP
                connector.action = Marker.ADD
                connector.pose.orientation.w = 1.0
                connector.scale.x = 0.02  # ketebalan garis

                from geometry_msgs.msg import Point
                p1 = Point()
                p1.x = robot_x
                p1.y = robot_y
                p1.z = 0.05
                p2 = Point()
                p2.x = person_x
                p2.y = person_y
                p2.z = 0.05
                connector.points.append(p1)
                connector.points.append(p2)

                connector.color.r = 1.0
                connector.color.g = 1.0
                connector.color.b = 1.0
                connector.color.a = 0.4
                marker_array.markers.append(connector)

        return marker_array


if __name__ == '__main__':
    try:
        viz = DatabaseVisualizer()
        viz.run()
    except rospy.ROSInterruptException:
        pass
