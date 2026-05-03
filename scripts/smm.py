#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import yaml
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import String
from actionlib import SimpleActionClient
import math
import tf2_ros


# ========================================
# CLASS DATABASE
# ========================================
class PositionDatabase:
    def __init__(self):
        package_path = os.path.expanduser('~/catkin_ws/src/ta2_farhan')
        yaml_path = os.path.join(package_path, 'config',
                                 'position_database.yaml')
        rospy.loginfo("Loading database from: %s", yaml_path)
        self.database = self.load_database(yaml_path)
        if self.database:
            rospy.loginfo("Database loaded successfully!")
            for entry in self.database:
                rospy.loginfo("  - %s: (%.2f, %.2f, %.2f)",
                             entry['name'], entry['x'], entry['y'],
                             entry['theta'])
        else:
            rospy.logerr("Failed to load database!")

    def load_database(self, yaml_path):
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                return data['positions']
        except Exception as e:
            rospy.logerr("Error loading YAML: %s", str(e))
            return None

    def get_position(self, name):
        if self.database is None:
            return None
        for entry in self.database:
            if entry['name'].lower() == name.lower():
                return (entry['x'], entry['y'], entry['theta'])
        return None

    def get_person_position(self, name):
        """Ambil posisi PERSON (person_x, person_y) dari database."""
        if self.database is None:
            return None
        for entry in self.database:
            if entry['name'].lower() == name.lower():
                px = entry.get('person_x', None)
                py = entry.get('person_y', None)
                if px is not None and py is not None:
                    return (px, py)
        return None

    def find_nearest_alternate(self, original_name):
        """
        Cari orang terdekat dari POSISI PENERIMA ASLI di database.
        Mengecualikan Origin dan penerima asli.

        Referensi: posisi penerima asli (bukan posisi robot saat ini),
        karena kita mencari orang yang lokasinya paling dekat dengan
        penerima asli agar robot tidak perlu menempuh jarak jauh.

        Return: dict {name, x, y, theta, person_x, person_y, distance}
                atau None jika tidak ada kandidat.
        """
        if self.database is None:
            return None

        # Ambil posisi penerima asli sebagai titik referensi
        original_pos = self.get_position(original_name)
        if original_pos is None:
            return None
        orig_x, orig_y, _ = original_pos

        nearest = None
        min_dist = float('inf')

        for entry in self.database:
            name = entry.get('name', '')
            if name.lower() == 'origin':
                continue
            if name.lower() == original_name.lower():
                continue

            ex = entry.get('x', 0)
            ey = entry.get('y', 0)
            dist = math.sqrt((orig_x - ex)**2 + (orig_y - ey)**2)

            rospy.loginfo("[FIND_ALTERNATE] Kandidat: %s (jarak %.2f m)",
                          name, dist)

            if dist < min_dist:
                min_dist = dist
                nearest = {
                    'name': name,
                    'x': ex, 'y': ey,
                    'theta': entry.get('theta', 0),
                    'person_x': entry.get('person_x', None),
                    'person_y': entry.get('person_y', None),
                    'distance': dist
                }

        return nearest


# ========================================
# MISSION TRACKER
# ========================================
class MissionTracker:
    def __init__(self):
        self.total_attempts = 0
        self.successful_deliveries = 0
        self.alternate_deliveries = 0
        self.navigation_failures = []

    def log_attempt(self):
        self.total_attempts += 1

    def log_success(self):
        self.successful_deliveries += 1

    def log_alternate_success(self):
        self.alternate_deliveries += 1
        self.successful_deliveries += 1

    def log_failure(self, reason):
        self.navigation_failures.append(reason)

    def print_statistics(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("MISSION STATISTICS")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Total attempts: %d", self.total_attempts)
        rospy.loginfo("Successful deliveries: %d",
                       self.successful_deliveries)
        rospy.loginfo("  Direct deliveries  : %d",
                       self.successful_deliveries -
                       self.alternate_deliveries)
        rospy.loginfo("  Alternate deliveries: %d",
                       self.alternate_deliveries)
        rospy.loginfo("Failed missions: %d",
                       len(self.navigation_failures))
        if self.total_attempts > 0:
            rate = (self.successful_deliveries /
                    float(self.total_attempts)) * 100.0
            rospy.loginfo("Success rate: %.1f%%", rate)
        if self.navigation_failures:
            rospy.loginfo("Failure reasons:")
            for i, reason in enumerate(self.navigation_failures, 1):
                rospy.loginfo("  %d. %s", i, reason)
        rospy.loginfo("=" * 60)


mission_tracker = MissionTracker()


# ========================================
# STATE: IDLE
# ========================================
class StateIdle(smach.State):
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['start_task', 'failed'],
            input_keys=['retry_count'],
            output_keys=['goal_x', 'goal_y', 'goal_theta',
                         'person_x', 'person_y', 'retry_count']
        )
        self.database = database
        self.mission_started = False
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: IDLE")
        rospy.loginfo("=" * 60)

        if self.mission_started and userdata.retry_count >= self.max_retries:
            rospy.logerr("MAX RETRY REACHED (%d/%d)",
                         userdata.retry_count, self.max_retries)
            mission_tracker.log_failure("MAX_RETRY_REACHED")
            mission_tracker.print_statistics()
            return 'failed'

        if self.mission_started:
            rospy.logwarn(">>> RETRY %d/%d",
                         userdata.retry_count + 1, self.max_retries)

        recipient = rospy.get_param('/delivery/recipient_name', '')
        if not recipient:
            rospy.logerr("Parameter /delivery/recipient_name kosong!")
            return 'failed'

        rospy.loginfo(">>> Target penerima: %s", recipient)
        result = self.database.get_position(recipient)

        if not result:
            rospy.logerr(">>> '%s' TIDAK ADA di database!", recipient)
            return 'failed'

        x, y, theta = result
        userdata.goal_x = x
        userdata.goal_y = y
        userdata.goal_theta = theta

        # Ambil posisi person untuk dual error calculation
        person_pos = self.database.get_person_position(recipient)
        if person_pos:
            userdata.person_x = person_pos[0]
            userdata.person_y = person_pos[1]
            rospy.loginfo(">>> Posisi person di DB: (%.4f, %.4f)",
                          person_pos[0], person_pos[1])
        else:
            userdata.person_x = None
            userdata.person_y = None

        rospy.loginfo(">>> Koordinat tujuan (robot): (%.2f, %.2f, %.2f)",
                       x, y, theta)
        self.mission_started = True
        mission_tracker.log_attempt()
        rospy.sleep(1.0)
        return 'start_task'


# ========================================
# STATE: NAVIGATING
# Timeout: 25 menit (1500 detik)
# ========================================
class StateNavigating(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed', 'timeout'],
            input_keys=['goal_x', 'goal_y', 'goal_theta',
                        'person_x', 'person_y', 'retry_count'],
            output_keys=['retry_count']
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: NAVIGATING")
        rospy.loginfo("=" * 60)

        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base tidak tersedia!")
            userdata.retry_count += 1
            mission_tracker.log_failure("MOVE_BASE_UNAVAILABLE")
            return 'failed'

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.goal_x
        goal.target_pose.pose.position.y = userdata.goal_y
        goal.target_pose.pose.orientation.z = math.sin(
            userdata.goal_theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(
            userdata.goal_theta / 2.0)

        rospy.loginfo(">>> Mengirim goal: (%.2f, %.2f)",
                       userdata.goal_x, userdata.goal_y)
        self.client.send_goal(goal)

        # Timeout navigasi: 25 menit = 1500 detik
        if self.client.wait_for_result(rospy.Duration(1500.0)):
            state = self.client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo(">>> NAVIGATION SUCCESS")
                try:
                    rospy.sleep(0.5)
                    transform = self.tf_buffer.lookup_transform(
                        'map', 'base_footprint',
                        rospy.Time(0), rospy.Duration(2.0))
                    actual_x = transform.transform.translation.x
                    actual_y = transform.transform.translation.y

                    # Error vs Goal (posisi robot di database)
                    error_goal = math.sqrt(
                        (actual_x - userdata.goal_x)**2 +
                        (actual_y - userdata.goal_y)**2
                    ) * 100.0
                    rospy.loginfo(">>> Error posisi ROBOT vs GOAL: "
                                  "%.2f cm", error_goal)

                    # Error vs Person (posisi orang di database)
                    if (hasattr(userdata, 'person_x') and
                            userdata.person_x is not None and
                            userdata.person_y is not None):
                        error_person = math.sqrt(
                            (actual_x - userdata.person_x)**2 +
                            (actual_y - userdata.person_y)**2
                        ) * 100.0
                        rospy.loginfo(">>> Error posisi ROBOT vs PERSON: "
                                      "%.2f cm", error_person)
                        rospy.loginfo("=" * 50)
                        rospy.loginfo("  RINGKASAN ERROR POSISI:")
                        rospy.loginfo("  Robot aktual : (%.4f, %.4f)",
                                      actual_x, actual_y)
                        rospy.loginfo("  Goal (DB)    : (%.4f, %.4f)",
                                      userdata.goal_x, userdata.goal_y)
                        rospy.loginfo("  Person (DB)  : (%.4f, %.4f)",
                                      userdata.person_x,
                                      userdata.person_y)
                        rospy.loginfo("  Err vs Goal  : %.2f cm",
                                      error_goal)
                        rospy.loginfo("  Err vs Person: %.2f cm",
                                      error_person)
                        rospy.loginfo("=" * 50)
                    else:
                        rospy.logwarn(">>> person_x/y tidak tersedia "
                                      "- hanya error vs goal dihitung")
                except Exception as e:
                    rospy.logwarn("Error calc failed: %s", str(e))
                return 'arrived'
            elif state == 4:  # ABORTED
                userdata.retry_count += 1
                mission_tracker.log_failure("ABORTED (state=4)")
                return 'failed'
            else:
                userdata.retry_count += 1
                mission_tracker.log_failure("NAV_STATE=%d" % state)
                return 'failed'

        self.client.cancel_goal()
        userdata.retry_count += 1
        mission_tracker.log_failure("TIMEOUT")
        return 'timeout'


# ========================================
# STATE: DELIVERING (Face Verification)
# Scan timeout : 25 detik
# Jeda setelah serah terima : 25 detik
# ========================================
class StateDelivering(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['delivered', 'no_confirm']
        )
        self.person_detected = False
        self.detection_confidence = 0.0
        self.last_detected_name = ""

        rospy.Subscriber('/yolo/detections', Point,
                         self.detection_callback)
        rospy.Subscriber('/yolo/person_id', String,
                         self.person_id_callback)

    def is_name_match(self, detected, recipient):
        if detected == "Unknown" or not detected:
            return False
        d = detected.lower().strip()
        r = recipient.lower().strip()
        return d == r or d in r or r in d

    def detection_callback(self, msg):
        self.detection_confidence = msg.z

    def person_id_callback(self, msg):
        detected_name = msg.data
        self.last_detected_name = detected_name
        recipient = rospy.get_param('/delivery/recipient_name', '')
        if not recipient:
            return
        if self.is_name_match(detected_name, recipient):
            self.person_detected = True
            rospy.loginfo("[DELIVERING] MATCH! Detected='%s', "
                          "Recipient='%s'", detected_name, recipient)
        elif detected_name != "Unknown":
            rospy.logwarn("[DELIVERING] Bukan penerima: "
                          "detected='%s', expected='%s'",
                          detected_name, recipient)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: DELIVERING (face verification)")
        rospy.loginfo("=" * 60)

        recipient = rospy.get_param('/delivery/recipient_name', '')
        rospy.loginfo(">>> Robot sudah di lokasi tujuan")
        rospy.loginfo(">>> Target penerima: %s", recipient)
        rospy.loginfo(">>> Scanning YOLO + InsightFace (25 detik)...")
        rospy.loginfo(">>> Delivery hanya jika penerima terverifikasi")

        self.person_detected = False
        self.last_detected_name = ""

        # Scan selama 25 detik
        timeout = rospy.Time.now() + rospy.Duration(25.0)
        rate = rospy.Rate(20)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.person_detected:
                rospy.loginfo("=" * 60)
                rospy.loginfo(">>> PENERIMA TERVERIFIKASI!")
                rospy.loginfo(">>> Nama   : '%s'",
                               self.last_detected_name)
                rospy.loginfo(">>> Conf   : %.2f",
                               self.detection_confidence)
                rospy.loginfo("=" * 60)
                rospy.loginfo(">>> Menyerahkan barang ke %s...",
                               recipient)

                # Jeda 25 detik untuk proses serah terima barang
                rospy.loginfo(">>> Menunggu 25 detik untuk serah "
                              "terima barang...")
                rospy.sleep(25.0)

                rospy.loginfo(">>> Barang berhasil diserahkan!")
                return 'delivered'
            rate.sleep()

        # Timeout — penerima tidak terverifikasi
        rospy.logwarn("=" * 60)
        rospy.logwarn(">>> TIMEOUT: '%s' tidak terverifikasi "
                       "dalam 25 detik", recipient)
        if self.last_detected_name:
            rospy.logwarn(">>> Terakhir terdeteksi: '%s'",
                           self.last_detected_name)
        else:
            rospy.logwarn(">>> Tidak ada orang terdeteksi")
        rospy.logwarn(">>> Melanjutkan ke FIND_ALTERNATE...")
        rospy.logwarn("=" * 60)

        # Jeda 25 detik sebelum pindah ke alternate
        rospy.loginfo(">>> Menunggu 25 detik sebelum mencari "
                      "orang alternatif...")
        rospy.sleep(25.0)

        return 'no_confirm'


# ========================================
# STATE: FIND_ALTERNATE
# Cari orang terdekat, navigasi, titipkan
# ========================================
class StateFindAlternate(smach.State):
    """
    Monolithic state: mencari orang terdekat di database,
    navigasi ke sana, verifikasi siapapun yang ada, dan
    titipkan barang. Menggabungkan fitur dari kedua versi
    state machine (modular dan monolithic).

    Log output:
      GOODS ENTRUSTED TO <NAMA> | Original recipient '<nama>'
      was not present. Nearest person '<nama>' (<jarak>m away)
      accepted the delivery.
    """
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['found_alternate', 'no_alternate',
                      'navigation_failed'],
            output_keys=['goal_x', 'goal_y', 'goal_theta',
                         'person_x', 'person_y']
        )
        self.database = database
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher untuk status alternate delivery
        self.status_pub = rospy.Publisher(
            '/delivery/alternate_status',
            String, queue_size=10)

        # Subscriber untuk face detection di lokasi alternate
        self.alt_person_detected = False
        self.alt_detected_name = ""
        self.alt_confidence = 0.0
        rospy.Subscriber('/yolo/person_id', String,
                         self._alt_person_callback)
        rospy.Subscriber('/yolo/detections', Point,
                         self._alt_detection_callback)

    def _alt_detection_callback(self, msg):
        self.alt_confidence = msg.z

    def _alt_person_callback(self, msg):
        name = msg.data
        self.alt_detected_name = name
        if name and name != "Unknown":
            self.alt_person_detected = True

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: FIND_ALTERNATE")
        rospy.loginfo("Penerima asli tidak terdeteksi. "
                      "Mencari orang terdekat...")
        rospy.loginfo("=" * 60)

        original_name = rospy.get_param(
            '/delivery/recipient_name', '')
        if not original_name:
            rospy.logerr("[FIND_ALTERNATE] recipient_name kosong!")
            return 'no_alternate'

        rospy.loginfo("[FIND_ALTERNATE] Penerima asli: %s",
                       original_name)

        # Cari orang terdekat dari posisi penerima asli
        nearest = self.database.find_nearest_alternate(original_name)

        if nearest is None:
            rospy.logwarn("[FIND_ALTERNATE] Tidak ada kandidat "
                          "alternatif di database!")
            msg = String()
            msg.data = ("NO_ALTERNATE: Tidak ada orang lain di "
                        "database. Barang tidak dapat diserahkan.")
            self.status_pub.publish(msg)
            return 'no_alternate'

        alt_name = nearest['name']
        alt_x = nearest['x']
        alt_y = nearest['y']
        alt_theta = nearest['theta']
        distance = nearest['distance']

        rospy.loginfo("=" * 60)
        rospy.loginfo("[FIND_ALTERNATE] Alternatif: %s", alt_name)
        rospy.loginfo("  Posisi   : (%.3f, %.3f)", alt_x, alt_y)
        rospy.loginfo("  Jarak dari %s: %.3f meter",
                       original_name, distance)
        rospy.loginfo("=" * 60)

        # ========================================
        # NAVIGASI KE ORANG ALTERNATIF
        # ========================================
        rospy.loginfo("[FIND_ALTERNATE] Navigasi ke %s...", alt_name)

        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("[FIND_ALTERNATE] move_base unavailable!")
            return 'navigation_failed'

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = alt_x
        goal.target_pose.pose.position.y = alt_y
        goal.target_pose.pose.orientation.z = math.sin(
            alt_theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(
            alt_theta / 2.0)

        self.client.send_goal(goal)

        # Timeout navigasi alternate: 25 menit
        if not self.client.wait_for_result(rospy.Duration(1500.0)):
            self.client.cancel_goal()
            rospy.logwarn("[FIND_ALTERNATE] Timeout navigasi ke %s!",
                           alt_name)
            return 'navigation_failed'

        nav_state = self.client.get_state()
        if nav_state != 3:  # bukan SUCCEEDED
            rospy.logwarn("[FIND_ALTERNATE] Navigasi gagal (state=%d)",
                           nav_state)
            return 'navigation_failed'

        # ========================================
        # HITUNG ERROR POSISI DI LOKASI ALTERNATE
        # ========================================
        try:
            rospy.sleep(0.5)
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rospy.Time(0), rospy.Duration(2.0))
            actual_x = transform.transform.translation.x
            actual_y = transform.transform.translation.y

            error_goal = math.sqrt(
                (actual_x - alt_x)**2 +
                (actual_y - alt_y)**2
            ) * 100.0
            rospy.loginfo("[FIND_ALTERNATE] Error vs Goal: %.2f cm",
                          error_goal)

            if (nearest['person_x'] is not None and
                    nearest['person_y'] is not None):
                error_person = math.sqrt(
                    (actual_x - nearest['person_x'])**2 +
                    (actual_y - nearest['person_y'])**2
                ) * 100.0
                rospy.loginfo("[FIND_ALTERNATE] Error vs Person: "
                              "%.2f cm", error_person)
        except Exception as e:
            rospy.logwarn("Error calc failed: %s", str(e))

        # ========================================
        # SCAN ORANG DI LOKASI ALTERNATE (25 detik)
        # ========================================
        rospy.loginfo("[FIND_ALTERNATE] Tiba di lokasi %s", alt_name)
        rospy.loginfo("[FIND_ALTERNATE] Scanning 25 detik...")

        self.alt_person_detected = False
        self.alt_detected_name = ""

        timeout = rospy.Time.now() + rospy.Duration(25.0)
        rate = rospy.Rate(20)

        person_found = False
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.alt_person_detected:
                person_found = True
                break
            rate.sleep()

        if person_found:
            # ========================================
            # BARANG DITITIPKAN — LOG UTAMA
            # ========================================
            rospy.loginfo("=" * 60)
            rospy.loginfo("[FIND_ALTERNATE] TIBA DI LOKASI ALTERNATIF!")
            rospy.loginfo("  Penerima asli     : %s (tidak hadir)",
                           original_name)
            rospy.loginfo("  Penerima pengganti: %s",
                           self.alt_detected_name)
            rospy.loginfo("  Status: GOODS ENTRUSTED TO %s",
                           self.alt_detected_name.upper())
            rospy.loginfo("=" * 60)

            # Publish status ke topic
            status_msg = String()
            status_msg.data = (
                "GOODS ENTRUSTED TO %s | "
                "Original recipient '%s' was not present. "
                "Nearest person '%s' (%.2fm away) "
                "accepted the delivery."
                % (self.alt_detected_name.upper(),
                   original_name, self.alt_detected_name, distance)
            )
            self.status_pub.publish(status_msg)
            rospy.loginfo("[FIND_ALTERNATE] Status dipublish ke "
                          "/delivery/alternate_status")

            # Jeda 25 detik untuk serah terima barang
            rospy.loginfo(">>> Menunggu 25 detik untuk serah "
                          "terima barang...")
            rospy.sleep(25.0)
            rospy.loginfo(">>> Barang berhasil dititipkan!")

            mission_tracker.log_alternate_success()
        else:
            # Tidak ada orang di lokasi alternate
            rospy.logwarn("[FIND_ALTERNATE] Tidak ada orang "
                          "di lokasi %s!", alt_name)
            rospy.logwarn("[FIND_ALTERNATE] Barang tidak "
                          "dapat dititipkan.")

            status_msg = String()
            status_msg.data = (
                "ALTERNATE FAILED | '%s' not present, "
                "alternate '%s' also not present."
                % (original_name, alt_name)
            )
            self.status_pub.publish(status_msg)

            # Tetap jeda 25 detik sebelum kembali
            rospy.loginfo(">>> Menunggu 25 detik sebelum kembali...")
            rospy.sleep(25.0)

        # Set goal ke Origin untuk RETURN_HOME
        origin = self.database.get_position("Origin")
        if origin:
            userdata.goal_x = origin[0]
            userdata.goal_y = origin[1]
            userdata.goal_theta = origin[2]
        else:
            userdata.goal_x = 0.0
            userdata.goal_y = 0.0
            userdata.goal_theta = 0.0
        userdata.person_x = None
        userdata.person_y = None

        return 'found_alternate'


# ========================================
# STATE: COMPLETE
# ========================================
class StateComplete(smach.State):
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['return_home'],
            input_keys=['goal_x', 'goal_y', 'goal_theta'],
            output_keys=['goal_x', 'goal_y', 'goal_theta',
                         'person_x', 'person_y']
        )
        self.database = database

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: COMPLETE")
        rospy.loginfo("=" * 60)

        mission_tracker.log_success()
        rospy.loginfo(">>> Delivery selesai!")

        origin = self.database.get_position("Origin")
        if not origin:
            rospy.logerr("Origin not found! Using (0,0,0)")
            origin = (0.0, 0.0, 0.0)

        userdata.goal_x = origin[0]
        userdata.goal_y = origin[1]
        userdata.goal_theta = origin[2]
        userdata.person_x = None
        userdata.person_y = None

        rospy.loginfo(">>> Kembali ke origin: (%.2f, %.2f)",
                       origin[0], origin[1])
        return 'return_home'


# ========================================
# MAIN
# ========================================
def main():
    rospy.init_node('delivery_state_machine')

    rospy.loginfo("=" * 60)
    rospy.loginfo("DELIVERY STATE MACHINE - Final Merged")
    rospy.loginfo("Normal : IDLE -> NAV -> DELIVER -> "
                  "COMPLETE -> RETURN_HOME")
    rospy.loginfo("Alt    : ... -> DELIVER(timeout) -> "
                  "FIND_ALT -> COMPLETE -> RETURN_HOME")
    rospy.loginfo("Timing : Nav=25min, Scan=25s, "
                  "Handoff=25s")
    rospy.loginfo("=" * 60)

    database = PositionDatabase()

    sm = smach.StateMachine(outcomes=['success', 'failed'])
    sm.userdata.retry_count = 0
    sm.userdata.person_x = None
    sm.userdata.person_y = None

    with sm:
        smach.StateMachine.add('IDLE', StateIdle(database),
            transitions={
                'start_task': 'NAVIGATING',
                'failed': 'failed'})

        smach.StateMachine.add('NAVIGATING', StateNavigating(),
            transitions={
                'arrived': 'DELIVERING',
                'failed': 'IDLE',
                'timeout': 'IDLE'})

        smach.StateMachine.add('DELIVERING', StateDelivering(),
            transitions={
                'delivered': 'COMPLETE',
                'no_confirm': 'FIND_ALTERNATE'})

        smach.StateMachine.add('FIND_ALTERNATE',
            StateFindAlternate(database),
            transitions={
                'found_alternate': 'COMPLETE',
                'no_alternate': 'COMPLETE',
                'navigation_failed': 'COMPLETE'})

        smach.StateMachine.add('COMPLETE',
            StateComplete(database),
            transitions={'return_home': 'RETURN_HOME'})

        smach.StateMachine.add('RETURN_HOME', StateNavigating(),
            transitions={
                'arrived': 'success',
                'failed': 'failed',
                'timeout': 'failed'})

    sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("State Machine ready.")
    rospy.loginfo("Set target: rosparam set /delivery/"
                  "recipient_name '<nama>'")

    outcome = sm.execute()
    rospy.loginfo("FINISHED. Outcome: %s", outcome)
    mission_tracker.print_statistics()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine interrupted.")
