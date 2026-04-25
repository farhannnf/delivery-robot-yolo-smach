#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
State Machine - Revisi Final + Face Recognition
Sesuai Diagram Alur Sistem.pdf dan Kesepakatan Bimbingan TA2

Mahasiswa: Achmad Farhan Firmansyah (1102220025)
Pembimbing: Pak Yani (Pembimbing I), Bu Eka (Pembimbing II)

PERUBAHAN DARI VERSI SEBELUMNYA:
    Hanya StateDelivering yang dimodifikasi.
    Semua state lain (StateIdle, StateNavigating, StateComplete) TIDAK BERUBAH.

    StateDelivering sebelumnya:
        - Subscribe /yolo/detections (Point)
        - person_detected = True kalau ADA SIAPAPUN yang terdeteksi
        - Tidak memverifikasi identitas penerima → bisa menyerahkan ke orang salah

    StateDelivering sekarang:
        - Subscribe /yolo/detections (Point) — tetap, untuk koordinat
        - Subscribe /yolo/person_id  (String) — BARU, untuk nama dari InsightFace
        - person_detected = True HANYA kalau nama match dengan recipient_name
        - Jika ada orang tapi bukan penerima yang benar → terus scan sampai timeout

    Tidak ada perubahan lain. coordinate_transformer.py tidak disentuh.
"""

import rospy
import smach
import smach_ros
import yaml
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point
from std_msgs.msg import String    # ← IMPORT BARU untuk /yolo/person_id
from actionlib import SimpleActionClient
import math
import tf2_ros


# ========================================
# CLASS DATABASE (TIDAK BERUBAH)
# ========================================
class PositionDatabase:
    def __init__(self):
        package_path = os.path.expanduser('~/catkin_ws/src/ta2_farhan')
        yaml_path = os.path.join(package_path, 'config', 'position_database.yaml')
        rospy.loginfo("Loading database from: %s", yaml_path)
        self.database = self.load_database(yaml_path)
        if self.database:
            rospy.loginfo("Database loaded successfully!")
            for entry in self.database:
                rospy.loginfo("  - %s: (%.2f, %.2f, %.2f)",
                             entry['name'], entry['x'], entry['y'], entry['theta'])
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


# ========================================
# GLOBAL MISSION TRACKER (TIDAK BERUBAH)
# ========================================
class MissionTracker:
    def __init__(self):
        self.total_attempts = 0
        self.successful_deliveries = 0
        self.navigation_failures = []

    def log_attempt(self):
        self.total_attempts += 1

    def log_success(self):
        self.successful_deliveries += 1

    def log_failure(self, reason):
        self.navigation_failures.append(reason)

    def print_statistics(self):
        rospy.loginfo("=" * 60)
        rospy.loginfo("MISSION STATISTICS")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Total attempts: %d", self.total_attempts)
        rospy.loginfo("Successful deliveries: %d", self.successful_deliveries)
        rospy.loginfo("Failed missions: %d", len(self.navigation_failures))
        if self.total_attempts > 0:
            rate = (self.successful_deliveries / float(self.total_attempts)) * 100.0
            rospy.loginfo("Success rate: %.1f%%", rate)
        if self.navigation_failures:
            rospy.loginfo("Failure reasons:")
            for i, reason in enumerate(self.navigation_failures, 1):
                rospy.loginfo("  %d. %s", i, reason)
        rospy.loginfo("=" * 60)


mission_tracker = MissionTracker()


# ========================================
# STATE 1: IDLE (TIDAK BERUBAH)
# ========================================
class StateIdle(smach.State):
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['start_task', 'failed'],
            input_keys=['retry_count'],
            output_keys=['goal_x', 'goal_y', 'goal_theta', 'retry_count']
        )
        self.database = database
        self.mission_started = False
        self.max_retries = 3

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: IDLE")
        rospy.loginfo("=" * 60)

        if self.mission_started and userdata.retry_count >= self.max_retries:
            rospy.logerr("MAXIMUM RETRY ATTEMPTS REACHED (%d/%d)",
                         userdata.retry_count, self.max_retries)
            mission_tracker.log_failure("MAX_RETRY_REACHED")
            mission_tracker.print_statistics()
            return 'failed'

        if self.mission_started:
            rospy.logwarn(">>> RETRY ATTEMPT %d/%d",
                         userdata.retry_count + 1, self.max_retries)

        recipient = rospy.get_param('/delivery/recipient_name', '')
        if not recipient:
            rospy.logerr("Parameter /delivery/recipient_name kosong!")
            return 'failed'

        rospy.loginfo(">>> Target penerima: %s", recipient)
        result = self.database.get_position(recipient)

        if not result:
            rospy.logerr(">>> Nama '%s' TIDAK DITEMUKAN di database!", recipient)
            return 'failed'

        x, y, theta = result
        userdata.goal_x     = x
        userdata.goal_y     = y
        userdata.goal_theta = theta

        rospy.loginfo(">>> Koordinat tujuan: (%.2f, %.2f, %.2f)", x, y, theta)
        self.mission_started = True
        mission_tracker.log_attempt()
        rospy.sleep(1.0)
        return 'start_task'


# ========================================
# STATE 2: NAVIGATING (TIDAK BERUBAH)
# ========================================
class StateNavigating(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['arrived', 'failed', 'timeout'],
            input_keys=['goal_x', 'goal_y', 'goal_theta', 'retry_count'],
            output_keys=['retry_count']
        )
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.client      = SimpleActionClient('move_base', MoveBaseAction)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: NAVIGATING")
        rospy.loginfo("=" * 60)

        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server tidak tersedia!")
            userdata.retry_count += 1
            mission_tracker.log_failure("MOVE_BASE_UNAVAILABLE")
            return 'failed'

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = userdata.goal_x
        goal.target_pose.pose.position.y = userdata.goal_y
        goal.target_pose.pose.orientation.z = math.sin(userdata.goal_theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(userdata.goal_theta / 2.0)

        rospy.loginfo(">>> Mengirim goal: (%.2f, %.2f)", userdata.goal_x, userdata.goal_y)
        self.client.cancel_all_goals()
        rospy.sleep(0.5)
        self.client.send_goal(goal)

        timeout_duration = 120.0
        if self.client.wait_for_result(rospy.Duration(timeout_duration)):
            state = self.client.get_state()
            if state == 3:  # SUCCEEDED
                rospy.loginfo(">>> NAVIGATION SUCCESS")
                try:
                    rospy.sleep(0.5)
                    transform = self.tf_buffer.lookup_transform(
                        'map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0)
                    )
                    actual_x = transform.transform.translation.x
                    actual_y = transform.transform.translation.y
                    error = math.sqrt(
                        (actual_x - userdata.goal_x)**2 +
                        (actual_y - userdata.goal_y)**2
                    ) * 100.0
                    rospy.loginfo(">>> Error posisi: %.2f cm", error)
                except Exception as e:
                    rospy.logwarn("Tidak dapat menghitung error posisi: %s", str(e))
                return 'arrived'
            elif state == 4:  # ABORTED
                userdata.retry_count += 1
                mission_tracker.log_failure("ABORTED (state=4)")
                return 'failed'
            else:
                userdata.retry_count += 1
                mission_tracker.log_failure("UNEXPECTED_STATE (state=%d)" % state)
                return 'failed'

        self.client.cancel_goal()
        userdata.retry_count += 1
        mission_tracker.log_failure("TIMEOUT")
        return 'timeout'


# ========================================
# STATE 3: DELIVERING — DIREVISI
# Sekarang memverifikasi identitas penerima via InsightFace
# ========================================
class StateDelivering(smach.State):
    """
    State verifikasi penerima: scanning YOLO + InsightFace selama 10 detik.

    PERUBAHAN dari versi lama:
        Sebelumnya, person_detected = True jika ADA SIAPAPUN yang terdeteksi.
        Sekarang, person_detected = True HANYA jika nama yang terdeteksi
        InsightFace cocok dengan nama penerima di /delivery/recipient_name.

        Pencocokan nama menggunakan case-insensitive comparison dan menangani
        variasi nama seperti "Pak Yani" vs "Yani" (lihat is_name_match()).

    Alur baru:
        1. Subscribe ke /yolo/detections (Point) — sama, untuk koordinat
        2. Subscribe ke /yolo/person_id  (String) — BARU, nama dari InsightFace
        3. Scan selama 10 detik
        4. Jika nama yang terdeteksi cocok dengan penerima → 'delivered'
        5. Jika ada orang tapi nama tidak cocok → log warning, terus scan
        6. Jika timeout 10 detik tanpa penerima yang benar → 'no_confirm' (SC04)
    """
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['delivered', 'no_confirm']
        )

        # Flag dan data deteksi
        self.person_detected    = False    # True hanya jika PENERIMA YANG BENAR hadir
        self.detection_confidence = 0.0
        self.last_detected_name = ""      # Nama terakhir yang dilaporkan InsightFace
        self.active = False

        # Subscribe ke /yolo/detections (Point) — sama persis dengan versi lama
        # Ini untuk mendapatkan koordinat centroid (dipakai coordinate_transformer)
        self.detection_subscriber = rospy.Subscriber(
            '/yolo/detections',
            Point,
            self.detection_callback
        )

        # Subscribe ke /yolo/person_id (String) — BARU
        # Topic ini dipublish oleh yolo_detector_hybrid.py
        # Berisi nama orang yang diidentifikasi InsightFace
        self.person_id_subscriber = rospy.Subscriber(
            '/yolo/person_id',
            String,
            self.person_id_callback
        )

    def is_name_match(self, detected_name, recipient_name):
        """
        Cek apakah nama yang dideteksi InsightFace cocok dengan penerima.

        Menggunakan beberapa strategi pencocokan agar lebih robust:
            1. Exact match (case-insensitive): "Pak Yani" == "pak yani"
            2. Substring match: "Yani" ada di dalam "Pak Yani"
               Ini penting karena database mungkin pakai nama pendek ("Yani")
               sedangkan rosparam memakai nama lengkap ("Pak Yani") atau sebaliknya.

        Contoh yang akan match:
            detected="Yani", recipient="Pak Yani"   → True (substring)
            detected="Pak Yani", recipient="Yani"   → True (substring)
            detected="Farhan", recipient="Pak Yani" → False
            detected="Unknown", recipient="Pak Yani" → False
        """
        if detected_name == "Unknown" or not detected_name:
            return False

        det  = detected_name.lower().strip()
        recv = recipient_name.lower().strip()

        # Exact match atau salah satu adalah bagian dari yang lain
        return det == recv or det in recv or recv in det

    def detection_callback(self, msg):
        """
        Callback dari /yolo/detections (Point).
        Simpan confidence — pencocokan nama dilakukan di person_id_callback.
        """
        self.detection_confidence = msg.z

    def person_id_callback(self, msg):
        """
        Callback dari /yolo/person_id (String) — BARU.
        Cek apakah nama yang dideteksi InsightFace cocok dengan penerima.

        Callback ini dipanggil setiap kali yolo_detector_hybrid.py berhasil
        mengidentifikasi seseorang. Jika cocok, set person_detected = True
        sehingga execute() bisa segera keluar dengan outcome 'delivered'.
        """
        detected_name = msg.data
        self.last_detected_name = detected_name

        # Baca nama penerima saat ini dari rosparam
        recipient = rospy.get_param('/delivery/recipient_name', '')

        if not recipient:
            return

        if self.is_name_match(detected_name, recipient):
            # Penerima yang benar terdeteksi!
            self.person_detected = True
            rospy.loginfo("[DELIVERING] MATCH! Detected='%s', Recipient='%s'",
                          detected_name, recipient)
        else:
            # Ada orang tapi bukan penerima yang dituju
            if detected_name != "Unknown":
                rospy.logwarn("[DELIVERING] Orang terdeteksi tapi bukan penerima: "
                              "detected='%s', expected='%s'",
                              detected_name, recipient)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: DELIVERING (dengan face verification)")
        rospy.loginfo("=" * 60)

        recipient = rospy.get_param('/delivery/recipient_name', 'Unknown')
        rospy.loginfo(">>> Robot sudah di lokasi tujuan")
        rospy.loginfo(">>> Target penerima: %s", recipient)
        rospy.loginfo(">>> Scanning area dengan YOLO + InsightFace (10 detik)...")
        rospy.loginfo(">>> Delivery hanya dilakukan jika penerima yang benar terdeteksi.")

        # Reset flag sebelum scanning dimulai
        self.person_detected    = False
        self.last_detected_name = ""

        timeout = rospy.Time.now() + rospy.Duration(20.0)
        rate    = rospy.Rate(20)  # Check 10x per detik

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.person_detected:
                rospy.loginfo("=" * 60)
                rospy.loginfo(">>> PENERIMA TERVERIFIKASI!")
                rospy.loginfo(">>> Nama terdeteksi : '%s'", self.last_detected_name)
                rospy.loginfo(">>> Penerima target  : '%s'", recipient)
                rospy.loginfo(">>> Confidence YOLO  : %.2f", self.detection_confidence)
                rospy.loginfo("=" * 60)

                rospy.loginfo(">>> Menyerahkan barang ke %s...", recipient)
                rospy.sleep(2.0)
                rospy.loginfo(">>> Barang berhasil diserahkan!")
                self.active = False

                return 'delivered'

            rate.sleep()

        # Timeout 10 detik — penerima yang benar tidak terdeteksi
        rospy.logwarn("=" * 60)
        rospy.logwarn(">>> TIMEOUT: Penerima '%s' tidak terverifikasi dalam 10 detik",
                      recipient)
        if self.last_detected_name:
            rospy.logwarn(">>> Orang terakhir terdeteksi: '%s' (bukan penerima)",
                          self.last_detected_name)
        else:
            rospy.logwarn(">>> Tidak ada orang terdeteksi sama sekali di area tujuan")
        rospy.logwarn(">>> SC04: timeout delivering — melanjutkan mission")
        rospy.logwarn("=" * 60)
        self.active = False

        return 'no_confirm'


# ========================================
# STATE 4: COMPLETE (TIDAK BERUBAH)
# ========================================
class StateComplete(smach.State):
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['return_home'],
            input_keys=['goal_x', 'goal_y', 'goal_theta'],
            output_keys=['goal_x', 'goal_y', 'goal_theta']
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
            rospy.logerr("'Origin' tidak ditemukan di database! Pakai (0,0,0)")
            origin = (0.0, 0.0, 0.0)

        userdata.goal_x     = origin[0]
        userdata.goal_y     = origin[1]
        userdata.goal_theta = origin[2]

        rospy.loginfo(">>> Kembali ke origin: (%.2f, %.2f)", origin[0], origin[1])
        return 'return_home'


# ========================================
# MAIN (TIDAK BERUBAH)
# ========================================
def main():
    rospy.init_node('delivery_state_machine')

    rospy.loginfo("=" * 60)
    rospy.loginfo("DELIVERY STATE MACHINE - Revisi Final + Face Verification")
    rospy.loginfo("States: IDLE -> NAVIGATING -> DELIVERING -> COMPLETE -> RETURN_HOME")
    rospy.loginfo("Perubahan: StateDelivering sekarang verifikasi identitas via InsightFace")
    rospy.loginfo("=" * 60)

    database = PositionDatabase()

    sm = smach.StateMachine(outcomes=['success', 'failed'])
    sm.userdata.retry_count = 0

    with sm:
        smach.StateMachine.add('IDLE', StateIdle(database),
            transitions={'start_task': 'NAVIGATING', 'failed': 'failed'})

        smach.StateMachine.add('NAVIGATING', StateNavigating(),
            transitions={'arrived': 'DELIVERING', 'failed': 'IDLE', 'timeout': 'IDLE'})

        smach.StateMachine.add('DELIVERING', StateDelivering(),
            transitions={'delivered': 'COMPLETE', 'no_confirm': 'COMPLETE'})

        smach.StateMachine.add('COMPLETE', StateComplete(database),
            transitions={'return_home': 'RETURN_HOME'})

        smach.StateMachine.add('RETURN_HOME', StateNavigating(),
            transitions={'arrived': 'success', 'failed': 'failed', 'timeout': 'failed'})

    sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("State Machine ready.")
    rospy.loginfo("Set delivery target: rosparam set /delivery/recipient_name '<nama_penerima>'")

    outcome = sm.execute()

    rospy.loginfo("STATE MACHINE FINISHED. Final outcome: %s", outcome)
    mission_tracker.print_statistics()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine interrupted.")
