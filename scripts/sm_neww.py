#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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
# CLASS DATABASE
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
# GLOBAL MISSION TRACKER
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
# STATE 1 : IDLE
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
# STATE 2 : NAVIGATING 
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
        self.client.send_goal(goal)

        timeout_duration = 1200.0
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
# STATE 3 : DELIVERING
# Verifikasi Identitas Penerima via InsightFace
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

        timeout = rospy.Time.now() + rospy.Duration(180.0)
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

        return 'no_confirm'


# ========================================
# STATE 4 : FIND_ALTERNATE
# ========================================
class StateFindAlternate(smach.State):
    """
    State ini dipanggil ketika StateDelivering timeout (outcome 'no_confirm'):
    penerima asli tidak terdeteksi dalam 10 detik.

    Mekanisme:
      1. Baca nama penerima asli dari rosparam /delivery/recipient_name
      2. Cari posisi penerima asli di database
      3. Hitung jarak Euclidean ke semua orang lain di database
      4. Pilih orang terdekat sebagai alternate recipient
      5. Navigasi ke posisi orang terdekat tersebut
      6. Setelah tiba, publish status "goods entrusted to <nama>"
      7. Lanjut ke COMPLETE

    Outcomes:
      'found_alternate'    : berhasil menemukan orang lain dan tiba di sana
      'no_alternate'       : database hanya ada 1 orang / tidak ada kandidat lain
      'navigation_failed'  : gagal navigasi ke alternate
    """
    def __init__(self, database):
        smach.State.__init__(
            self,
            outcomes=['found_alternate', 'no_alternate', 'navigation_failed'],
            output_keys=['goal_x', 'goal_y', 'goal_theta']
        )
        self.database = database
        self.client   = SimpleActionClient('move_base', MoveBaseAction)

        # Publisher untuk status pengiriman alternatif
        # Di-monitor via: rostopic echo /delivery/alternate_status
        self.status_pub = rospy.Publisher(
            '/delivery/alternate_status',
            String,
            queue_size=10
        )

    def _find_nearest_alternate(self, original_name):
        """
        Cari orang terdekat dari posisi penerima asli.

        Langkah:
          1. Ambil posisi penerima asli dari database
          2. Loop semua entry, skip Origin dan penerima asli
          3. Hitung jarak Euclidean ke masing-masing
          4. Return nama dan posisi orang dengan jarak terkecil

        Return: (nama, x, y, theta, jarak) atau None jika tidak ada kandidat
        """
        # Ambil posisi penerima asli sebagai titik referensi
        original_pos = self.database.get_position(original_name)
        if original_pos is None:
            rospy.logerr("[FIND_ALTERNATE] Posisi '%s' tidak ada di database!",
                         original_name)
            return None

        orig_x, orig_y, _ = original_pos

        best_name     = None
        best_pos      = None
        best_distance = float('inf')

        for entry in self.database.database:
            candidate_name = entry.get('name', '')

            # Skip: Origin dan penerima asli tidak dihitung sebagai kandidat
            if candidate_name.lower() == 'origin':
                continue
            if candidate_name.lower() == original_name.lower():
                continue

            cand_x = entry.get('x', 0.0)
            cand_y = entry.get('y', 0.0)
            cand_t = entry.get('theta', 0.0)

            # Jarak Euclidean antara posisi penerima asli dan kandidat
            distance = math.sqrt(
                (cand_x - orig_x) ** 2 +
                (cand_y - orig_y) ** 2
            )

            rospy.loginfo("[FIND_ALTERNATE] Kandidat: %s → jarak %.3f m",
                          candidate_name, distance)

            if distance < best_distance:
                best_distance = distance
                best_name     = candidate_name
                best_pos      = (cand_x, cand_y, cand_t)

        if best_name is None:
            return None

        return (best_name, best_pos[0], best_pos[1], best_pos[2], best_distance)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: FIND_ALTERNATE")
        rospy.loginfo("Penerima asli tidak terdeteksi. Mencari orang terdekat...")
        rospy.loginfo("=" * 60)

        # Baca nama penerima asli dari rosparam
        original_name = rospy.get_param('/delivery/recipient_name', '')
        if not original_name:
            rospy.logerr("[FIND_ALTERNATE] /delivery/recipient_name kosong!")
            return 'no_alternate'

        rospy.loginfo("[FIND_ALTERNATE] Penerima asli: %s", original_name)

        # Cari orang terdekat
        result = self._find_nearest_alternate(original_name)

        if result is None:
            rospy.logwarn("[FIND_ALTERNATE] Tidak ada kandidat alternatif di database!")
            msg = String()
            msg.data = ("NO_ALTERNATE: Tidak ada orang lain di database. "
                        "Barang tidak dapat diserahkan.")
            self.status_pub.publish(msg)
            return 'no_alternate'

        alt_name, alt_x, alt_y, alt_theta, distance = result

        rospy.loginfo("=" * 60)
        rospy.loginfo("[FIND_ALTERNATE] Alternatif ditemukan: %s", alt_name)
        rospy.loginfo("  Posisi    : (%.3f, %.3f)", alt_x, alt_y)
        rospy.loginfo("  Jarak dari %s: %.3f meter", original_name, distance)
        rospy.loginfo("=" * 60)

        # Navigasi ke posisi alternatif
        rospy.loginfo("[FIND_ALTERNATE] Mengirim goal ke move_base...")

        if not self.client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("[FIND_ALTERNATE] move_base action server tidak tersedia!")
            return 'navigation_failed'

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp    = rospy.Time.now()
        goal.target_pose.pose.position.x = alt_x
        goal.target_pose.pose.position.y = alt_y
        goal.target_pose.pose.orientation.z = math.sin(alt_theta / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(alt_theta / 2.0)

        self.client.send_goal(goal)

        rospy.loginfo("[FIND_ALTERNATE] Navigasi ke %s (%.2f, %.2f)...",
                      alt_name, alt_x, alt_y)

        if not self.client.wait_for_result(rospy.Duration(120.0)):
            self.client.cancel_goal()
            rospy.logwarn("[FIND_ALTERNATE] Timeout navigasi ke %s!", alt_name)
            return 'navigation_failed'

        nav_state = self.client.get_state()
        if nav_state != 3:  # 3 = SUCCEEDED
            rospy.logwarn("[FIND_ALTERNATE] Navigasi ke %s gagal (state=%d).",
                          alt_name, nav_state)
            return 'navigation_failed'

        # Tiba di lokasi alternatif — publish status dan log
        rospy.loginfo("=" * 60)
        rospy.loginfo("[FIND_ALTERNATE] TIBA DI LOKASI ALTERNATIF!")
        rospy.loginfo("  Penerima asli    : %s (tidak hadir)", original_name)
        rospy.loginfo("  Penerima pengganti: %s", alt_name)
        rospy.loginfo("  Status           : GOODS ENTRUSTED TO %s",
                      alt_name.upper())
        rospy.loginfo("=" * 60)

        # Publish status ke topic — bisa di-monitor atau di-log untuk Bab IV
        status_msg = String()
        status_msg.data = (
            "GOODS ENTRUSTED TO %s | "
            "Original recipient '%s' was not present. "
            "Nearest person '%s' (%.2fm away) accepted the delivery."
            % (alt_name.upper(), original_name, alt_name, distance)
        )
        self.status_pub.publish(status_msg)
        rospy.loginfo("[FIND_ALTERNATE] Status dipublish ke /delivery/alternate_status")

        # Set goal userdata ke posisi Origin untuk StateComplete → RETURN_HOME
        origin = self.database.get_position("Origin")
        if origin:
            userdata.goal_x     = origin[0]
            userdata.goal_y     = origin[1]
            userdata.goal_theta = origin[2]
        else:
            userdata.goal_x     = 0.0
            userdata.goal_y     = 0.0
            userdata.goal_theta = 0.0

        rospy.sleep(2.0)  # Jeda sebentar sebelum lanjut ke COMPLETE
        return 'found_alternate'


# ========================================
# STATE 5: COMPLETE
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
# MAIN
# ========================================
def main():
    rospy.init_node('delivery_state_machine')

    rospy.loginfo("=" * 60)
    rospy.loginfo("DELIVERY STATE MACHINE - Revisi Final + Face Verification")
    rospy.loginfo("States: IDLE -> NAVIGATING -> DELIVERING -> [FIND_ALTERNATE] -> COMPLETE -> RETURN_HOME")
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
            transitions={'delivered': 'COMPLETE', 'no_confirm': 'FIND_ALTERNATE'})

        smach.StateMachine.add('FIND_ALTERNATE', StateFindAlternate(database),
            transitions={
                'found_alternate'   : 'COMPLETE',
                'no_alternate'      : 'COMPLETE',
                'navigation_failed' : 'COMPLETE'
            })

        smach.StateMachine.add('COMPLETE', StateComplete(database),
            transitions={'return_home': 'RETURN_HOME'})

        smach.StateMachine.add('RETURN_HOME', StateNavigating(),
            transitions={'arrived': 'success', 'failed': 'failed', 'timeout': 'failed'})

    sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("State Machine ready.")
    rospy.loginfo("Set delivery target: rosparam set /delivery/recipient_name 'Pak Yani'")

    outcome = sm.execute()

    rospy.loginfo("STATE MACHINE FINISHED. Final outcome: %s", outcome)
    mission_tracker.print_statistics()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine interrupted.")
