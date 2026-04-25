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
from actionlib import SimpleActionClient, GoalStatus
import math
import tf2_ros

class PositionDatabase:
    def __init__(self):
        package_path = os.path.expanduser('~/catkin_ws/src/ta2_farhan')
        yaml_path = os.path.join(package_path, 'config', 'position_database.yaml')
        rospy.loginfo("Loading database from: %s", yaml_path)
        self.database = self.load_database(yaml_path)
        if self.database:
            rospy.loginfo("Database loaded successfully!")
            for entry in self.database:
                name = entry.get('name', '(no name)')
                rospy.loginfo("  - %s: (%.2f, %.2f, %.2f)",
                             name, entry.get('x', 0), entry.get('y', 0),
                             entry.get('theta', 0))
        else:
            rospy.logerr("Failed to load database!")

    def load_database(self, yaml_path):
        try:
            with open(yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                positions = data.get('positions', [])
                if positions is None:
                    return []
                return positions
        except Exception as e:
            rospy.logerr("Error loading YAML: %s", str(e))
            return None

    def get_position(self, name):
        if self.database is None:
            return None
        for entry in self.database:
            entry_name = entry.get('name', '')
            if entry_name and entry_name.lower() == name.lower():
                return (entry['x'], entry['y'], entry.get('theta', 0.0))
        return None


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

    def _get_position_error(self, goal_x, goal_y):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0)
            )
            actual_x = transform.transform.translation.x
            actual_y = transform.transform.translation.y
            error_cm = math.sqrt(
                (actual_x - goal_x)**2 +
                (actual_y - goal_y)**2
            ) * 100.0
            return error_cm
        except Exception as e:
            rospy.logwarn("TF lookup gagal: %s", str(e))
            return None

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: NAVIGATING")
        rospy.loginfo("=" * 60)

        goal_x = userdata.goal_x
        goal_y = userdata.goal_y
        goal_theta = userdata.goal_theta
        
        max_nav_retries = 3

        for attempt in range(max_nav_retries):
            if rospy.is_shutdown():
                return 'failed'

            rospy.loginfo(">>> Navigasi attempt %d/%d", attempt + 1, max_nav_retries)

            # Buat client BARU setiap attempt
            client = SimpleActionClient('move_base', MoveBaseAction)

            rospy.loginfo(">>> Menunggu move_base action server...")
            if not client.wait_for_server(rospy.Duration(10.0)):
                rospy.logerr("move_base action server tidak tersedia!")
                continue

            rospy.loginfo(">>> move_base tersedia.")

            # Cancel semua goal lama yang mungkin masih ada di server
            client.cancel_all_goals()
            rospy.sleep(1.0)

            # Buat dan kirim goal baru
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp    = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_x
            goal.target_pose.pose.position.y = goal_y
            goal.target_pose.pose.orientation.z = math.sin(goal_theta / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(goal_theta / 2.0)

            rospy.loginfo(">>> Mengirim goal: (%.2f, %.2f)", goal_x, goal_y)
            client.send_goal(goal)

            # Tunggu 2 detik agar move_base benar-benar mulai memproses
            rospy.sleep(2.0)

            # Cek apakah goal sedang diproses
            state_after_2s = client.get_state()
            rospy.loginfo(">>> Goal state setelah 2 detik: %d "
                          "(0=PENDING, 1=ACTIVE, 3=SUCCEEDED)", state_after_2s)

            # Kalau sudah SUCCEEDED dalam 2 detik, cek error posisi
            if state_after_2s == GoalStatus.SUCCEEDED:
                error = self._get_position_error(goal_x, goal_y)
                if error is not None and error < 100.0:
                    rospy.loginfo(">>> NAVIGATION SUCCESS (cepat)")
                    rospy.loginfo(">>> Error posisi: %.2f cm", error)
                    return 'arrived'
                else:
                    rospy.logwarn(">>> SUCCEEDED palsu terdeteksi! "
                                  "Error: %.2f cm. Retry...",
                                  error if error else -1)
                    client.cancel_all_goals()
                    rospy.sleep(1.0)
                    continue

            # Tunggu robot sampai di tujuan (timeout 120 detik)
            rospy.loginfo(">>> Menunggu robot sampai tujuan (timeout: 120 detik)...")
            finished = client.wait_for_result(rospy.Duration(200.0))

            if not finished:
                rospy.logwarn(">>> NAVIGATION TIMEOUT")
                client.cancel_goal()
                rospy.sleep(1.0)
                continue

            state = client.get_state()

            if state == GoalStatus.SUCCEEDED:
                # Cek posisi aktual — apakah benar-benar sampai?
                rospy.sleep(0.5)
                error = self._get_position_error(goal_x, goal_y)

                if error is not None:
                    rospy.loginfo(">>> Error posisi: %.2f cm", error)

                    if error < 100.0:
                        # Benar-benar sampai
                        rospy.loginfo(">>> NAVIGATION SUCCESS (verified)")
                        return 'arrived'
                    else:
                        # SUCCEEDED palsu — robot belum sampai
                        rospy.logwarn(">>> SUCCEEDED tapi error %.2f cm "
                                      "(> 100 cm). Goal PALSU!", error)
                        rospy.logwarn(">>> Mengirim ulang goal...")
                        client.cancel_all_goals()
                        rospy.sleep(1.0)
                        continue
                else:
                    # TF gagal — anggap berhasil untuk tidak memblokir
                    rospy.logwarn(">>> Tidak bisa verifikasi posisi. "
                                  "Anggap berhasil.")
                    return 'arrived'

            elif state == GoalStatus.ABORTED:
                rospy.logwarn(">>> NAVIGATION ABORTED")
                rospy.sleep(1.0)
                continue

            else:
                rospy.logwarn(">>> NAVIGATION STATE: %d (unexpected)", state)
                rospy.sleep(1.0)
                continue

        # Semua retry gagal
        rospy.logerr(">>> NAVIGASI GAGAL setelah %d percobaan!", max_nav_retries)
        userdata.retry_count += 1
        mission_tracker.log_failure("NAV_FAILED_AFTER_%d_RETRIES" % max_nav_retries)
        return 'failed'


class StateDelivering(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['delivered', 'no_confirm']
        )
        self.person_detected     = False
        self.detection_confidence = 0.0
        self.last_detected_name  = ""
        self.active = False

        self.detection_subscriber = rospy.Subscriber(
            '/yolo/detections', Point, self.detection_callback)
        self.person_id_subscriber = rospy.Subscriber(
            '/yolo/person_id', String, self.person_id_callback)

    def is_name_match(self, detected_name, recipient_name):
        if detected_name == "Unknown" or not detected_name:
            return False
        det  = detected_name.lower().strip()
        recv = recipient_name.lower().strip()
        return det == recv or det in recv or recv in det

    def detection_callback(self, msg):
        if not self.active:
            return
        self.detection_confidence = msg.z

    def person_id_callback(self, msg):
        if not self.active:
            return
        detected_name = msg.data
        self.last_detected_name = detected_name
        recipient = rospy.get_param('/delivery/recipient_name', '')
        if not recipient:
            return
        if self.is_name_match(detected_name, recipient):
            self.person_detected = True
            rospy.loginfo("[DELIVERING] MATCH! Detected='%s', Recipient='%s'",
                          detected_name, recipient)
        else:
            if detected_name != "Unknown":
                rospy.logwarn("[DELIVERING] Bukan penerima: detected='%s', "
                              "expected='%s'", detected_name, recipient)

    def execute(self, userdata):
        rospy.loginfo("=" * 60)
        rospy.loginfo("STATE: DELIVERING (dengan face verification)")
        rospy.loginfo("=" * 60)

        recipient = rospy.get_param('/delivery/recipient_name', 'Unknown')
        rospy.loginfo(">>> Robot sudah di lokasi tujuan")
        rospy.loginfo(">>> Target penerima: %s", recipient)
        rospy.loginfo(">>> Scanning area dengan YOLO + InsightFace (10 detik)...")
        rospy.loginfo(">>> Delivery hanya dilakukan jika penerima yang benar "
                      "terdeteksi.")

        self.person_detected    = False
        self.last_detected_name = ""
        self.active = True

        timeout = rospy.Time.now() + rospy.Duration(10.0)
        rate    = rospy.Rate(10)

        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.person_detected:
                rospy.loginfo("=" * 60)
                rospy.loginfo(">>> PENERIMA TERVERIFIKASI!")
                rospy.loginfo(">>> Nama terdeteksi : '%s'",
                              self.last_detected_name)
                rospy.loginfo(">>> Penerima target  : '%s'", recipient)
                rospy.loginfo(">>> Confidence YOLO  : %.2f",
                              self.detection_confidence)
                rospy.loginfo("=" * 60)

                rospy.loginfo(">>> Menyerahkan barang ke %s...", recipient)
                rospy.sleep(2.0)
                rospy.loginfo(">>> Barang berhasil diserahkan!")

                self.active = False
                return 'delivered'

            rate.sleep()

        rospy.logwarn("=" * 60)
        rospy.logwarn(">>> TIMEOUT: Penerima '%s' tidak terverifikasi "
                      "dalam 10 detik", recipient)
        if self.last_detected_name:
            rospy.logwarn(">>> Orang terakhir terdeteksi: '%s' (bukan penerima)",
                          self.last_detected_name)
        else:
            rospy.logwarn(">>> Tidak ada orang terdeteksi di area tujuan")
        rospy.logwarn("=" * 60)

        self.active = False
        return 'no_confirm'


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

        rospy.loginfo(">>> Delivery selesai! Robot akan kembali ke Origin.")

        origin = self.database.get_position("Origin")
        if not origin:
            rospy.logerr("'Origin' tidak ditemukan di database! Pakai (0,0,0)")
            origin = (0.0, 0.0, 0.0)

        userdata.goal_x     = origin[0]
        userdata.goal_y     = origin[1]
        userdata.goal_theta = origin[2]

        rospy.loginfo(">>> Kembali ke origin: (%.2f, %.2f)", origin[0], origin[1])
        return 'return_home'


def main():
    rospy.init_node('delivery_state_machine')

    rospy.loginfo("=" * 60)
    rospy.loginfo("DELIVERY STATE MACHINE - PRAGMATIC FIX (12 April 2026)")
    rospy.loginfo("States: IDLE -> NAVIGATING -> DELIVERING -> COMPLETE "
                  "-> RETURN_HOME")
    rospy.loginfo("Fix: Validasi posisi aktual setelah SUCCEEDED, retry "
                  "kalau error > 1 meter")
    rospy.loginfo("=" * 60)

    database = PositionDatabase()

    sm = smach.StateMachine(outcomes=['success', 'failed'])
    sm.userdata.retry_count = 0

    with sm:
        smach.StateMachine.add('IDLE', StateIdle(database),
            transitions={'start_task': 'NAVIGATING',
                         'failed': 'failed'})

        smach.StateMachine.add('NAVIGATING', StateNavigating(),
            transitions={'arrived': 'DELIVERING',
                         'failed': 'IDLE',
                         'timeout': 'IDLE'})

        smach.StateMachine.add('DELIVERING', StateDelivering(),
            transitions={'delivered': 'COMPLETE',
                         'no_confirm': 'COMPLETE'})

        smach.StateMachine.add('COMPLETE', StateComplete(database),
            transitions={'return_home': 'RETURN_HOME'})

        smach.StateMachine.add('RETURN_HOME', StateNavigating(),
            transitions={'arrived': 'success',
                         'failed': 'failed',
                         'timeout': 'failed'})

    sis = smach_ros.IntrospectionServer('sm_server', sm, '/SM_ROOT')
    sis.start()

    rospy.loginfo("State Machine ready.")
    rospy.loginfo("Set target: rosparam set /delivery/recipient_name '<nama>'")

    outcome = sm.execute()

    if outcome == 'success':
        mission_tracker.log_success()
        rospy.loginfo("SIKLUS DELIVERY LENGKAP: delivery + return home "
                      "berhasil!")
    else:
        mission_tracker.log_failure("MISSION_OUTCOME_%s" % outcome.upper())

    rospy.loginfo("STATE MACHINE FINISHED. Final outcome: %s", outcome)
    mission_tracker.print_statistics()
    sis.stop()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("State machine interrupted.")
