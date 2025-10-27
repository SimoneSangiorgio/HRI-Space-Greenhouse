#!/usr/bin/env python

import rospy
import json
from std_msgs.msg import String
import threading
import time

class DiagnosticsNode:
    def __init__(self):
        rospy.loginfo("Initializing Diagnostics Node...")
        self.serra_state = {}
        self.robot_sensors = {}
        self.data_lock = threading.Lock()

        rospy.Subscriber("/serra/state", String, self.serra_state_callback)
        rospy.Subscriber("/robot/sensors", String, self.robot_sensors_callback)
        
        rospy.loginfo("Subscribers created. DiagnosticsNode is running.")

    def serra_state_callback(self, msg):
        with self.data_lock:
            self.serra_state = json.loads(msg.data)

    def robot_sensors_callback(self, msg):
        with self.data_lock:
            self.robot_sensors = json.loads(msg.data)

    # Getter methods for the MainController to use
    def get_serra_state(self):
        with self.data_lock:
            return self.serra_state.copy()

    def get_robot_sensors(self):
        with self.data_lock:
            return self.robot_sensors.copy()