#!/usr/bin/env python3

"""
Author: Chistiaan Peppelman
Updated: 08-06-2026

This script is used to run the data record button. When using this script for your application ensure to edit the self.topics and rospy.Subscriber definitions of the main DatasetRecorder class to be coherent with those of your project.

Use of AI: LLM were used to assist the author in writing this script.
"""

import rospy
from sensor_msgs.msg import Joy
import subprocess
import signal
import os
import time
from datetime import datetime


class DatasetRecorder:
    def __init__(self):
        self.recording = False
        self.converting = False
        self.process = None
        self.previous_button_state = 0
        self.current_bag_path = None

        
        self.trigger_button_index = 9

        
        self.output_dir = os.path.expanduser("~/ros_data")
        os.makedirs(self.output_dir, exist_ok=True)

        
        self.script_dir = os.path.dirname(os.path.abspath(__file__))

        
        self.converter_script = os.path.join(
            self.script_dir,
            "algorithmv1.py"
        )

        self.topics = [
            "/camera/color/image_raw/compressed",
            "/dingo1/dinova/joint_states",
            "/tf",
            "/tf_static"
        ]

        rospy.Subscriber(
            "/dingo1/bluetooth_teleop/joy",
            Joy,
            self.joy_callback
        )

        rospy.loginfo("Recorder ready.")
        rospy.loginfo("Press the option button to start/stop recording.")
        rospy.loginfo(f"Bags are saved in: {self.output_dir}")
        rospy.loginfo(f"Converter script: {self.converter_script}")

    def joy_callback(self, msg):
        if len(msg.buttons) <= self.trigger_button_index:
            rospy.logwarn(
                f"Button index {self.trigger_button_index} does not exist in Joy message."
            )
            return

        current_button_state = msg.buttons[self.trigger_button_index]

        
        if current_button_state == 1 and self.previous_button_state == 0:

            if self.converting:
                rospy.logwarn("Conversion is still running. Button press ignored.")
                self.previous_button_state = current_button_state
                return

            if not self.recording:
                self.start_recording()
            else:
                self.stop_recording()
                self.run_conversion(self.current_bag_path)

        self.previous_button_state = current_button_state

    def start_recording(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        self.current_bag_path = os.path.join(
            self.output_dir,
            f"session_{timestamp}.bag"
        )

        cmd = [
            "rosbag",
            "record",
            "-O",
            self.current_bag_path
        ] + self.topics

        rospy.loginfo("Starting rosbag recording...")
        rospy.loginfo(" ".join(cmd))

        self.process = subprocess.Popen(cmd)
        self.recording = True

    def stop_recording(self):
        rospy.loginfo("Stopping rosbag recording...")

        if self.process is not None:
            self.process.send_signal(signal.SIGINT)

            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                rospy.logwarn("rosbag did not stop after SIGINT. Terminating process...")
                self.process.terminate()
                self.process.wait()

            self.process = None

        self.recording = False
        rospy.loginfo(f"Bag saved at: {self.current_bag_path}")

    def wait_for_bag_file(self, bag_path, timeout_seconds=15):
        """
        Wait until rosbag has finished creating the final .bag file.
        Sometimes rosbag briefly uses a .bag.active file before the final .bag appears.
        """

        rospy.loginfo("Checking if bag file is ready...")

        start_time = time.time()
        active_path = bag_path + ".active"

        while time.time() - start_time < timeout_seconds:
            if os.path.exists(bag_path):
                rospy.loginfo(f"Bag file found: {bag_path}")
                return True

            if os.path.exists(active_path):
                rospy.loginfo(f"Bag file still active: {active_path}")
            else:
                rospy.loginfo("Waiting for bag file to appear...")

            time.sleep(0.5)

        rospy.logerr(f"Bag file not found after waiting: {bag_path}")

        if os.path.exists(active_path):
            rospy.logerr(f"Found active bag file instead: {active_path}")
            rospy.logerr("This means rosbag may not have closed cleanly.")

        return False

    def run_conversion(self, bag_path):
        if not os.path.exists(self.converter_script):
            rospy.logerr(f"Converter script not found: {self.converter_script}")
            return

        if bag_path is None:
            rospy.logerr("No bag path available. Cannot run conversion.")
            return

        
        if not self.wait_for_bag_file(bag_path):
            return

        rospy.loginfo("Starting automatic Zarr conversion...")
        rospy.loginfo(f"Converting bag file: {bag_path}")

        self.converting = True

        cmd = [
            "python3",
            self.converter_script,
            bag_path
        ]

        rospy.loginfo(" ".join(cmd))

        result = subprocess.run(cmd)

        if result.returncode == 0:
            rospy.loginfo("Zarr conversion finished successfully.")
        else:
            rospy.logerr(f"Zarr conversion failed with return code: {result.returncode}")

        self.converting = False


if __name__ == "__main__":
    rospy.init_node("dataset_recorder")
    recorder = DatasetRecorder()
    rospy.spin()
