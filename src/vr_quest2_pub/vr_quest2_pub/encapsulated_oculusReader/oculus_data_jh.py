#!/usr/bin/env python3
"""
OculusInterface

This code continuously collects data from an OculusReader into thread-safe buffers.
Each call to get_action() computes the incremental change in pose between the two latest samples,
returning a 7D vector:
  [dx, dy, dz, qx, qy, qz, qw]
where:
  - dx, dy, dz: incremental position change in meters.
  - qx, qy, qz, qw: incremental rotation as a quaternion.

It also returns the latest button state (a dictionary) aligned in time with the pose.

Functions:
    - get_action_delta(): Get the latest incremental pose change and button state.
    - get_action(): Get the latest pose and button state.
"""

import time
import numpy as np
import math
import threading
from collections import deque
from .oculus_reader.reader import OculusReader
from scipy.spatial.transform import Rotation
class OculusInterface:
    def __init__(self, oculus_reader, max_buffer=10, hz=60, degree=True,filter=False, alpha_pos=0.5, alpha_rot=0.5):
        """
        Initialize the OculusInterface.
        Args:
            oculus_reader: An instance of OculusReader.
            max_buffer: Maximum number of samples to keep in the buffers.
            hz: Frequency (Hz) at which to read data from OculusReader.
        """
        self.hz = hz    
        self.oculus = oculus_reader
        self.degree = degree # Use degree or radian
        self.filter = filter
        self.alpha_pos = alpha_pos
        self.alpha_rot = alpha_rot
        # Thread-safe data buffers.
        self.pose_buffer = deque(maxlen=max_buffer)
        self.button_buffer = deque(maxlen=max_buffer)
        self.lock = threading.Lock()
        # Filter states
        if self.filter:
            self.filtered_r_pos = None
            self.filtered_r_quat = None
            self.filtered_l_pos = None
            self.filtered_l_quat = None
        # Start data collection thread.
        self.running = True
        self.read_thread = threading.Thread(target=self._read_data)
        self.read_thread.daemon = True
        self.read_thread.start()

    def _read_data(self):
        """
        Continuously read data from OculusReader and store it in buffers.
        Apply EMA filtering if enabled.
        """
        while self.running:
            poses, buttons = self.oculus.get_transformations_and_buttons()
            if not poses:
                print("No poses received.")
                time.sleep(0.1)
                continue

            # Process right controller
            pose_r = poses.get("r")
            tracked_r = pose_r is not None
            if tracked_r:
                # Extract translation (position)
                pos_r = pose_r[:3, 3].copy()
                pos_r = pos_r[[2, 0, 1]]
                pos_r[0] = -pos_r[0]
                pos_r[1] = -pos_r[1]
                rot_mat_r = pose_r[:3, :3]
                if self.degree:
                    euler_r = Rotation.from_matrix(rot_mat_r).as_euler('xyz', degrees=True)
                    quat_r = Rotation.from_euler('xyz', euler_r, degrees=True).as_quat()
                else:
                    euler_r = Rotation.from_matrix(rot_mat_r).as_euler('xyz')
                    quat_r = Rotation.from_euler('xyz', euler_r).as_quat()
      
                quat_r = quat_r[[0, 1, 2, 3]]

                if quat_r[3] < 0.0:
                    quat_r *= -1.0
                pose_7d_r = np.hstack([pos_r, quat_r])
            else:
                pose_7d_r = np.zeros(7, dtype=np.float32)

            # Process left controller
            pose_l = poses.get("l")
            tracked_l = pose_l is not None
            if tracked_l:
                pos_l = pose_l[:3, 3].copy()
                pos_l = pos_l[[2, 0, 1]]
                pos_l[0] = -pos_l[0]
                pos_l[1] = -pos_l[1]
                rot_mat_l = pose_l[:3, :3]
                if self.degree:
                    euler_l = Rotation.from_matrix(rot_mat_l).as_euler('xyz', degrees=True)
                    quat_l = Rotation.from_euler('xyz', euler_l, degrees=True).as_quat()
                else:
                    euler_l = Rotation.from_matrix(rot_mat_l).as_euler('xyz')
                    quat_l = Rotation.from_euler('xyz', euler_l).as_quat()

                quat_l = quat_l[[0, 1, 2, 3]]

                if quat_l[3] < 0.0:
                    quat_l *= -1.0
                pose_7d_l = np.hstack([pos_l, quat_l])
            else:
                pose_7d_l = np.zeros(7, dtype=np.float32)

            # Apply filtering if enabled
            if self.filter:
                # Process right controller
                if tracked_r:
                    current_r_pos = pose_7d_r[:3]
                    current_r_quat = pose_7d_r[3:]
                    if self.filtered_r_pos is None:
                        self.filtered_r_pos = current_r_pos.copy()
                        self.filtered_r_quat = current_r_quat.copy()
                    else:
                        # EMA for position
                        self.filtered_r_pos = self.alpha_pos * current_r_pos + (1 - self.alpha_pos) * self.filtered_r_pos
                        # EMA for quaternion with sign correction
                        prev_q = self.filtered_r_quat.copy()
                        curr_q = current_r_quat.copy()
                        dot_product = np.dot(prev_q, curr_q)
                        if dot_product < 0.0:
                            curr_q = -curr_q
                        interpolated_q = (1 - self.alpha_rot) * prev_q + self.alpha_rot * curr_q
                        interpolated_q /= np.linalg.norm(interpolated_q)
                        self.filtered_r_quat = interpolated_q
                    # Update pose with filtered values
                    pose_7d_r = np.hstack([self.filtered_r_pos, self.filtered_r_quat])

                # Process left controller
                if tracked_l:
                    current_l_pos = pose_7d_l[:3]
                    current_l_quat = pose_7d_l[3:]
                    if self.filtered_l_pos is None:
                        self.filtered_l_pos = current_l_pos.copy()
                        self.filtered_l_quat = current_l_quat.copy()
                    else:
                        # EMA for position
                        self.filtered_l_pos = self.alpha_pos * current_l_pos + (1 - self.alpha_pos) * self.filtered_l_pos
                        # EMA for quaternion with sign correction
                        prev_q = self.filtered_l_quat.copy()
                        curr_q = current_l_quat.copy()
                        dot_product = np.dot(prev_q, curr_q)
                        if dot_product < 0.0:
                            curr_q = -curr_q
                        interpolated_q = (1 - self.alpha_rot) * prev_q + self.alpha_rot * curr_q
                        interpolated_q /= np.linalg.norm(interpolated_q)
                        self.filtered_l_quat = interpolated_q
                    # Update pose with filtered values
                    pose_7d_l = np.hstack([self.filtered_l_pos, self.filtered_l_quat])

            pose_2x7 = np.stack([pose_7d_r, pose_7d_l], axis=0)

            with self.lock:
                self.pose_buffer.append(pose_2x7)
                self.button_buffer.append(buttons)
            
            time.sleep(1.0 / self.hz)

    def reset_filter(self):
        """Reset the filter states to None."""
        if self.filter:
            self.filtered_r_pos = None
            self.filtered_r_quat = None
            self.filtered_l_pos = None
            self.filtered_l_quat = None
            
    def flush_buffers(self):
        """Clear all old samples and keep only the latest one."""
        with self.lock:
            if len(self.pose_buffer) > 0:
                last_pose = self.pose_buffer[-1]
                self.pose_buffer.clear()
                self.pose_buffer.append(last_pose)
            if len(self.button_buffer) > 0:
                last_buttons = self.button_buffer[-1]
                self.button_buffer.clear()
                self.button_buffer.append(last_buttons)

    # def get_action_delta(self):
    #     """
    #     返回 (2,7) 的数组，分别为左右手的增量 [dx, dy, dz, qx, qy, qz, qw]，
    #     以及与该增量对应的最新 button 状态。
    #     """
    #     with self.lock:
    #         if len(self.pose_buffer) == 0:
    #             # No pose data available at all.
    #             buttons = {}
    #             return np.array([[0, 0, 0, 0, 0, 0, 1],[0, 0, 0, 0, 0, 0, 1]], dtype=np.float32), buttons
    #         elif len(self.pose_buffer) < 2:
    #             # Only one sample exists; use it for both previous and current.
    #             pose_prev = self.pose_buffer[-1].copy()
    #             pose_curr = self.pose_buffer[-1].copy()
    #             buttons = self.button_buffer[-1].copy()
    #         else:
    #             # Use the two most recent samples.
    #             pose_prev = self.pose_buffer[-2].copy()
    #             pose_curr = self.pose_buffer[-1].copy()
    #             buttons = self.button_buffer[-1].copy()

    #     # 分别计算右手(0)和左手(1)的增量
    #     delta_poses = []
    #     for i in range(2):
    #         pos_prev = pose_prev[i, :3]
    #         pos_curr = pose_curr[i, :3]
    #         delta_pos = np.clip(pos_curr - pos_prev, -0.1, 0.1)

    #         quat_prev = pose_prev[i, 3:]
    #         quat_curr = pose_curr[i, 3:]
    #         delta_quat = self.quat_multiply(quat_curr, self.quat_inverse(quat_prev))

    #         delta_poses.append(np.hstack([delta_pos, delta_quat]))

    #     return np.stack(delta_poses, axis=0).astype(np.float32), buttons
    def get_action_delta(self):
        """
        返回 (2,7) 的数组，分别为左右手的增量 [dx, dy, dz, qx, qy, qz, qw]。
        """
        with self.lock:
            if len(self.pose_buffer) == 0:
                buttons = {}
                zero_delta = np.array([[0,0,0,0,0,0,1], [0,0,0,0,0,0,1]], dtype=np.float32)
                return zero_delta, buttons
            elif len(self.pose_buffer) < 2:
                pose_prev = pose_curr = self.pose_buffer[-1].copy()
                buttons = self.button_buffer[-1].copy()
            else:
                pose_prev = self.pose_buffer[-2].copy()
                pose_curr = self.pose_buffer[-1].copy()
                buttons = self.button_buffer[-1].copy()

        delta_poses = []
        for i in range(2):
            pos_prev = pose_prev[i, :3]
            pos_curr = pose_curr[i, :3]
            pos_increment = pos_curr - pos_prev
            delta_pos = np.clip(pos_increment, -0.1, 0.1)

            quat_prev = pose_prev[i, 3:]
            quat_curr = pose_curr[i, 3:]
            delta_quat = self.quat_multiply(quat_curr, self.quat_inverse(quat_prev))

            delta_poses.append(np.hstack([delta_pos, delta_quat]))

        return np.stack(delta_poses, axis=0).astype(np.float32), buttons

    def get_action(self):
        """
        返回 (2,7) 的数组，分别为右手和左手 Pose，以及对应的 buttons。
        """
        with self.lock:
            if len(self.pose_buffer) == 0:
                # No pose data available at all.
                return None,None
            else:
                # Use the two most recent samples.
                pose_curr = self.pose_buffer[-1].copy()
                buttons = self.button_buffer[-1].copy()

        return pose_curr.astype(np.float32), buttons

    def close(self):
        """Stop the data collection thread and close the OculusReader."""
        self.running = False
        self.read_thread.join(timeout=1)
        self.oculus.stop()

    # --- Helper functions for quaternion and matrix operations ---
    def quat_multiply(self, q1, q0):
        """
        Multiply two quaternions (order: q1 * q0) where each is (x, y, z, w).
        """
        x0, y0, z0, w0 = q0
        x1, y1, z1, w1 = q1
        return np.array([
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0
        ], dtype=np.float32)

    def quat_conjugate(self, q):
        """Return the conjugate of quaternion q (x, y, z, w)."""
        return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float32)

    def quat_inverse(self, q):
        """Return the inverse of quaternion q (x, y, z, w)."""
        return self.quat_conjugate(q) / np.dot(q, q)


# --- Test code mapping the increments to a robot target state ---
def main():
    """
    Test routine:
      - Creates an OculusReader and wraps it in OculusInterface.
      - In a loop (10 Hz), obtains the incremental pose change and button info.
      - Maps the increments onto a simulated robot target state (position in meters and rotation in Euler angles in degrees).
    """
    oculus_reader = OculusReader()
    oculus_interface = OculusInterface(oculus_reader)
    
    # Initialize the robot's target state.
    robot_position = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    robot_euler = np.array([0.0, 0.0, 0.0], dtype=np.float32)  # roll, pitch, yaw in degrees
    
    print("Starting OculusInterface test loop (Press Ctrl+C to exit)...")
    
    try:
        while True:
            # Get the incremental action and button state.
            # action is [dx, dy, dz, qx, qy, qz, qw]
            action, buttons = oculus_interface.get_action()
            print("action:", action)
            print("buttons:", buttons)
            delta_action, buttons = oculus_interface.get_action_delta()
            print("delta_action:", delta_action)
            print("buttons:", buttons)
            time.sleep(0.1)  # Loop at approximately 10 Hz.
    except KeyboardInterrupt:
        print("Program terminated by user.")
    finally:
        oculus_interface.close()

if __name__ == "__main__":
    main()
