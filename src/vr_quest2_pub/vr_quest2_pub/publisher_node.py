#!/usr/bin/env python3
import rclpy
import rclpy.logging
from rclpy.node import Node
from rclpy.time import Time
from piper_msgs.msg import PosCmd
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R
from vr_quest2_pub.encapsulated_oculusReader.oculus_data_jh import OculusInterface
from vr_quest2_pub.encapsulated_oculusReader.oculus_reader.reader import OculusReader
from vr_quest2_pub.encapsulated_oculusReader.precise_sleep import precise_wait
import time
import numpy as np
import math

class PublisherNode(Node):
    def __init__(self, output="data/", frequency=50):
        super().__init__('vr_quest2')
        # 订阅最新的机械臂末端位姿和关节状态
        self.end_pose_cal_sub = self.create_subscription(PoseStamped, 'current_pose', self.endPose_cal_callback, 10)
        self.end_pose_sub = self.create_subscription(Pose, 'end_pose', self.endPose_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, 'joint_states_single', self.joint_state_callback, 10)

        # 发布目标末端位姿和复位时的关节状态
        self.pos_cmd_pub = self.create_publisher(PosCmd, 'pos_cmd', 10)
        self.pos_cmd_pub_sim = self.create_publisher(PoseStamped, 'target_poses', 10)
        self.joint_publisher = self.create_publisher(JointState, 'joint_commands', 10)
        self.gripper_cmd_pub = self.create_publisher(Pose, 'gripper_cmd', 10)

        self.output = output
        self.frequency = frequency
        self.dt = 1.0 / self.frequency

        # 存储最新状态信息（修复初始化问题）
        self.current_end_pose = [0.0] * 7  # [x, y, z, qx, qy, qz, qw]
        self.current_end_pose_cal = [0.0] * 7  # [x, y, z, qx, qy, qz, qw]
        self.current_joint_state = [0.0] * 6  # 根据实际关节数调整

        # 命令变量
        self.commanded_pose = None

        # 初始化标志
        self.initialized = False
        self.baseline_pose = None

        # 复位相关变量
        self.reset_mode = False
        self.reset_start_time = None
        self.reset_duration = 2.0  # 复位持续时间（秒）

        self.timer = self.create_timer(self.dt, self.timer_callback)

        # VR 接口初始化
        self.oculus_reader = OculusReader()
        self.oculus_interface = OculusInterface(oculus_reader=OculusReader(), degree=False, filter=True,hz=self.frequency)

    def endPose_cal_callback(self, msg):
        self.current_end_pose_cal[0] = round(msg.pose.position.x * 1000) / 1000
        self.current_end_pose_cal[1] = round(msg.pose.position.y * 1000) / 1000 
        self.current_end_pose_cal[2] = round(msg.pose.position.z * 1000) / 1000 
        self.current_end_pose_cal[3] = round(msg.pose.orientation.x * 1000) / 1000 
        self.current_end_pose_cal[4] = round(msg.pose.orientation.y * 1000) / 1000 
        self.current_end_pose_cal[5] = round(msg.pose.orientation.z * 1000) / 1000 
        self.current_end_pose_cal[6] = round(msg.pose.orientation.w * 1000) / 1000 

    def endPose_callback(self, msg):
        # 更新末端位姿（修复索引错误）
        self.current_end_pose[0] = msg.position.x 
        self.current_end_pose[1] = msg.position.y
        self.current_end_pose[2] = msg.position.z
        self.current_end_pose[3] = msg.orientation.x
        self.current_end_pose[4] = msg.orientation.y
        self.current_end_pose[5] = msg.orientation.z
        self.current_end_pose[6] = msg.orientation.w

    def joint_state_callback(self, msg):
        # 更新所有关节状态（支持动态关节数）
        for i in range(min(len(msg.position), len(self.current_joint_state))):
            self.current_joint_state[i] = msg.position[i]
    def timer_callback(self):
        # print("in timer")
        # 初始化逻辑
        if not self.initialized:
            for v in self.current_end_pose_cal:
                print(v)
            if all(v != 0.0 for v in self.current_end_pose_cal):  # 等待有效数据

                self.commanded_pose = self.current_end_pose_cal.copy()
                self.initialized = True
            return

        # print("in timer 111")
        # self.commanded_pose = self.current_end_pose_cal.copy()
        # print(self.commanded_pose[0])
        # print(self.commanded_pose[1])
        # print(self.commanded_pose[2])
        # print(self.commanded_pose[3])
        # print(self.commanded_pose[4])
        # print(self.commanded_pose[5])
        # print(self.commanded_pose[6])
        # print("in timer 222")
        # 获取VR输入
        delta_actions, buttons = self.oculus_interface.get_action_delta()
        # print("in timer 333")
        # print(delta_actions[0][0])
        # print(delta_actions[0][1])
        # print(delta_actions[0][2])
        # print(delta_actions[0][3])
        # print(delta_actions[0][4])
        # print(delta_actions[0][5])
        # print(delta_actions[0][6])
        print("button pushed")
        print(buttons)
        B_button = buttons.get("B", False)
        grip_force = 0
        if (len(buttons) > 0) :
            Right_grip = buttons.get("rightGrip")
            grip_force = Right_grip[0]
        gripper_msg = Pose()
        gripper_msg.position.x = 0.035 - grip_force * 0.035
        gripper_msg.position.y = -0.035 + grip_force * 0.035
        self.gripper_cmd_pub.publish(gripper_msg)
        
        # 复位逻辑
        if B_button:
            self.reset_mode = True
            self.reset_start_time = time.monotonic()
        if self.reset_mode:
            # 发布复位指令并冻结其他控制
            self.joint_publisher.publish(self.reset_joint_state_command())
            
            # 检查复位是否完成
            if time.monotonic() - self.reset_start_time >= self.reset_duration:
                # 强制从最新末端位姿更新基准
                #self.commanded_pose = self.current_end_pose.copy()
                self.commanded_pose = self.current_end_pose_cal.copy()
                self.reset_mode = False
                print('Reset completed with latest end pose.')
            return

        # 正常控制模式
        A_button = buttons.get("A", False)
        if A_button:
            self.update_pos(delta_actions[0])
        else:
            self.freeze()
            return

        # 发布目标位姿
        pos_cmd_msg = self.warpPoseMsg_from_pose_sim(self.commanded_pose)
        self.pos_cmd_pub_sim.publish(pos_cmd_msg)

    def update_pos(self, delta_action):
        """更新指令位姿（增加幅度限制）"""
        # 位置增量（限制最大移动速度）
        
        for i in range(7):
            delta_action[i] = round(delta_action[i] * 1000) / 1000
        print("start")
        print(delta_action[0])
        print(delta_action[1])
        print(delta_action[2])
        print(delta_action[3])
        print(delta_action[4])
        print(delta_action[5])
        print(delta_action[6])
        # delta_action[0] = 0.000
        # delta_action[1] = 0.000
        # delta_action[2] = 0.001

        self.commanded_pose[0] += float(delta_action[0]) * 1.0
        self.commanded_pose[1] += float(delta_action[1]) * 1.0
        self.commanded_pose[2] += float(delta_action[2]) * 1.0

        curr_quat = self.commanded_pose[3:7]
        curr_rot = R.from_quat([curr_quat[0], curr_quat[1], curr_quat[2], curr_quat[3]]).as_matrix()
        print("Rotation Matrix:\n", curr_rot)
        delta_quat = delta_action[3:7]
        delta_rot = R.from_quat([delta_quat[1], delta_quat[0], delta_quat[2], delta_quat[3]]).as_matrix()
        delta_rot_fixed = delta_rot[:, [1, 0, 2]]
        print("Rotation Matrix 2:\n", delta_rot)
        print("Rotation Matrix 21:\n", delta_rot_fixed)

        euler_angles = R.from_matrix(delta_rot).as_euler('xyz')


        x_rotation = R.from_euler('x', euler_angles[0]).as_matrix()
        y_rotation = R.from_euler('y', euler_angles[1]).as_matrix()


        z_rotation = R.from_euler('z', euler_angles[2]).as_matrix()
        new_rotation = curr_rot @ delta_rot
        rotation = R.from_matrix(new_rotation)
        new_quat = rotation.as_quat()
        print("Rotation Matrix 3:\n", new_rotation)
        # new_quat = new_rotation.as_quat()
        # new_quat = self.quat_multiply(curr_quat, self.quat_inverse(delta_quat))
        self.commanded_pose[3:7] = new_quat
        print("start222")
        print(self.commanded_pose[0])
        print(self.commanded_pose[1])
        print(self.commanded_pose[2])
        print(self.commanded_pose[3])
        print(self.commanded_pose[4])
        print(self.commanded_pose[5])
        print(self.commanded_pose[6])

    def reset_joint_state_command(self):
        """
        构造并返回一个复位 JointState 消息：
          - header.frame_id 为 'piper_single'
          - name: ['joint1', ..., 'joint7']
          - position, velocity, effort 使用示例数据
        """
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = "piper_single"
        js.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
        js.position = [0.1, 1.6, -1.3, 0.0, 0.9, 1.1, 0.035, -0.035]
        js.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 30.0, 30.0]
        js.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10000.0, 10000.0]
        return js

    def freeze(self):
        """
        freeze 模式下不更新 commanded_pose（保持上次命令不变）
        """
        pass  # 无操作

    def warpPoseMsg_from_pose(self, pose_array):
        """
        根据给定的 7 元素位姿数组构造 PosCmd 消息：
          - 前 3 个元素为位置
          - 后 4 个元素为四元数（x,y,z,w），转换为欧拉角
        """
        msg = PosCmd()
        msg.x = pose_array[0]
        msg.y = pose_array[1]
        msg.z = pose_array[2]
        quat = pose_array[3:7]
        r = R.from_quat(quat)
        euler = r.as_euler('xyz', degrees=False)
        msg.roll = euler[0]
        msg.pitch = euler[1]
        msg.yaw = euler[2]
        msg.gripper = 0.01  # 保持为 float 类型
        return msg
    
    def warpPoseMsg_from_pose_sim(self, pose_array):
        """
        根据给定的 7 元素位姿数组构造 PosCmd 消息：
          - 前 3 个元素为位置
          - 后 4 个元素为四元数（x,y,z,w），转换为欧拉角
        """


        msg = PoseStamped()
        msg.header.frame_id = "map"  
        msg.header.stamp = self.get_clock().now().to_msg()
 
        msg.pose.position.x = float(pose_array[0])
        msg.pose.position.y = float(pose_array[1])
        msg.pose.position.z = float(pose_array[2])
        msg.pose.orientation.x = float(pose_array[3])
        msg.pose.orientation.y = float(pose_array[4])
        msg.pose.orientation.z = float(pose_array[5])
        msg.pose.orientation.w = float(pose_array[6])  

        return msg
    

    def quat_multiply(self, quaternion1, quaternion0):
        """返回两个四元数乘积，四元数顺序为 (x, y, z, w)。"""
        x0, y0, z0, w0 = quaternion0
        x1, y1, z1, w1 = quaternion1
        return np.array([
            x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
            -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
            x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0,
            -x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
        ])
    
    def quat_conjugate(self, q):
        """返回四元数 q (x, y, z, w) 的共轭。"""
        return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float32)

    def quat_inverse(self, q):
        """返回四元数 q (x, y, z, w) 的逆，即共轭除以模平方。"""
        return self.quat_conjugate(q) / np.dot(q, q)
    

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()