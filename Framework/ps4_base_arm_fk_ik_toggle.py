#!/usr/bin/env python3

"Full code showcasing mapping of the singular controller. Forward and Inverse kinematics features are shown below. "
"This was made and finalised June 8th 2026 by Sitor Sla with assistence of Anthropic's Claude and OpenAI's ChatGPT. "

import time
import numpy as np
import rospy

from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger



# ROS topics en services
JOY_TOPIC = "/dingo1/bluetooth_teleop/joy"
JOINT_STATE_TOPIC = "/dingo1/kinova/joint_states"
COMMAND_TOPIC = "/dingo1/kinova/command"

HLC_VELOCITY_SERVICE = "/dingo1/kinova/change_to_HLC_velocity"
GRIPPER_OPEN_SERVICE = "/dingo1/kinova/gripper/open"
GRIPPER_CLOSE_SERVICE = "/dingo1/kinova/gripper/close"
DEFAULT_SERVICE = "/dingo1/kinova/go_default_position"
HOME_SERVICE = "/dingo1/kinova/go_home_position"
ZERO_SERVICE = "/dingo1/kinova/go_zero_position"
START_SERVICE = "/dingo1/kinova/go_start_position"



# Controller mapping
MODE_TOGGLE_BUTTON = 5       
FK_IK_TOGGLE_BUTTON = 10     

OPEN_BUTTON = 0              
CLOSE_BUTTON = 1             
DEFAULT_BUTTON = 2           
HOME_BUTTON = 3              
ZERO_BUTTON = 6              
START_BUTTON = 7             

# Assen
AXIS_LEFT_LR = 1             
AXIS_LEFT_UD = 0            
AXIS_RIGHT_LR = 3           
AXIS_RIGHT_UD = 4           

# D-pad/buttons voor FK joints 5 en 6
J5_POS_BUTTON = 12
J5_NEG_BUTTON = 11
J6_POS_BUTTON = 14
J6_NEG_BUTTON = 13



# Control parametersy

PUBLISH_RATE = 40.0
DEADZONE = 0.10

# FK joint velocity schaal
FK_SCALE = 0.40              # rad/s
multiplier = 1
# IK Cartesian snelheidsschaling in m/s
SPEED_X = 0.08 * multiplier              
SPEED_Y = 0.08  * multiplier              
SPEED_Z = 0.06   * multiplier             
SPEED_RZ = 0.20   * multiplier            
MAX_LINEAR_SPEED = 0.08     * multiplier  
MAX_ANGULAR_SPEED = 0.25   * multiplier   


DAMPING = 1e-3 * multiplier
MAX_ANGVEL = 0.30            
JOINT_WEIGHTS = np.array([1.0, 1.0, 1.2, 1.8, 14.0, 20.0], dtype=float)

ORI_GAIN = 2.0
TOOL_TWIST_GAIN = 4.0
POS_ERROR_DEADZONE = 0.004
ORI_ERROR_DEADZONE = 0.015
DQ_MIN_THRESHOLD = 0.001

JOINT_LIMITS = np.array([
    [-2.76,  2.76],
    [-2.76,  2.76],
    [-2.76,  2.76],
    [-2.67,  2.67],
    [-2.67,  2.67],
    [-2.67,  2.67],
])





_JOINT_ORIGINS = [
    (0,       0,     0.12825,  0,       0,       0),
    (0,      -0.03,  0.115,    1.5708,  0,       0),
    (0,       0.28,  0,       -3.1416,  0,       0),
    (0,      -0.14,  0.02,     1.5708,  0,       0),
    (0.0285,  0,     0.105,    0,       1.5708,  0),
    (-0.105,  0,     0.0285,   0,      -1.5708,  0),
]
_EE_OFFSET = np.array([0.0, 0.0, 0.13])


def _Rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _Ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def _T_fixed(tx, ty, tz, rx, ry, rz):
    R = _Rx(rx) @ _Ry(ry) @ _Rz(rz)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [tx, ty, tz]
    return T


def _T_revolute(theta):
    T = np.eye(4)
    T[:3, :3] = _Rz(theta)
    return T


def forward_kinematics(q):
    T = np.eye(4)
    for i, origin in enumerate(_JOINT_ORIGINS):
        T = T @ _T_fixed(*origin) @ _T_revolute(q[i])

    T_ee = np.eye(4)
    T_ee[:3, 3] = _EE_OFFSET
    T = T @ T_ee
    return T[:3, 3].copy(), T[:3, :3].copy()


def jacobian(q):
    J = np.zeros((6, 6))
    T = np.eye(4)
    origins = []
    z_axes = []

    for i, origin in enumerate(_JOINT_ORIGINS):
        T = T @ _T_fixed(*origin) @ _T_revolute(q[i])
        origins.append(T[:3, 3].copy())
        z_axes.append(T[:3, 2].copy())

    T_ee = np.eye(4)
    T_ee[:3, 3] = _EE_OFFSET
    p_ee = (T @ T_ee)[:3, 3]

    for i in range(6):
        z = z_axes[i]
        o = origins[i]
        J[:3, i] = np.cross(z, p_ee - o)
        J[3:, i] = z

    return J


def weighted_dls(J, error, joint_weights, damping):
    W_inv = np.diag(1.0 / joint_weights)
    A = J @ W_inv @ J.T + damping * np.eye(J.shape[0])
    dq = W_inv @ J.T @ np.linalg.solve(A, error)
    return dq


def quat_from_rot(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25 / s, (R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s, (R[1, 0] - R[0, 1]) * s])
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return np.array([(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s])
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return np.array([(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        return np.array([(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s])


def quat_error_to_rotvec(q_current, q_target):
    w0, x0, y0, z0 = q_current
    w1, x1, y1, z1 = q_target

    wc, xc, yc, zc = w0, -x0, -y0, -z0

    xe = w1 * xc + x1 * wc + y1 * zc - z1 * yc
    ye = w1 * yc - x1 * zc + y1 * wc + z1 * xc
    ze = w1 * zc + x1 * yc - y1 * xc + z1 * wc

    return 2.0 * np.array([xe, ye, ze])


def apply_deadzone(value, deadzone=DEADZONE):
    if abs(value) < deadzone:
        return 0.0
    return value


def scaled_deadzone(value, deadzone=DEADZONE):
    """Deadzone met herschaling zodat de stick na deadzone weer vloeiend oploopt."""
    if abs(value) < deadzone:
        return 0.0
    sign = 1.0 if value >= 0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def clamp_vec(v, limit):
    return np.clip(v, -limit, limit)



# Node

class PS4BaseArmFKIKToggle:
    def __init__(self):
        self.arm_mode = False
        self.arm_control_mode = "FK"  

        self.prev_buttons = []
        self.latest_joy = None
        self.q = None

        self.last_time = time.time()

        # IK targets
        self.target_pos = None
        self.target_quat = None
        self.tool_axis_ref = None

        self.cmd_pub = rospy.Publisher(COMMAND_TOPIC, Float64MultiArray, queue_size=10)
        rospy.Subscriber(JOY_TOPIC, Joy, self.joy_cb)
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.joint_state_cb)

        self.wait_for_services()
        self.create_service_proxies()

        rospy.loginfo("Wachten op joint states...")
        while self.q is None and not rospy.is_shutdown():
            rospy.sleep(0.05)
        self.reset_ik_target()

        rospy.Timer(rospy.Duration(1.0 / PUBLISH_RATE), self.publish_command)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("=" * 70)
        rospy.loginfo("ps4_base_arm_fk_ik_toggle gestart")
        rospy.loginfo("Start in BASE MODE. De base teleop blijft dan normaal werken.")
        rospy.loginfo(f"Button {MODE_TOGGLE_BUTTON}: BASE MODE <-> ARM MODE")
        rospy.loginfo(f"Button {FK_IK_TOGGLE_BUTTON}: in ARM MODE wisselen tussen FK en IK")
        rospy.loginfo("=" * 70)

    def wait_for_services(self):
        services = [
            HLC_VELOCITY_SERVICE,
            GRIPPER_OPEN_SERVICE,
            GRIPPER_CLOSE_SERVICE,
            DEFAULT_SERVICE,
            HOME_SERVICE,
            ZERO_SERVICE,
            START_SERVICE,
        ]
        for service in services:
            try:
                rospy.wait_for_service(service, timeout=5.0)
            except rospy.ROSException:
                rospy.logwarn(f"Service niet gevonden binnen timeout: {service}")

    def create_service_proxies(self):
        self.hlc_velocity_srv = rospy.ServiceProxy(HLC_VELOCITY_SERVICE, Trigger)
        self.gripper_open_srv = rospy.ServiceProxy(GRIPPER_OPEN_SERVICE, Trigger)
        self.gripper_close_srv = rospy.ServiceProxy(GRIPPER_CLOSE_SERVICE, Trigger)
        self.default_pos_srv = rospy.ServiceProxy(DEFAULT_SERVICE, Trigger)
        self.home_pos_srv = rospy.ServiceProxy(HOME_SERVICE, Trigger)
        self.zero_pos_srv = rospy.ServiceProxy(ZERO_SERVICE, Trigger)
        self.start_pos_srv = rospy.ServiceProxy(START_SERVICE, Trigger)

    def joint_state_cb(self, msg):
        if len(msg.position) >= 6:
            self.q = np.array(msg.position[:6], dtype=float)

    def joy_cb(self, msg):
        self.latest_joy = msg

        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
            return

        def rising_edge(button_idx):
            return (
                len(msg.buttons) > button_idx
                and len(self.prev_buttons) > button_idx
                and msg.buttons[button_idx] == 1
                and self.prev_buttons[button_idx] == 0
            )

        # BASE to ARM
        if rising_edge(MODE_TOGGLE_BUTTON):
            self.arm_mode = not self.arm_mode

            if self.arm_mode:
                self.change_to_hlc_velocity()
                if self.arm_control_mode == "IK":
                    self.reset_ik_target()
                rospy.loginfo(f"MODE CHANGED: ARM MODE ({self.arm_control_mode})")
            else:
                self.publish_zero()
                rospy.loginfo("MODE CHANGED: BASE MODE")

        # FK to IK, only in Arm mode
        if self.arm_mode and rising_edge(FK_IK_TOGGLE_BUTTON):
            self.publish_zero()
            self.arm_control_mode = "IK" if self.arm_control_mode == "FK" else "FK"
            if self.arm_control_mode == "IK":
                self.reset_ik_target()
            rospy.loginfo(f"ARM CONTROL MODE CHANGED: {self.arm_control_mode}")

        # Services 
        try:
            if rising_edge(OPEN_BUTTON):
                rospy.loginfo("Gripper open")
                self.gripper_open_srv()

            if rising_edge(CLOSE_BUTTON):
                rospy.loginfo("Gripper close")
                self.gripper_close_srv()

            if rising_edge(DEFAULT_BUTTON):
                rospy.loginfo("Go default position")
                self.publish_zero()
                self.default_pos_srv()
                rospy.sleep(0.2)
                self.reset_ik_target()

            if rising_edge(HOME_BUTTON):
                rospy.loginfo("Go home position")
                self.publish_zero()
                self.home_pos_srv()
                rospy.sleep(0.2)
                self.reset_ik_target()

            if rising_edge(ZERO_BUTTON):
                rospy.loginfo("Go zero position")
                self.publish_zero()
                self.zero_pos_srv()
                rospy.sleep(0.2)
                self.reset_ik_target()

            if rising_edge(START_BUTTON):
                rospy.loginfo("Go start position")
                self.publish_zero()
                self.start_pos_srv()
                rospy.sleep(0.2)
                self.reset_ik_target()

        except rospy.ServiceException as e:
            rospy.logwarn(f"Service call mislukt: {e}")

        self.prev_buttons = list(msg.buttons)

    def change_to_hlc_velocity(self):
        try:
            self.hlc_velocity_srv()
        except rospy.ServiceException as e:
            rospy.logwarn(f"Kon HLC velocity mode niet zetten: {e}")

    def axis(self, axes, i, scaled=False):
        if len(axes) <= i:
            return 0.0
        if scaled:
            return scaled_deadzone(axes[i])
        return apply_deadzone(axes[i])

    def button(self, buttons, i):
        if len(buttons) <= i:
            return 0
        return buttons[i]

    def publish_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.cmd_pub.publish(msg)

    def reset_ik_target(self):
        if self.q is None:
            return
        pos, R = forward_kinematics(self.q)
        self.target_pos = pos.copy()
        self.target_quat = quat_from_rot(R)
        self.tool_axis_ref = R[:, 2].copy()
        self.last_time = time.time()

    def publish_command(self, event):
        if not self.arm_mode:
            return

        if self.latest_joy is None or self.q is None:
            return

        if self.arm_control_mode == "FK":
            self.publish_fk_command()
        else:
            self.publish_ik_command()

    def publish_fk_command(self):
        axes = self.latest_joy.axes
        buttons = self.latest_joy.buttons

        j1 = FK_SCALE * self.axis(axes, AXIS_LEFT_LR)
        j2 = FK_SCALE * self.axis(axes, AXIS_LEFT_UD)
        j3 = FK_SCALE * self.axis(axes, AXIS_RIGHT_UD)
        j4 = FK_SCALE * self.axis(axes, AXIS_RIGHT_LR)

        j5 = 0.0
        if self.button(buttons, J5_POS_BUTTON):
            j5 += FK_SCALE
        if self.button(buttons, J5_NEG_BUTTON):
            j5 -= FK_SCALE

        j6 = 0.0
        if self.button(buttons, J6_POS_BUTTON):
            j6 += FK_SCALE
        if self.button(buttons, J6_NEG_BUTTON):
            j6 -= FK_SCALE

        cmd = [j1, j2, j3, j4, j5, j6]
        msg = Float64MultiArray(data=cmd)
        self.cmd_pub.publish(msg)

        rospy.loginfo_throttle(
            0.5,
            f"ARM MODE FK | cmd = [{j1:.2f}, {j2:.2f}, {j3:.2f}, {j4:.2f}, {j5:.2f}, {j6:.2f}]"
        )

    def publish_ik_command(self):
        axes = self.latest_joy.axes

        now = time.time()
        dt = np.clip(now - self.last_time, 0.005, 0.1)
        self.last_time = now

        # ROS Joy mapping
        ax_x = self.axis(axes, AXIS_LEFT_LR, scaled=True)
        ax_y = self.axis(axes, AXIS_LEFT_UD, scaled=True)
        ax_z = self.axis(axes, AXIS_RIGHT_UD, scaled=True)
        ax_rz = self.axis(axes, AXIS_RIGHT_LR, scaled=True)

        vx = ax_x * SPEED_X
        vy = ax_y * SPEED_Y       # stick omhoog = Y+
        vz = ax_z * SPEED_Z       # stick omhoog = Z+
        wz = ax_rz * SPEED_RZ

        current_pos, error_pos_raw = self.compute_ik_and_publish(vx, vy, vz, wz, dt)

        rospy.loginfo_throttle(
            0.5,
            f"ARM MODE IK | EE={np.round(current_pos, 3)} | target={np.round(self.target_pos, 3)} | err={np.round(error_pos_raw, 3)}"
        )

    def compute_ik_and_publish(self, vx, vy, vz, wz, dt):
        if self.target_pos is None or self.target_quat is None:
            self.reset_ik_target()

        q = self.q.copy()

        moving = (abs(vx) + abs(vy) + abs(vz) + abs(wz)) > 1e-6

        if moving:
            v_target = clamp_vec(np.array([vx, vy, vz]), MAX_LINEAR_SPEED)
            self.target_pos = self.target_pos + v_target * dt

            if abs(wz) > 1e-6:
                dtheta = np.clip(wz, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED) * dt
                dw = np.cos(dtheta / 2.0)
                dz_q = np.sin(dtheta / 2.0)
                w1, x1, y1, z1 = self.target_quat
                self.target_quat = np.array([
                    dw * w1 - dz_q * z1,
                    dw * x1 - dz_q * y1,
                    dw * y1 + dz_q * x1,
                    dw * z1 + dz_q * w1,
                ])
                self.target_quat /= np.linalg.norm(self.target_quat)
        else:
            current_pos_now, _ = forward_kinematics(q)
            alpha = 0.05
            self.target_pos = (1.0 - alpha) * self.target_pos + alpha * current_pos_now

        current_pos, current_R = forward_kinematics(q)
        current_quat = quat_from_rot(current_R)

        error_pos_raw = self.target_pos - current_pos
        pos_err_norm = np.linalg.norm(error_pos_raw)
        if pos_err_norm < POS_ERROR_DEADZONE:
            error_pos = np.zeros(3)
        else:
            error_pos = error_pos_raw * (pos_err_norm - POS_ERROR_DEADZONE) / pos_err_norm

        ori_raw = quat_error_to_rotvec(current_quat, self.target_quat)
        ori_err_norm = np.linalg.norm(ori_raw)
        if ori_err_norm < ORI_ERROR_DEADZONE:
            error_ori = np.zeros(3)
        else:
            error_ori = ORI_GAIN * ori_raw * (ori_err_norm - ORI_ERROR_DEADZONE) / ori_err_norm

        if moving:
            tool_axis_current = current_R[:, 2]
            twist_error = np.cross(tool_axis_current, self.tool_axis_ref)
            twist_norm = np.linalg.norm(twist_error)
            if twist_norm > ORI_ERROR_DEADZONE:
                error_ori = error_ori + TOOL_TWIST_GAIN * twist_error

        error = np.concatenate([error_pos, error_ori])

        if np.linalg.norm(error) < 1e-6:
            self.publish_zero()
            return current_pos, error_pos_raw

        J = jacobian(q)
        dq = weighted_dls(J, error, JOINT_WEIGHTS, DAMPING)

        dq_max = np.abs(dq).max()
        if dq_max > MAX_ANGVEL:
            dq *= MAX_ANGVEL / dq_max

        if np.abs(dq).max() < DQ_MIN_THRESHOLD:
            self.publish_zero()
            return current_pos, error_pos_raw

        for i in range(6):
            if q[i] >= JOINT_LIMITS[i, 1] - 0.01 and dq[i] > 0:
                dq[i] = 0.0
            if q[i] <= JOINT_LIMITS[i, 0] + 0.01 and dq[i] < 0:
                dq[i] = 0.0

        msg = Float64MultiArray(data=dq.tolist())
        self.cmd_pub.publish(msg)
        return current_pos, error_pos_raw

    def on_shutdown(self):
        rospy.loginfo("Shutdown: stuur nulcommando naar arm")
        self.publish_zero()


if __name__ == "__main__":
    rospy.init_node("ps4_base_arm_fk_ik_toggle")
    PS4BaseArmFKIKToggle()
    rospy.spin()
