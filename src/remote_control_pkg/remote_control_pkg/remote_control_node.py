import logging
import time
from typing import Sequence, Optional, Dict, Any
import argparse
import sys

# Try to import the bundled `fairino` subpackage first (when running as part
# of the `remote_control_pkg` package). If that fails, fall back to a
# globally installed `fairino` package. If neither is available, leave
# `FairinoSDK` as None so the code can raise a clear ImportError later.
FairinoSDK = None

from .fairino import Robot as FairinoSDK


OculusQuest3Controller = None
VRConfig = None

from .vr_oculus2 import OculusQuest3Controller, VRConfig


# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)


class FairinoConfig:
    def __init__(
        self,
        robot_ip: str,
        cmd_period: float = 0.008,
        pos_gain: Optional[Sequence[float]] = None,
        robot_side: Optional[str] = None,
    ):
        self.robot_ip = robot_ip
        self.cmd_period = cmd_period
        self.pos_gain = list(pos_gain) if pos_gain else [1.0] * 6
        self.robot_side = robot_side if robot_side in ('left', 'right') else 'right'


class FairinoCartRobot:
    name = "FairinoCartSimple"

    def __init__(self, config: FairinoConfig):
        self.config = config
        self.robot_ip = config.robot_ip
        self._cmdT = config.cmd_period
        self._pos_gain = config.pos_gain
        self._side = config.robot_side
        self.robot = None
        self._last_abs_pose: Optional[list[float]] = None
        self._last_send_time = 0.0

        self._acc = 80.0
        self._vel = 50.0
        self._filterT = 0.5
        self._gain_amplifier = 0.0
        self._lock = threading.RLock()

    @property
    def is_connected(self) -> bool:
        return self.robot is not None

    def connect(self):
        with self._lock:
            if self.is_connected:
                return
            if FairinoSDK is None:
                raise ImportError("未找到 fairino SDK，请先安装。")
            try:
                self.robot = FairinoSDK.RPC(self.robot_ip) if hasattr(FairinoSDK, "RPC") else FairinoSDK(self.robot_ip)
                try:
                    self.robot.SetSpeed(20)
                except Exception:
                    pass
                logger.info(f"[Fairino] Connected {self.robot_ip}")
            except Exception as e:
                self.robot = None
                raise RuntimeError(f"连接失败 {self.robot_ip}: {e}") from e

    def disconnect(self):
        with self._lock:
            if not self.is_connected:
                return
            try:
                if hasattr(self.robot, "CloseRPC"):
                    self.robot.CloseRPC()
            finally:
                self.robot = None
                logger.info("[Fairino] Disconnected")

    def _throttle(self):
        now = time.time()
        dt = now - self._last_send_time
        remain = self._cmdT - dt
        if remain > 0:
            time.sleep(remain)
        self._last_send_time = time.time()

    def _call_servo(self, mode: int, desc_pos: Sequence[float]):
        with self._lock:
            if not self.is_connected:
                raise RuntimeError("Robot 未连接")
            try:
                self.robot.ServoCart(
                    mode=mode,
                    desc_pos=list(desc_pos),
                    pos_gain=self._pos_gain,
                    acc=self._acc,
                    vel=self._vel,
                    cmdT=self._cmdT,
                    filterT=self._filterT,
                    gain=self._gain_amplifier,
                )
            except Exception as e:
                logger.error(f"ServoCart 失败 mode={mode}: {e}")

    def send_abs_pose(self, pose6: Sequence[float]):
        if len(pose6) != 6:
            raise ValueError("pose6 需要 6 个元素")
        self._call_servo(mode=0, desc_pos=pose6)
        self._last_abs_pose = list(pose6)
        self._throttle()

    def send_delta(self, delta6: Sequence[float]):
        if len(delta6) != 6:
            raise ValueError("delta6 需要 6 个元素")
        
        scaled_delta6 = [d * g for d, g in zip(delta6, self._pos_gain)]
        if self._side == 'right':
            scaled_delta6[0], scaled_delta6[1], scaled_delta6[2] = -scaled_delta6[1], -scaled_delta6[2], scaled_delta6[0]
            scaled_delta6[3], scaled_delta6[4], scaled_delta6[5] = -scaled_delta6[4], -scaled_delta6[5], scaled_delta6[3]
        elif self._side == 'left':
            scaled_delta6[0], scaled_delta6[1], scaled_delta6[2] = -scaled_delta6[2], -scaled_delta6[1], -scaled_delta6[0]
            scaled_delta6[3], scaled_delta6[4], scaled_delta6[5] = -scaled_delta6[5], -scaled_delta6[4], -scaled_delta6[3]
        
        self._call_servo(mode=1, desc_pos=scaled_delta6)

        if self._last_abs_pose:
            self._last_abs_pose = [a + d for a, d in zip(self._last_abs_pose, delta6)]
        self._throttle()

    def send_delta_tool(self, delta6: Sequence[float]):
        if len(delta6) != 6:
            raise ValueError("delta6 需要 6 个元素")
        self._call_servo(mode=2, desc_pos=delta6)
        if self._last_abs_pose:
            self._last_abs_pose = [a + d for a, d in zip(self._last_abs_pose, delta6)]
        self._throttle()

    def apply_action_dict(self, action: Dict[str, Any], mapping: str):
        if mapping == "absolute":
            keys = ["abs_x", "abs_y", "abs_z", "abs_roll", "abs_pitch", "abs_yaw"]
            if all(k in action for k in keys):
                self.send_abs_pose([action[k] for k in keys])
        elif mapping == "delta_base":
            keys = ["delta_x", "delta_y", "delta_z", "delta_roll", "delta_pitch", "delta_yaw"]
            if all(k in action for k in keys):
                self.send_delta([action[k] for k in keys])
        elif mapping == "delta_tool":
            keys = ["right_delta_x", "right_delta_y", "right_delta_z", "right_delta_roll", "right_delta_pitch", "right_delta_yaw"]
            if all(k in action for k in keys):
                self.send_delta_tool([action[k] for k in keys])
        else:
            raise ValueError("mapping 必须为 absolute | delta_base | delta_tool")

    def get_tcp_pose(self) -> Optional[list]:
        """获取当前 TCP 位姿 [x, y, z, rx, ry, rz]"""
        with self._lock:
            if not self.is_connected:
                return None
            try:
                ret_org = self.robot.GetActualTCPPose()
            
                return ret_org[1]
                
            except Exception as e:
                logger.error(f"获取 TCP 位姿异常: {e}")
                return None

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass


class DualFairinoCartRobot(Node):
    def __init__(self, config1: FairinoConfig, config2: FairinoConfig):
        super().__init__('dual_fairino_robot')
        self.robot1 = FairinoCartRobot(config1)
        self.robot2 = FairinoCartRobot(config2)
        self.robot1_connected = False
        self.robot2_connected = False
        
        # TCP pose storage
        self.tcp_pose1 = None
        self.tcp_pose2 = None
        
        # ROS2 publishers for TCP poses
        self.tcp_publisher1 = self.create_publisher(Float64MultiArray, '/robot1/tcp_pose', 10)
        self.tcp_publisher2 = self.create_publisher(Float64MultiArray, '/robot2/tcp_pose', 10)
        
        # TCP publishing timer
        self.tcp_publish_timer = self.create_timer(0.03, self.publish_tcp_poses)  # 30Hz
        
        # Optional peripherals
        self.gripper = None
        self._gripper_last_pressed = False
        self._gripper_last_value = None
        self._gripper_last_send_time = 0.0
        self._gripper_pending_pct = None
        self._gripper_pending_count = 0
        self._gripper_enabled = True
        self._gripper_toggle_last = False
        self._button_A_last = False


    def publish_tcp_poses(self):
        """Publish current TCP poses for both robots"""
        # Get and publish robot1 TCP pose
        if self.robot1_connected:
            tcp1 = self.robot1.get_tcp_pose()
            # logger.info(f"获取 TCP 位姿: {tcp1}")
            if tcp1 is not None:
                msg1 = Float64MultiArray()
                msg1.data = tcp1
                self.tcp_publisher1.publish(msg1)
        
        # Get and publish robot2 TCP pose
        if self.robot2_connected:
            tcp2 = self.robot2.get_tcp_pose()
            # logger.info(f"获取 TCP 位姿: {tcp2}")
            if tcp2 is not None:
                msg2 = Float64MultiArray()
                msg2.data = tcp2
                self.tcp_publisher2.publish(msg2)

    def connect(self):
        try:
            self.robot1.connect()
            self.robot1_connected = True
            self.get_logger().info('Robot 1 connected successfully')
        except Exception as e:
            logger.warning(f"Robot 1 连接失败: {e}")
            self.robot1_connected = False

        try:
            self.robot2.connect()
            self.robot2_connected = True
            self.get_logger().info('Robot 2 connected successfully')
        except Exception as e:
            logger.warning(f"Robot 2 连接失败: {e}")
            self.robot2_connected = False

    def disconnect(self):
        if self.robot1_connected:
            self.robot1.disconnect()
        if self.robot2_connected:
            self.robot2.disconnect()
        self.get_logger().info('Both robots disconnected')

    def apply_action_dict(self, action: Dict[str, Any], mapping: str, flag):
        right_action = {k[6:]: v for k, v in action.items() if k.startswith("right_")}
        left_action = {k[5:]: v for k, v in action.items() if k.startswith("left_")}
        if flag:
            logger.info(f"右侧动作: {right_action}")
            logger.info(f"左侧动作: {left_action}")

        if self.robot1_connected and right_action:
            self.robot1.apply_action_dict(right_action, mapping)
        if self.robot2_connected and left_action:
            self.robot2.apply_action_dict(left_action, mapping)

    def get_button_status(self, action: Dict[str, Any], threshold: float = 0.5) -> Dict[str, Dict[str, Any]]:
        buttons = action.get("buttons", {}) or {}
        status: Dict[str, Dict[str, Any]] = {}
        for name in buttons.keys():
            try:
                if OculusQuest3Controller is not None:
                    val = OculusQuest3Controller._get_button(buttons, name)
                else:
                    raw = buttons.get(name, 0.0)
                    if isinstance(raw, (tuple, list)):
                        raw = raw[0]
                    val = float(raw)
            except Exception:
                val = 0.0
            pressed = val > threshold
            status[name] = {"value": val, "pressed": pressed}
        return status

    def run_vr_loop(
        self,
        *,
        vr_mode: str = "relative",
        mapping: str = "delta_base",
        robot_idle_sleep: float = 0.01,
        print_interval: float = 1.0,
        use_gripper: bool = False,
        gripper_toggle_button: str = "B",
        joint6_step: float = 5.0,
    ):
        if OculusQuest3Controller is None:
            raise RuntimeError("未找到 VR 控制器类 (vr_oculus 导入失败)")
        
        vr = OculusQuest3Controller(
            VRConfig(control_mode=vr_mode, use_gripper=use_gripper)
        )
        vr.connect()
        logger.info(f"[VR] Connected, vr_mode={vr_mode}, mapping={mapping}")
        last_print = time.time()
        
        try:
            while rclpy.ok():
                act = vr.get_action()
                self.apply_action_dict(act, mapping=mapping, flag=False)

                # Process button and gripper logic (保持原有逻辑)
                try:
                    status = self.get_button_status(act)
                    # Toggle gripper receive state
                    try:
                        toggle_pressed = False
                        if isinstance(status, dict) and gripper_toggle_button in status:
                            toggle_pressed = bool(status.get(gripper_toggle_button, {}).get('pressed'))
                        if toggle_pressed and not getattr(self, '_gripper_toggle_last', False):
                            self._gripper_enabled = not getattr(self, '_gripper_enabled', True)
                            if not self._gripper_enabled:
                                self._gripper_pending_pct = None
                                self._gripper_pending_count = 0
                            logger.info('Gripper receive toggled to %s (button=%s)', self._gripper_enabled, gripper_toggle_button)
                        self._gripper_toggle_last = toggle_pressed
                    except Exception:
                        logger.exception('Failed to process gripper toggle button')
                    
                    # Gripper and button processing (保持原有逻辑)
                    if hasattr(self, 'button_bridge') and self.button_bridge is not None:
                        try:
                            self.button_bridge.process_status(status)
                        except Exception:
                            logger.exception('button_bridge.process_status failed')
                    
                    # Gripper command sending
                    try:
                        right_motion = any(
                            k.startswith('right_') and (not k.endswith('gripper')) and float(act.get(k, 0.0)) != 0.0
                            for k in act
                        )
                    except Exception:
                        right_motion = False

                    if right_motion and getattr(self, 'gripper', None) is not None and getattr(self, '_gripper_enabled', True):
                        try:
                            grip_val = None
                            for k in ('right_gripper', 'rightGrip', 'right_grip'):
                                if k in act:
                                    grip_val = act.get(k)
                                    break

                            if grip_val is None and status and isinstance(status.get('rightGrip'), dict):
                                grip_val = status['rightGrip'].get('value', 0.0)

                            if grip_val is not None:
                                try:
                                    val_float = float(grip_val)
                                    val_clamped = max(0.0, min(1.0, val_float))
                                    pct = int(round((1.0 - val_clamped) * 100.0))
                                    self._gripper_pending_pct = None
                                    self._gripper_pending_count = 0
                                    self.gripper.send_command_with_monitoring_percent(pct, 50, wait_for_completion=False)
                                    self._gripper_last_value = pct
                                    self._gripper_last_send_time = time.time()
                                    logger.info('Gripper: sent per-right-act pct=%s (right)', pct)
                                except Exception:
                                    logger.exception('Failed to send per-right-act gripper command')
                        except Exception:
                            logger.exception('Per-right-act gripper handling failed')

                    # Button A for joint-6 rotation
                    try:
                        a_pressed = False
                        if isinstance(status, dict) and 'A' in status:
                            a_pressed = bool(status.get('A', {}).get('pressed'))
                        if a_pressed and not getattr(self, '_button_A_last', False):
                            try:
                                if self.robot1_connected and getattr(self.robot1, 'robot', None) is not None:
                                    r = self.robot1.robot
                                    try:
                                        cur = r.robot_state_pkg.jt_cur_pos
                                        cur_list = [float(cur[i]) for i in range(6)]
                                    except Exception:
                                        cur_list = None
                                    if cur_list is None:
                                        logger.warning('Cannot read joint state for right robot (A-press ignored)')
                                    else:
                                        try:
                                            step = float(joint6_step)
                                        except Exception:
                                            step = 0.5
                                        cur_list[5] = cur_list[5] + step
                                        try:
                                            err = r.MoveJ(cur_list, 0, 0)
                                            if err != 0:
                                                try:
                                                    safety = None
                                                    try:
                                                        safety = r.GetSafetyCode()
                                                    except Exception:
                                                        safety = None
                                                    logger.warning('MoveJ returned error %s on A-press; cur_joints=%s target_joints=%s step=%s safety=%s',
                                                                   err, cur_list, cur_list, step, safety)
                                                except Exception:
                                                    logger.warning('MoveJ returned error %s on A-press', err)
                                            else:
                                                logger.info('MoveJ sent to right robot: joint6 += %s', step)
                                        except Exception:
                                            logger.exception('Failed to send MoveJ on A-press')
                            except Exception:
                                logger.exception('Failed during MoveJ on A-press')
                    except Exception:
                        logger.exception('Failed to process A button for joint-6')
                    finally:
                        try:
                            if isinstance(status, dict) and 'A' in status:
                                self._button_A_last = bool(status.get('A', {}).get('pressed'))
                            else:
                                self._button_A_last = False
                        except Exception:
                            self._button_A_last = False
                except Exception:
                    logger.exception('Failed to obtain/process button status')

                now = time.time()
                if all(v == 0.0 for k, v in act.items() if k.startswith("right_") or k.startswith("left_")):
                    time.sleep(robot_idle_sleep)
                    continue
                if now - last_print > print_interval:
                    logger.info(f"Right Act: { {k: act[k] for k in act if k.startswith('right_')} }")
                    logger.info(f"Left Act: { {k: act[k] for k in act if k.startswith('left_')} }")
                    last_print = now
                
                # Process ROS callbacks
                # rclpy.spin_once(self, timeout_sec=0.001)
                
        except KeyboardInterrupt:
            logger.info("用户中断 VR loop")
        finally:
            vr.disconnect()
            self.disconnect()
            logger.info("程序已停止")
            try:
                if getattr(self, 'gripper', None) is not None:
                    try:
                        self.gripper.disconnect()
                        logger.info('Gripper disconnected')
                    except Exception:
                        logger.exception('Failed to disconnect gripper')
            except Exception:
                pass


def _build_arg_parser():
    p = argparse.ArgumentParser(description="Fairino + Oculus VR 控制")
    p.add_argument("--robot-ip", default="192.168.57.3", help="机器人 IP")
    p.add_argument("--robot2-ip", default="192.168.57.2", help="第二个机器人 IP")
    p.add_argument("--vr-mode", default="relative", choices=["relative", "absolute"], help="VR 输出模式")
    p.add_argument("--mapping", default="delta_base", choices=["absolute", "delta_base", "delta_tool"], help="机器人执行映射")
    p.add_argument("--pos-gain", type=float, nargs=6, default=[25.0, 25.0, 25.00, 3.0, 3.0, 3.0], help="六轴增量增益")
    p.add_argument("--cmd-period", type=float, default=0.008, help="发送周期")
    p.add_argument("--use-gripper", action="store_true", default=True, help="Enable continuous gripper value from VR controller")
    p.add_argument("--joint6-step", type=float, default=-5.0, help="Joint-6 rotation step (degrees) to apply on A-press")
    return p


def main():
    rclpy.init()
    
    parser = _build_arg_parser()
    args, _ = parser.parse_known_args()

    cfg1 = FairinoConfig(
        robot_ip=args.robot2_ip,
        cmd_period=args.cmd_period,
        pos_gain=args.pos_gain,
        robot_side='right',
    )
    cfg2 = FairinoConfig(
        robot_ip=args.robot_ip,
        cmd_period=args.cmd_period,
        pos_gain=args.pos_gain,
        robot_side='left',
    )

    dual_robot = DualFairinoCartRobot(cfg1, cfg2)

    dual_robot.connect()
    
    # Start ROS spin thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(dual_robot,), daemon=True)
    spin_thread.start()

    try:
        dual_robot.run_vr_loop(vr_mode=args.vr_mode, use_gripper=args.use_gripper, joint6_step=args.joint6_step)
    finally:
        dual_robot.disconnect()
        dual_robot.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()