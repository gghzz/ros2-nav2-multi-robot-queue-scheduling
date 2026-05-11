#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import math
import os
import time
from enum import Enum

import yaml
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


class RobotState(Enum):
    IDLE = 0
    GOING_TO_WAIT = 1
    WAITING_IN_QUEUE = 2
    GOING_TO_WORKSTATION = 3
    WORKING = 4
    GOING_TO_EXIT = 5
    FINISHED = 6
    FAILED = 7
    MOVING_TO_FRONT = 8   # 从后方等待点补位到队首等待点 P1
    RECOVERING = 9        # 导航失败后的自动恢复/重试状态


def yaw_to_quaternion(yaw):
    """
    2D yaw 转四元数
    """
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class RobotTask:
    def __init__(self, name, wait_pose, work_pose, exit_pose, wait_slot_name):
        self.name = name

        # 初始等待点。例如 robot1 -> P1，robot2 -> P2
        self.wait_pose = wait_pose
        self.initial_wait_pose = wait_pose
        self.initial_wait_slot_name = wait_slot_name

        # 当前正在前往的等待点名称，到达后会写入 current_wait_slot_name
        self.target_wait_slot_name = wait_slot_name
        self.current_wait_slot_name = None

        self.work_pose = work_pose
        self.exit_pose = exit_pose

        self.state = RobotState.IDLE

        self.goal_handle = None
        self.result_future = None

        self.arrived_wait = False
        self.arrived_work = False
        self.arrived_exit = False

        # 第八步：失败恢复相关字段
        # last_goal_* 用于导航失败后自动重发上一个目标。
        self.last_goal_pose = None
        self.last_goal_description = ''
        self.last_motion_state = None
        self.retry_count = 0          # 当前目标已经重试的次数
        self.total_retry_count = 0    # 整个任务累计重试次数，用于 CSV 统计
        self.retry_after_time = None
        self.recovery_reason = ''
        self.last_recovery_log_time = 0.0


class MultiRobotQueueDemo(Node):
    def __init__(self):
        super().__init__('multi_robot_queue_demo')

        # ============================================================
        # 1. 从 queue_config.yaml 加载调度配置
        #
        # 默认读取：
        #   install/queue_manager/share/queue_manager/config/queue_config.yaml
        # 也可以运行时指定：
        #   ros2 run queue_manager multi_robot_queue_demo --ros-args \
        #     -p config_file:=/绝对路径/queue_config.yaml
        # ============================================================

        self.declare_parameter('config_file', '')

        self.config = self.load_queue_config()

        # 机器人列表、初始位姿、队列顺序
        self.robot_configs = self.parse_robot_configs(self.config)
        self.robot_names = [robot_cfg['name'] for robot_cfg in self.robot_configs]
        self.initial_poses = {
            robot_cfg['name']: robot_cfg['initial_pose']
            for robot_cfg in self.robot_configs
        }

        # 等待点槽位：P1/P2/P3...
        self.wait_slots = self.parse_wait_slots(self.config)
        self.front_wait_slot_name = self.config.get('front_wait_slot', 'P1')

        if self.front_wait_slot_name not in self.wait_slots:
            raise RuntimeError(
                f'front_wait_slot={self.front_wait_slot_name} 不在 wait_slots 中，请检查 queue_config.yaml'
            )

        # 工作站配置
        workstation_cfg = self.config.get('workstation', {})
        self.workstation_pose = self.parse_pose(
            workstation_cfg.get('pose'),
            'workstation.pose'
        )
        self.work_duration = float(workstation_cfg.get('work_duration', 5.0))

        # 恢复参数
        recovery_cfg = self.config.get('recovery', {})
        self.max_retry_count = int(recovery_cfg.get('max_retry_count', 2))
        self.retry_delay = float(recovery_cfg.get('retry_delay', 3.0))

        # AMCL 初始化参数
        amcl_cfg = self.config.get('amcl', {})
        self.initial_pose_publish_count = 0
        self.initial_pose_publish_times = int(amcl_cfg.get('initial_pose_publish_times', 5))
        self.initial_pose_wait_time = float(amcl_cfg.get('initial_pose_wait_time', 5.0))
        self.initial_pose_done = False
        self.initial_pose_done_time = None

        # 可视化参数
        visualization_cfg = self.config.get('visualization', {})
        self.marker_lifetime_sec = int(visualization_cfg.get('marker_lifetime_sec', 2))
        self.summary_pose = self.parse_pose(
            visualization_cfg.get('summary_pose', [-2.7, 2.0, 0.0]),
            'visualization.summary_pose'
        )

        # CSV 日志路径
        logging_cfg = self.config.get('logging', {})
        result_csv_path = str(logging_cfg.get('result_csv_path', 'queue_result.csv'))
        self.result_csv_path = os.path.abspath(os.path.expanduser(result_csv_path))

        # 构造每台机器人的 wait/work/exit 点位
        self.waypoints = {}
        for robot_cfg in self.robot_configs:
            name = robot_cfg['name']
            wait_slot = robot_cfg['wait_slot']

            if wait_slot not in self.wait_slots:
                raise RuntimeError(
                    f'{name}.wait_slot={wait_slot} 不在 wait_slots 中，请检查 queue_config.yaml'
                )

            self.waypoints[name] = {
                'wait': self.wait_slots[wait_slot],
                'wait_slot': wait_slot,
                'work': self.workstation_pose,
                'exit': robot_cfg['exit_pose'],
            }

        # 队列顺序，默认按 robots 里出现的顺序。
        self.queue = list(self.config.get('queue_order', self.robot_names))

        for name in self.queue:
            if name not in self.robot_names:
                raise RuntimeError(
                    f'queue_order 中的机器人 {name} 不在 robots 列表中，请检查 queue_config.yaml'
                )

        # 是否已经进入正式调度阶段
        self.started = False

        self.get_logger().info(
            f'已从配置文件加载排队调度参数: {self.config_file_path}'
        )
        self.get_logger().info(
            f'机器人列表: {self.robot_names}, 队列顺序: {self.queue}, 队首等待点: {self.front_wait_slot_name}'
        )

        # ============================================================
        # 6. 创建机器人任务对象
        # ============================================================

        self.robots = {}

        for name in self.robot_names:
            self.robots[name] = RobotTask(
                name=name,
                wait_pose=self.waypoints[name]['wait'],
                work_pose=self.waypoints[name]['work'],
                exit_pose=self.waypoints[name]['exit'],
                wait_slot_name=self.waypoints[name]['wait_slot']
            )

        # ============================================================
        # 7. 创建 NavigateToPose ActionClient
        # ============================================================

        self.nav_clients = {}

        for name in self.robot_names:
            action_name = f'/{name}/navigate_to_pose'

            self.nav_clients[name] = ActionClient(
                self,
                NavigateToPose,
                action_name
            )

            self.get_logger().info(f'创建 ActionClient: {action_name}')

        # ============================================================
        # 8. 创建 initialpose publisher
        # ============================================================

        initialpose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.initialpose_pubs = {}

        for name in self.robot_names:
            topic_name = f'/{name}/initialpose'

            self.initialpose_pubs[name] = self.create_publisher(
                PoseWithCovarianceStamped,
                topic_name,
                initialpose_qos
            )

            self.get_logger().info(f'创建 InitialPose Publisher: {topic_name}')

        # ============================================================
        # 9. 排队队列
        #
        # 半并行版：
        # robot1 占用工作站时，robot2 可以提前去等待点。
        # 但 robot2 不能进入工作站，直到 robot1 到达 EXIT。
        # ============================================================

        # self.queue 已由 queue_config.yaml 加载。

        # 当前正在执行“进站 / 作业 / 出站”主流程的机器人。
        # 这个变量主要用于调度主循环判断当前由谁占用主流程。
        self.current_working_robot = None

        # ============================================================
        # 10. 共享资源锁
        #
        # workstation_occupied_by:
        #   工作站锁。机器人一旦开始从 P1 前往 WORKSTATION，
        #   就认为它已经预约/占用工作站；直到它到达 EXIT 后才释放。
        #
        # exit_lane_occupied_by:
        #   出口通道锁。机器人从 WORKSTATION 离开前往 EXIT 时占用；
        #   到达 EXIT 后释放。
        #
        # 第五步的核心：
        #   robot1 从 WORKSTATION 去 EXIT 期间，robot2 不能从 P1 进 WORKSTATION。
        # ============================================================
        self.workstation_occupied_by = None
        self.exit_lane_occupied_by = None

        # 作业开始时间
        self.work_start_time = {}

        # ============================================================
        # 11. 第六步：状态话题 + RViz Marker 可视化
        #
        # /queue_manager/status:
        #   发布当前队列、机器人状态、资源锁状态，方便 ros2 topic echo 查看。
        #
        # /queue_manager/markers:
        #   在 RViz 中显示 P1/P2/P3、WORKSTATION、EXIT、资源锁和机器人调度状态。
        # ============================================================
        self.status_pub = self.create_publisher(
            String,
            '/queue_manager/status',
            10
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/queue_manager/markers',
            10
        )

        # self.marker_lifetime_sec 已由 queue_config.yaml 加载。

        # ============================================================
        # 12. 第七步：实验日志与调度指标统计
        #
        # 程序会在当前终端所在目录自动生成 queue_result.csv。
        # 如果你是在 ~/ros2test/duoji/chapt7_ws 下执行 ros2 run，
        # 那么文件通常就在该目录下。
        # ============================================================
        self.record_zero_time = None
        # self.result_csv_path 已由 queue_config.yaml 加载。
        self.task_records = {}

        for name in self.robot_names:
            self.task_records[name] = {
                'robot_name': name,
                'start_time': None,
                'wait_arrival_time': None,
                'front_wait_arrival_time': None,
                'work_start_time': None,
                'work_finish_time': None,
                'exit_time': None,
                'total_time': None,
                'queue_wait_time': None,
                'front_move_time': None,
                'retry_count': 0,
                'failure_reason': '',
                'recovery_status': '',
                'status': 'IDLE'
            }

        self.result_summary_printed = False

        self.get_logger().info(
            f'实验日志 CSV 将保存到: {self.result_csv_path}'
        )

        # 主循环
        self.timer = self.create_timer(1.0, self.main_loop)

        self.get_logger().info('multi_robot_queue_demo 已启动')
        self.get_logger().info('等待 Nav2 action server 就绪...')


    # ================================================================
    # 第九步：配置文件读取与解析
    # ================================================================

    def load_queue_config(self):
        """
        读取 queue_config.yaml。

        默认路径：
            <queue_manager 包安装目录>/config/queue_config.yaml

        也支持运行时指定：
            ros2 run queue_manager multi_robot_queue_demo --ros-args \
              -p config_file:=/home/ggb/ros2test/duoji/chapt7_ws/src/queue_manager/config/queue_config.yaml
        """
        param_path = self.get_parameter('config_file').get_parameter_value().string_value

        candidate_paths = []

        if param_path:
            candidate_paths.append(os.path.abspath(os.path.expanduser(param_path)))

        try:
            package_share = get_package_share_directory('queue_manager')
            candidate_paths.append(
                os.path.join(package_share, 'config', 'queue_config.yaml')
            )
        except Exception as e:
            self.get_logger().warn(
                f'获取 queue_manager 包共享目录失败: {e}'
            )

        # 兜底：如果你直接把 queue_config.yaml 放在当前运行目录，也能读取。
        candidate_paths.append(os.path.abspath('queue_config.yaml'))

        for path in candidate_paths:
            if not path:
                continue
            if os.path.exists(path):
                try:
                    with open(path, 'r', encoding='utf-8') as f:
                        config = yaml.safe_load(f)

                    if config is None:
                        raise RuntimeError('配置文件为空')

                    self.config_file_path = path
                    return config

                except Exception as e:
                    raise RuntimeError(
                        f'读取配置文件失败: {path}, error={e}'
                    )

        raise RuntimeError(
            '没有找到 queue_config.yaml。请确认已创建 queue_manager/config/queue_config.yaml，'
            '并在 setup.py 中安装 config/*.yaml；或者用 -p config_file:=/绝对路径/queue_config.yaml 指定。'
        )

    def parse_pose(self, value, field_name):
        """
        解析 [x, y, yaw] 格式的位姿。
        """
        if value is None:
            raise RuntimeError(f'配置缺少字段: {field_name}')

        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise RuntimeError(
                f'{field_name} 必须是 [x, y, yaw] 三个数，例如 [1.2, -0.65, 0.0]'
            )

        try:
            return (float(value[0]), float(value[1]), float(value[2]))
        except Exception as e:
            raise RuntimeError(f'{field_name} 解析失败: {value}, error={e}')

    def parse_wait_slots(self, config):
        """
        解析等待点槽位。
        """
        wait_slots_cfg = config.get('wait_slots')
        if not isinstance(wait_slots_cfg, dict) or len(wait_slots_cfg) == 0:
            raise RuntimeError('queue_config.yaml 中必须配置 wait_slots')

        wait_slots = {}
        for slot_name, pose in wait_slots_cfg.items():
            wait_slots[str(slot_name)] = self.parse_pose(
                pose,
                f'wait_slots.{slot_name}'
            )

        return wait_slots

    def parse_robot_configs(self, config):
        """
        解析 robots 配置。

        推荐格式：
            robots:
              - name: robot1
                initial_pose: [-3.5, 0.5, 0.0]
                wait_slot: P1
                exit_pose: [0.35, -2.0, -1.57]
        """
        robots_cfg = config.get('robots')
        if not isinstance(robots_cfg, list) or len(robots_cfg) == 0:
            raise RuntimeError('queue_config.yaml 中必须配置 robots 列表')

        parsed = []
        used_names = set()

        for index, robot_cfg in enumerate(robots_cfg):
            if not isinstance(robot_cfg, dict):
                raise RuntimeError(f'robots[{index}] 必须是字典')

            name = str(robot_cfg.get('name', '')).strip()
            if not name:
                raise RuntimeError(f'robots[{index}] 缺少 name')
            if name in used_names:
                raise RuntimeError(f'robots 中存在重复机器人名称: {name}')

            wait_slot = str(robot_cfg.get('wait_slot', '')).strip()
            if not wait_slot:
                raise RuntimeError(f'{name} 缺少 wait_slot')

            parsed.append({
                'name': name,
                'initial_pose': self.parse_pose(
                    robot_cfg.get('initial_pose'),
                    f'robots.{name}.initial_pose'
                ),
                'wait_slot': wait_slot,
                'exit_pose': self.parse_pose(
                    robot_cfg.get('exit_pose'),
                    f'robots.{name}.exit_pose'
                ),
            })
            used_names.add(name)

        return parsed

    # ================================================================
    # 创建消息
    # ================================================================

    def create_initial_pose_msg(self, pose):
        """
        创建 AMCL 初始位姿消息
        """
        x, y, yaw = pose

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'

        # 使用 0 时间戳，和命令行手动发布效果一致
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0685
        ]

        return msg

    def create_goal_msg(self, pose):
        """
        创建 NavigateToPose 目标消息
        """
        x, y, yaw = pose

        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'

        # 使用 0 时间戳，让 Nav2 使用最新 TF
        goal_msg.pose.header.stamp.sec = 0
        goal_msg.pose.header.stamp.nanosec = 0

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(yaw)
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        goal_msg.behavior_tree = ''

        return goal_msg

    # ================================================================
    # 第七步：实验日志与调度指标统计
    # ================================================================

    def now_record_time(self):
        """
        返回相对实验开始的时间，单位秒。
        record_zero_time 在 start_demo() 中设置，这样 AMCL 初始化时间不会计入任务耗时。
        """
        if self.record_zero_time is None:
            return 0.0
        return time.time() - self.record_zero_time

    def set_record_field_once(self, robot_name, field_name, value=None):
        """
        只在字段为空时写入时间，避免主循环重复覆盖。
        """
        record = self.task_records[robot_name]
        if record.get(field_name) is None:
            record[field_name] = self.now_record_time() if value is None else value

    def update_record_status(self, robot_name, status):
        """
        更新机器人任务状态。
        """
        self.task_records[robot_name]['status'] = status

    def update_derived_metrics(self, robot_name):
        """
        根据已有时间戳计算派生指标。
        """
        record = self.task_records[robot_name]

        start_time = record.get('start_time')
        exit_time = record.get('exit_time')
        wait_arrival_time = record.get('wait_arrival_time')
        front_wait_arrival_time = record.get('front_wait_arrival_time')
        work_start_time = record.get('work_start_time')

        if start_time is not None and exit_time is not None:
            record['total_time'] = exit_time - start_time

        if wait_arrival_time is not None and work_start_time is not None:
            record['queue_wait_time'] = work_start_time - wait_arrival_time

        if wait_arrival_time is not None and front_wait_arrival_time is not None:
            record['front_move_time'] = front_wait_arrival_time - wait_arrival_time

    def mark_robot_started(self, robot_name):
        """
        记录机器人开始执行任务的时间。
        """
        self.set_record_field_once(robot_name, 'start_time')
        self.update_record_status(robot_name, 'RUNNING')
        self.write_results_csv()

    def mark_robot_failed(self, robot_name, reason=''):
        """
        记录机器人最终失败状态，并写入 CSV，防止资源释放后丢失信息。
        """
        record = self.task_records[robot_name]
        robot = self.robots[robot_name]
        record['retry_count'] = robot.total_retry_count
        record['failure_reason'] = reason or robot.recovery_reason
        record['recovery_status'] = 'GIVE_UP'
        self.update_record_status(robot_name, 'FAILED')
        self.update_derived_metrics(robot_name)
        self.write_results_csv()

    def write_results_csv(self):
        """
        将当前实验结果写入 CSV。
        采用覆盖写入方式：每次状态变化都会刷新一次，程序中断时也能保留最新结果。
        """
        fieldnames = [
            'robot_name',
            'start_time',
            'wait_arrival_time',
            'front_wait_arrival_time',
            'work_start_time',
            'work_finish_time',
            'exit_time',
            'total_time',
            'queue_wait_time',
            'front_move_time',
            'retry_count',
            'failure_reason',
            'recovery_status',
            'status'
        ]

        try:
            with open(self.result_csv_path, 'w', newline='', encoding='utf-8') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()

                for robot_name in self.robot_names:
                    self.update_derived_metrics(robot_name)
                    record = dict(self.task_records[robot_name])

                    # 数值统一保留 3 位小数，便于阅读和写报告。
                    for key, value in list(record.items()):
                        if isinstance(value, float):
                            record[key] = f'{value:.3f}'

                    writer.writerow(record)

        except Exception as e:
            self.get_logger().error(
                f'写入实验日志 CSV 失败: {self.result_csv_path}, error={e}'
            )

    def print_result_summary(self):
        """
        所有任务完成后，在终端打印一次简要统计结果。
        """
        self.write_results_csv()

        summary_lines = []
        summary_lines.append('========== Queue Scheduling Result Summary ==========')
        summary_lines.append(f'CSV: {self.result_csv_path}')

        for robot_name in self.robot_names:
            self.update_derived_metrics(robot_name)
            record = self.task_records[robot_name]
            total_time = record.get('total_time')
            queue_wait_time = record.get('queue_wait_time')
            status = record.get('status')

            total_text = f'{total_time:.2f}s' if isinstance(total_time, float) else '-'
            wait_text = f'{queue_wait_time:.2f}s' if isinstance(queue_wait_time, float) else '-'

            summary_lines.append(
                f'{robot_name}: status={status}, total_time={total_text}, queue_wait_time={wait_text}'
            )

        self.get_logger().info('\n'.join(summary_lines))

    # ================================================================
    # 第八步：失败恢复机制
    # ================================================================

    def update_recovery_record(self, robot_name, recovery_status, reason=''):
        """
        更新 CSV 中的恢复相关字段。
        """
        robot = self.robots[robot_name]
        record = self.task_records[robot_name]
        record['retry_count'] = robot.total_retry_count
        if reason:
            record['failure_reason'] = reason
        record['recovery_status'] = recovery_status
        self.write_results_csv()

    def reset_retry_state_after_success(self, robot_name):
        """
        单个目标成功后，清空该目标的恢复状态。
        下一个目标重新拥有 max_retry_count 次重试机会。
        """
        robot = self.robots[robot_name]
        robot.retry_count = 0
        robot.retry_after_time = None
        robot.recovery_reason = ''
        robot.last_recovery_log_time = 0.0

        record = self.task_records[robot_name]
        record['retry_count'] = robot.total_retry_count
        record['recovery_status'] = 'OK'
        self.write_results_csv()

    def fail_robot_permanently(self, robot_name, reason):
        """
        超过最大重试次数后，标记机器人最终失败。
        同时释放它占用的共享资源，避免 WORKSTATION / EXIT_LANE 锁死。
        """
        robot = self.robots[robot_name]
        robot.state = RobotState.FAILED
        robot.retry_after_time = None
        robot.recovery_reason = reason

        self.get_logger().error(
            f'[失败恢复] {robot_name} 超过最大重试次数，标记为 FAILED。原因: {reason}'
        )

        self.mark_robot_failed(robot_name, reason)
        self.release_resources_for_robot(robot_name)

        # 如果失败机器人仍在等待队列中，将其移除，避免队列卡死。
        if robot_name in self.queue:
            self.queue = [name for name in self.queue if name != robot_name]
            self.get_logger().warn(
                f'[失败恢复] 已将 {robot_name} 从等待队列中移除'
            )

    def schedule_navigation_retry(self, robot_name, description, reason):
        """
        导航失败后的统一处理入口。

        逻辑：
        1. 如果还有重试次数，进入 RECOVERING 状态；
        2. 等 retry_delay 秒后自动重发 last_goal；
        3. 如果超过 max_retry_count，最终 FAILED 并释放资源锁。
        """
        robot = self.robots[robot_name]

        # 如果没有可重发的目标，只能直接失败。
        if robot.last_goal_pose is None or robot.last_motion_state is None:
            self.fail_robot_permanently(
                robot_name,
                f'无法恢复：没有保存上一次导航目标。原始原因: {reason}'
            )
            return

        # 已经到达最大重试次数，放弃该机器人。
        if robot.retry_count >= self.max_retry_count:
            self.fail_robot_permanently(
                robot_name,
                f'{description} 连续失败，已重试 {robot.retry_count} 次。最后原因: {reason}'
            )
            return

        robot.retry_count += 1
        robot.total_retry_count += 1
        robot.state = RobotState.RECOVERING
        robot.retry_after_time = time.time() + self.retry_delay
        robot.recovery_reason = reason
        robot.last_recovery_log_time = 0.0

        self.update_record_status(robot_name, 'RECOVERING')
        self.update_recovery_record(
            robot_name,
            recovery_status=f'RETRY_{robot.retry_count}_PENDING',
            reason=reason
        )

        self.get_logger().warn(
            f'[失败恢复] {robot_name} 导航失败: {description} | reason={reason} | '
            f'{self.retry_delay:.1f}s 后第 {robot.retry_count}/{self.max_retry_count} 次重试'
        )

    def process_recovering_robots(self):
        """
        处理处于 RECOVERING 状态的机器人。
        到达 retry_after_time 后，自动重发上一次导航目标。

        返回值：
            True  表示当前有机器人正在恢复，主调度本轮应暂停；
            False 表示没有恢复任务，可以继续正常调度。
        """
        recovering_found = False
        now = time.time()

        for robot_name in self.robot_names:
            robot = self.robots[robot_name]

            if robot.state != RobotState.RECOVERING:
                continue

            recovering_found = True

            if robot.retry_after_time is None:
                self.fail_robot_permanently(
                    robot_name,
                    'RECOVERING 状态异常：retry_after_time 为空'
                )
                continue

            remaining = robot.retry_after_time - now
            if remaining > 0.0:
                # 避免每秒疯狂打印同一句，只每约 2 秒提示一次。
                if now - robot.last_recovery_log_time > 2.0:
                    robot.last_recovery_log_time = now
                    self.get_logger().info(
                        f'[失败恢复] {robot_name} 正在等待重试，剩余 {remaining:.1f}s，'
                        f'目标: {robot.last_goal_description}'
                    )
                continue

            # 恢复到失败前的运动状态，然后重发上一个目标。
            retry_state = robot.last_motion_state
            retry_pose = robot.last_goal_pose
            retry_desc = robot.last_goal_description

            self.get_logger().warn(
                f'[失败恢复] {robot_name} 开始第 {robot.retry_count}/{self.max_retry_count} 次重试: {retry_desc}'
            )

            robot.state = retry_state
            robot.retry_after_time = None
            self.update_record_status(robot_name, f'RETRYING_{retry_desc}')
            self.update_recovery_record(
                robot_name,
                recovery_status=f'RETRY_{robot.retry_count}_SENT',
                reason=robot.recovery_reason
            )

            ok = self.send_nav_goal(robot_name, retry_pose, retry_desc)

            if not ok:
                # send_nav_goal 内部会再次调用 schedule_navigation_retry 或最终失败。
                recovering_found = True

        return recovering_found

    # ================================================================
    # AMCL 初始化
    # ================================================================

    def publish_initial_poses(self):
        """
        给所有机器人发布 AMCL 初始位姿
        """
        for robot_name in self.robot_names:
            pose = self.initial_poses[robot_name]
            msg = self.create_initial_pose_msg(pose)

            self.initialpose_pubs[robot_name].publish(msg)

            self.get_logger().info(
                f'发布 {robot_name} 初始位姿: '
                f'x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={pose[2]:.2f}'
            )

    # ================================================================
    # Nav2 action server 检查
    # ================================================================

    def all_nav_servers_ready(self):
        """
        检查所有机器人 NavigateToPose action server 是否就绪
        """
        for name, client in self.nav_clients.items():
            if not client.wait_for_server(timeout_sec=0.2):
                return False
        return True

    # ================================================================
    # 状态辅助函数
    # ================================================================

    def is_robot_moving(self, robot_name):
        """
        判断某台机器人是否正在执行导航动作
        """
        robot = self.robots[robot_name]

        return robot.state in [
            RobotState.GOING_TO_WAIT,
            RobotState.MOVING_TO_FRONT,
            RobotState.GOING_TO_WORKSTATION,
            RobotState.GOING_TO_EXIT
        ]

    def any_robot_moving(self):
        """
        判断当前是否有机器人正在移动
        """
        for name in self.robot_names:
            if self.is_robot_moving(name):
                return True
        return False

    def get_next_queue_robot(self):
        """
        获取队首机器人
        """
        if len(self.queue) == 0:
            return None
        return self.queue[0]

    # ================================================================
    # 共享资源锁辅助函数
    # ================================================================

    def is_workstation_free(self):
        """
        工作站是否空闲。
        注意：机器人从 P1 前往 WORKSTATION 开始，就认为工作站被预约/占用。
        直到机器人到达 EXIT 后才释放。
        """
        return self.workstation_occupied_by is None

    def is_exit_lane_free(self):
        """
        出口通道是否空闲。
        机器人从 WORKSTATION 前往 EXIT 期间占用出口通道。
        """
        return self.exit_lane_occupied_by is None

    def lock_workstation(self, robot_name):
        """
        预约/占用工作站。
        """
        self.workstation_occupied_by = robot_name
        self.current_working_robot = robot_name
        self.get_logger().info(
            f'[资源锁] WORKSTATION 已被 {robot_name} 占用/预约'
        )

    def lock_exit_lane(self, robot_name):
        """
        占用出口通道。
        """
        self.exit_lane_occupied_by = robot_name
        self.get_logger().info(
            f'[资源锁] EXIT_LANE 已被 {robot_name} 占用'
        )

    def release_resources_for_robot(self, robot_name):
        """
        释放某台机器人占用的共享资源。
        用于正常到达 EXIT，也用于异常失败时避免资源锁死。
        """
        released = []

        if self.workstation_occupied_by == robot_name:
            self.workstation_occupied_by = None
            released.append('WORKSTATION')

        if self.exit_lane_occupied_by == robot_name:
            self.exit_lane_occupied_by = None
            released.append('EXIT_LANE')

        if self.current_working_robot == robot_name:
            self.current_working_robot = None

        if released:
            self.get_logger().info(
                f'[资源锁] {robot_name} 释放资源: {", ".join(released)}'
            )

    # ================================================================
    # 发送导航目标
    # ================================================================

    def send_nav_goal(self, robot_name, pose, description):
        """
        给指定机器人发送导航目标点。

        第八步新增：
        每次发送目标前保存 last_goal_pose / last_motion_state，
        这样导航失败后可以自动重发同一个目标。
        """
        client = self.nav_clients[robot_name]
        robot = self.robots[robot_name]

        # 保存恢复所需信息。
        robot.last_goal_pose = pose
        robot.last_goal_description = description
        robot.last_motion_state = robot.state

        if not client.wait_for_server(timeout_sec=1.0):
            reason = 'NavigateToPose action server 未就绪'
            self.get_logger().warn(f'{robot_name} 的 {reason}')
            self.schedule_navigation_retry(robot_name, description, reason)
            return False

        goal_msg = self.create_goal_msg(pose)

        self.get_logger().info(
            f'发送目标给 {robot_name}: {description}, '
            f'x={pose[0]:.2f}, y={pose[1]:.2f}, yaw={pose[2]:.2f}'
        )

        send_goal_future = client.send_goal_async(goal_msg)

        send_goal_future.add_done_callback(
            lambda future, rn=robot_name, desc=description:
            self.goal_response_callback(future, rn, desc)
        )

        return True

    def goal_response_callback(self, future, robot_name, description):
        """
        Nav2 是否接受目标
        """
        robot = self.robots[robot_name]

        try:
            goal_handle = future.result()
        except Exception as e:
            reason = f'发送目标异常: {e}'
            self.get_logger().error(
                f'{robot_name} {reason}, description={description}'
            )
            self.schedule_navigation_retry(robot_name, description, reason)
            return

        if not goal_handle.accepted:
            reason = '目标被 Nav2 拒绝'
            self.get_logger().error(
                f'{robot_name} 的目标被拒绝: {description}'
            )
            self.schedule_navigation_retry(robot_name, description, reason)
            return

        self.get_logger().info(
            f'{robot_name} 的目标已被接受: {description}'
        )

        robot.goal_handle = goal_handle
        robot.result_future = goal_handle.get_result_async()

        robot.result_future.add_done_callback(
            lambda result_future, rn=robot_name, desc=description:
            self.result_callback(result_future, rn, desc)
        )

    def result_callback(self, future, robot_name, description):
        """
        导航结果回调
        """
        robot = self.robots[robot_name]

        try:
            result = future.result()
            status = result.status
        except Exception as e:
            reason = f'获取导航结果异常: {e}'
            self.get_logger().error(
                f'{robot_name} 获取导航结果异常: {description}, error={e}'
            )
            self.schedule_navigation_retry(robot_name, description, reason)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                f'{robot_name} 成功到达: {description}'
            )

            # 单个导航目标成功后，下一段目标重新计算重试次数。
            self.reset_retry_state_after_success(robot_name)

            if robot.state == RobotState.GOING_TO_WAIT:
                robot.arrived_wait = True
                robot.current_wait_slot_name = robot.target_wait_slot_name
                robot.state = RobotState.WAITING_IN_QUEUE

                # 第七步：记录到达初始等待点时间。
                self.set_record_field_once(robot_name, 'wait_arrival_time')
                self.update_record_status(robot_name, 'WAITING_IN_QUEUE')
                self.write_results_csv()

                self.get_logger().info(
                    f'{robot_name} 已到达 {robot.current_wait_slot_name}，进入等待队列'
                )

            elif robot.state == RobotState.MOVING_TO_FRONT:
                robot.current_wait_slot_name = robot.target_wait_slot_name
                robot.wait_pose = self.wait_slots[robot.current_wait_slot_name]
                robot.state = RobotState.WAITING_IN_QUEUE

                # 第七步：记录补位到 P1 的时间。
                self.set_record_field_once(robot_name, 'front_wait_arrival_time')
                self.update_record_status(robot_name, 'WAITING_AT_FRONT')
                self.write_results_csv()

                self.get_logger().info(
                    f'{robot_name} 已完成补位，当前等待点: {robot.current_wait_slot_name}'
                )

            elif robot.state == RobotState.GOING_TO_WORKSTATION:
                robot.arrived_work = True
                robot.state = RobotState.WORKING

                # 到达工作站后，工作站锁继续保持，直到该机器人到达 EXIT 才释放。
                self.current_working_robot = robot_name
                self.workstation_occupied_by = robot_name
                self.work_start_time[robot_name] = time.time()

                # 第七步：记录进入工作站/开始作业时间。
                self.set_record_field_once(robot_name, 'work_start_time')
                self.update_record_status(robot_name, 'WORKING')
                self.write_results_csv()

                self.get_logger().info(
                    f'{robot_name} 到达 WORKSTATION，开始作业，工作站锁保持占用'
                )

            elif robot.state == RobotState.GOING_TO_EXIT:
                robot.arrived_exit = True
                robot.state = RobotState.FINISHED

                # 第七步：记录到达出口/任务完成时间。
                self.set_record_field_once(robot_name, 'exit_time')
                self.update_record_status(robot_name, 'FINISHED')
                self.update_derived_metrics(robot_name)
                self.write_results_csv()

                # 到达出口后，释放工作站锁和出口通道锁。
                self.release_resources_for_robot(robot_name)

                self.get_logger().info(
                    f'{robot_name} 已到达 EXIT，任务完成，释放 WORKSTATION 与 EXIT_LANE'
                )

        elif status == GoalStatus.STATUS_CANCELED:
            reason = '导航被取消'
            self.get_logger().warn(
                f'{robot_name} 导航被取消: {description}'
            )
            self.schedule_navigation_retry(robot_name, description, reason)

        else:
            reason = f'导航失败，GoalStatus={status}'
            self.get_logger().error(
                f'{robot_name} 导航失败: {description}, status={status}'
            )
            self.schedule_navigation_retry(robot_name, description, reason)

    # ================================================================
    # 调度逻辑
    # ================================================================

    def start_demo(self):
        """
        开始正式排队调度

        半并行版：
        不在这里同时发送目标。
        main_loop 会按状态逐步放行机器人。
        """
        self.get_logger().info(
            '开始排队作业 demo：配置文件化 + 等待点补位 + 资源锁 + 可视化 + 实验日志 + 失败恢复版'
        )

        self.record_zero_time = time.time()
        self.write_results_csv()

        self.started = True

    def remove_failed_front_robot(self):
        """
        清理队列中已经最终失败的机器人，避免队列卡死。
        名字保留为 remove_failed_front_robot，是为了不影响主循环已有调用。
        """
        if len(self.queue) == 0:
            return

        old_queue = list(self.queue)
        self.queue = [name for name in self.queue if self.robots[name].state != RobotState.FAILED]

        removed = [name for name in old_queue if name not in self.queue]
        for name in removed:
            self.get_logger().warn(
                f'机器人 {name} 已 FAILED，从等待队列中移除，避免调度卡死'
            )

    def send_robot_to_its_initial_wait(self, robot_name):
        """
        发送机器人去自己的初始等待点：
        robot1 -> P1
        robot2 -> P2
        """
        robot = self.robots[robot_name]

        if robot.state != RobotState.IDLE:
            return False

        robot.target_wait_slot_name = robot.initial_wait_slot_name
        robot.wait_pose = robot.initial_wait_pose
        robot.state = RobotState.GOING_TO_WAIT

        # 第七步：第一次发送等待点目标时，记录该机器人任务开始时间。
        self.mark_robot_started(robot_name)

        ok = self.send_nav_goal(
            robot_name,
            robot.wait_pose,
            f'WAIT_SLOT_{robot.target_wait_slot_name}'
        )

        if not ok:
            return False

        return True

    def send_waiting_robot_to_front_slot(self, robot_name):
        """
        让后方等待点的机器人自动补位到队首等待点 P1。

        典型过程：
        robot2 已经在 P2 等待；
        robot1 从 P1 进入 WORKSTATION 后，P1 被释放；
        此时 robot2 从 P2 前移到 P1。
        """
        robot = self.robots[robot_name]

        if robot.state != RobotState.WAITING_IN_QUEUE:
            return False

        if robot.current_wait_slot_name == self.front_wait_slot_name:
            return False

        if self.any_robot_moving():
            return False

        front_pose = self.wait_slots[self.front_wait_slot_name]

        self.get_logger().info(
            f'{robot_name} 当前在 {robot.current_wait_slot_name}，'
            f'队首等待点 {self.front_wait_slot_name} 已释放，开始自动补位'
        )

        robot.target_wait_slot_name = self.front_wait_slot_name
        robot.wait_pose = front_pose
        robot.state = RobotState.MOVING_TO_FRONT

        ok = self.send_nav_goal(
            robot_name,
            front_pose,
            f'MOVE_FORWARD_TO_{self.front_wait_slot_name}'
        )

        if not ok:
            return False

        return True

    def ensure_initial_waiting_order_ready(self):
        """
        正式进入工作站前，先建立等待队列：
        1. robot1 去 P1
        2. robot2 去 P2

        为了降低仿真中两车在狭窄通道相遇的概率，这里采用顺序放行：
        前一个到达等待点后，再放行下一个。
        """
        for robot_name in self.queue:
            robot = self.robots[robot_name]

            if robot.state == RobotState.IDLE:
                if self.any_robot_moving():
                    return False

                self.get_logger().info(
                    f'建立初始等待队列：放行 {robot_name} 去 {robot.initial_wait_slot_name}'
                )

                self.send_robot_to_its_initial_wait(robot_name)
                return False

            if robot.state in [RobotState.GOING_TO_WAIT, RobotState.MOVING_TO_FRONT]:
                self.get_logger().info(
                    f'等待 {robot_name} 到达等待点，当前状态: {robot.state.name}'
                )
                return False

            if robot.state in [RobotState.RECOVERING, RobotState.FAILED]:
                return False

        return True

    def maybe_move_next_robot_to_front_slot(self):
        """
        当队首机器人已经进入 WORKSTATION 后，
        队列中的下一台机器人从 P2 自动补位到 P1。
        """
        next_robot_name = self.get_next_queue_robot()

        if next_robot_name is None:
            return

        next_robot = self.robots[next_robot_name]

        # 还没到初始等待点时，不做补位
        if next_robot.state != RobotState.WAITING_IN_QUEUE:
            return

        # 已经在 P1，无需补位
        if next_robot.current_wait_slot_name == self.front_wait_slot_name:
            return

        self.send_waiting_robot_to_front_slot(next_robot_name)

    def main_loop(self):
        """
        主循环

        初始化阶段：
        1. 等 NavigateToPose action server
        2. 自动发布 robot1 / robot2 initialpose
        3. 等 AMCL 稳定
        4. 进入正式调度

        正式调度阶段：
        robot1 可以先进入工作站；
        robot1 工作时，robot2 可以提前去等待点；
        robot1 到达 EXIT 后，robot2 才能进入 WORKSTATION。
        """

        # 第六步：无论初始化阶段还是正式调度阶段，都持续发布状态与 RViz Marker。
        self.publish_status_and_markers()

        # ============================================================
        # 1. 初始化阶段
        # ============================================================

        if not self.started:

            if not self.all_nav_servers_ready():
                self.get_logger().info('继续等待 Nav2 action server...')
                return

            if self.initial_pose_publish_count < self.initial_pose_publish_times:
                self.publish_initial_poses()
                self.initial_pose_publish_count += 1

                self.get_logger().info(
                    f'正在初始化 AMCL 位姿: '
                    f'{self.initial_pose_publish_count}/{self.initial_pose_publish_times}'
                )
                return

            if not self.initial_pose_done:
                self.initial_pose_done = True
                self.initial_pose_done_time = time.time()

                self.get_logger().info(
                    f'初始位姿发布完成，等待 {self.initial_pose_wait_time} 秒让 AMCL 稳定'
                )
                return

            elapsed_amcl = time.time() - self.initial_pose_done_time

            if elapsed_amcl < self.initial_pose_wait_time:
                self.get_logger().info(
                    f'等待 AMCL 稳定: {elapsed_amcl:.1f}/{self.initial_pose_wait_time:.1f} 秒'
                )
                return

            self.get_logger().info('AMCL 初始化完成，进入等待点补位调度')
            self.start_demo()
            return

        # ============================================================
        # 2. 正式调度阶段
        # ============================================================

        self.print_status()

        self.remove_failed_front_robot()

        # 第八步：优先处理失败恢复。恢复期间暂停新的调度决策，
        # 防止其他机器人趁恢复机器人停在通道中时继续进入冲突区域。
        if self.process_recovering_robots():
            return

        # ------------------------------------------------------------
        # A. 如果当前有机器人预约/占用工作站
        # ------------------------------------------------------------

        if self.current_working_robot is not None:
            robot_name = self.current_working_robot
            robot = self.robots[robot_name]

            # 当前机器人正在去工作站，其他机器人不动
            if robot.state == RobotState.GOING_TO_WORKSTATION:
                self.get_logger().info(
                    f'工作站已被 {robot_name} 预约，等待其到达 WORKSTATION'
                )
                return

            # 当前机器人正在工作
            if robot.state == RobotState.WORKING:
                elapsed = time.time() - self.work_start_time[robot_name]

                # 1. 工作期间，允许下一台机器人从 P2 自动补位到 P1。
                #    注意：这里只允许补位到等待点，不允许进入 WORKSTATION。
                self.maybe_move_next_robot_to_front_slot()

                # 2. 作业时间到了，准备离开
                if elapsed >= self.work_duration:

                    # 如果此时下一台机器人还在去 WAIT_POINT 的路上，
                    # 先等它到达 WAIT_POINT，避免 robot1 出站和 robot2 进等待点同时运动撞车。
                    if self.any_robot_moving():
                        self.get_logger().info(
                            f'{robot_name} 作业已完成，但有机器人正在移动，等待其到达 WAIT_POINT 后再离开'
                        )
                        return

                    # 出口通道必须空闲，当前机器人才能从 WORKSTATION 离开。
                    if not self.is_exit_lane_free():
                        self.get_logger().info(
                            f'{robot_name} 作业已完成，但 EXIT_LANE 被 {self.exit_lane_occupied_by} 占用，继续等待'
                        )
                        return

                    # 第七步：记录作业完成时间。
                    self.set_record_field_once(robot_name, 'work_finish_time')
                    self.update_record_status(robot_name, 'GOING_TO_EXIT')
                    self.write_results_csv()

                    self.get_logger().info(
                        f'{robot_name} 作业完成，准备占用 EXIT_LANE 并离开 WORKSTATION'
                    )

                    self.lock_exit_lane(robot_name)
                    robot.state = RobotState.GOING_TO_EXIT

                    ok = self.send_nav_goal(
                        robot_name,
                        robot.exit_pose,
                        'EXIT'
                    )

                    if not ok:
                        self.get_logger().warn(
                            f'{robot_name} 发送 EXIT 目标失败，已进入失败恢复流程'
                        )

                return

            # 当前机器人正在离开工作站，其他机器人不动
            if robot.state == RobotState.GOING_TO_EXIT:
                self.get_logger().info(
                    f'{robot_name} 正在占用 EXIT_LANE 离开 WORKSTATION，等待其到达 EXIT'
                )
                return

        # ------------------------------------------------------------
        # B. 工作站空闲：先建立等待队列，再允许队首进入工作站
        # ------------------------------------------------------------

        if len(self.queue) == 0:
            self.get_logger().info('队列为空，所有可执行任务已完成')
            if not self.result_summary_printed:
                self.print_result_summary()
                self.result_summary_printed = True
            return

        # 第一次进入正式调度时：
        # robot1 先去 P1，robot2 再去 P2。
        # 两台机器人都到达等待点后，才允许 robot1 进入工作站。
        if not self.ensure_initial_waiting_order_ready():
            return

        first_robot_name = self.queue[0]
        first_robot = self.robots[first_robot_name]

        # 队首机器人必须在 P1，才能进入工作站。
        # 如果它还在 P2/P3，说明需要先补位。
        if first_robot.state == RobotState.WAITING_IN_QUEUE:

            if first_robot.current_wait_slot_name != self.front_wait_slot_name:
                self.get_logger().info(
                    f'队首机器人 {first_robot_name} 当前在 {first_robot.current_wait_slot_name}，'
                    f'需要先补位到 {self.front_wait_slot_name}'
                )
                self.send_waiting_robot_to_front_slot(first_robot_name)
                return

            if self.any_robot_moving():
                self.get_logger().info(
                    f'当前有机器人正在移动，暂不允许 {first_robot_name} 进入 WORKSTATION'
                )
                return

            # 第五步新增：工作站和出口通道都必须空闲，队首机器人才能进站。
            # 这样可以避免 robot1 出站时，robot2 同时从 P1 进站造成路径冲突。
            if not self.is_workstation_free():
                self.get_logger().info(
                    f'WORKSTATION 被 {self.workstation_occupied_by} 占用，{first_robot_name} 继续在 {self.front_wait_slot_name} 等待'
                )
                return

            if not self.is_exit_lane_free():
                self.get_logger().info(
                    f'EXIT_LANE 被 {self.exit_lane_occupied_by} 占用，{first_robot_name} 暂不进入 WORKSTATION'
                )
                return

            self.get_logger().info(
                f'工作站与出口通道均空闲，允许队首机器人 {first_robot_name} 从 {self.front_wait_slot_name} 进入 WORKSTATION'
            )

            first_robot.state = RobotState.GOING_TO_WORKSTATION

            # 预约/占用工作站
            self.lock_workstation(first_robot_name)

            # 从队列弹出。
            # 注意：此时 P1 被释放，后续 robot2 可以从 P2 补位到 P1。
            self.queue.pop(0)

            ok = self.send_nav_goal(
                first_robot_name,
                first_robot.work_pose,
                'WORKSTATION'
            )

            if not ok:
                self.get_logger().warn(
                    f'{first_robot_name} 发送 WORKSTATION 目标失败，已进入失败恢复流程'
                )

            return

        # 其他状态等待
        self.get_logger().info(
            f'等待队首机器人 {first_robot_name} 当前动作完成，状态: {first_robot.state.name}'
        )


    # ================================================================
    # 第六步：状态话题 + RViz Marker 可视化
    # ================================================================

    def build_status_text(self):
        """
        构造 /queue_manager/status 文本。
        这个话题主要给命令行和调试使用：
            ros2 topic echo /queue_manager/status
        """
        queue_str = ' -> '.join(self.queue) if self.queue else 'EMPTY'

        working = self.current_working_robot if self.current_working_robot else 'None'
        workstation = self.workstation_occupied_by if self.workstation_occupied_by else 'None'
        exit_lane = self.exit_lane_occupied_by if self.exit_lane_occupied_by else 'None'

        lines = []
        lines.append('========== Multi Robot Queue Status ==========')
        lines.append(f'Queue: {queue_str}')
        lines.append(f'Current working robot: {working}')
        lines.append(f'WORKSTATION_LOCK: {workstation}')
        lines.append(f'EXIT_LANE_LOCK: {exit_lane}')
        lines.append(f'CSV_RESULT: {self.result_csv_path}')
        lines.append('')

        for name in self.robot_names:
            robot = self.robots[name]
            slot = robot.current_wait_slot_name if robot.current_wait_slot_name else '-'
            target_slot = robot.target_wait_slot_name if robot.target_wait_slot_name else '-'

            recovery_text = ''
            if robot.state == RobotState.RECOVERING:
                remaining = 0.0
                if robot.retry_after_time is not None:
                    remaining = max(0.0, robot.retry_after_time - time.time())
                recovery_text = (
                    f', retry={robot.retry_count}/{self.max_retry_count}, '
                    f'retry_in={remaining:.1f}s, last_goal={robot.last_goal_description}'
                )

            lines.append(
                f'{name}: state={robot.state.name}, '
                f'current_slot={slot}, target_slot={target_slot}, '
                f'arrived_wait={robot.arrived_wait}, '
                f'arrived_work={robot.arrived_work}, '
                f'arrived_exit={robot.arrived_exit}'
                f'{recovery_text}'
            )

        return '\n'.join(lines)

    def publish_status(self):
        """
        发布调度状态文本。
        """
        msg = String()
        msg.data = self.build_status_text()
        self.status_pub.publish(msg)

    def fill_marker_common(self, marker, marker_id, namespace):
        """
        填充 Marker 通用字段。
        """
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.lifetime.sec = self.marker_lifetime_sec
        marker.lifetime.nanosec = 0
        return marker

    def set_marker_pose(self, marker, pose, z=0.05):
        """
        设置 marker 位姿。
        pose: (x, y, yaw)
        """
        x, y, yaw = pose
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        qz, qw = yaw_to_quaternion(yaw)
        marker.pose.orientation.z = qz
        marker.pose.orientation.w = qw
        return marker

    def set_marker_color(self, marker, r, g, b, a=0.85):
        """
        设置 marker 颜色。
        RViz 中颜色用于区分：等待点、工作站、出口、机器人状态。
        """
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(a)
        return marker

    def create_cylinder_marker(self, marker_id, pose, radius, height, r, g, b, namespace='queue_points'):
        """
        创建圆柱 marker，用于显示 P1/P2/P3 等等待点。
        """
        marker = Marker()
        self.fill_marker_common(marker, marker_id, namespace)
        marker.type = Marker.CYLINDER
        self.set_marker_pose(marker, pose, z=height / 2.0)
        marker.scale.x = radius
        marker.scale.y = radius
        marker.scale.z = height
        self.set_marker_color(marker, r, g, b, 0.65)
        return marker

    def create_cube_marker(self, marker_id, pose, sx, sy, sz, r, g, b, namespace='queue_points'):
        """
        创建方块 marker，用于显示 WORKSTATION / EXIT 等区域。
        """
        marker = Marker()
        self.fill_marker_common(marker, marker_id, namespace)
        marker.type = Marker.CUBE
        self.set_marker_pose(marker, pose, z=sz / 2.0)
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        self.set_marker_color(marker, r, g, b, 0.55)
        return marker

    def create_text_marker(self, marker_id, pose, text, z=0.7, size=0.22, namespace='queue_text'):
        """
        创建文字 marker。
        """
        marker = Marker()
        self.fill_marker_common(marker, marker_id, namespace)
        marker.type = Marker.TEXT_VIEW_FACING
        self.set_marker_pose(marker, pose, z=z)
        marker.scale.z = size
        self.set_marker_color(marker, 1.0, 1.0, 1.0, 1.0)
        marker.text = text
        return marker

    def get_robot_display_pose(self, robot_name):
        """
        根据调度状态估计机器人当前显示位置。
        注意：这里显示的是“调度状态位置”，不是实时定位轨迹。
        实时机器人位置仍然看 Gazebo 模型或 TF。
        """
        robot = self.robots[robot_name]

        if robot.state == RobotState.IDLE:
            return self.initial_poses[robot_name]

        if robot.state in [RobotState.GOING_TO_WAIT, RobotState.MOVING_TO_FRONT, RobotState.WAITING_IN_QUEUE]:
            return robot.wait_pose

        if robot.state in [RobotState.GOING_TO_WORKSTATION, RobotState.WORKING]:
            return robot.work_pose

        if robot.state in [RobotState.GOING_TO_EXIT, RobotState.FINISHED]:
            return robot.exit_pose

        if robot.state == RobotState.RECOVERING:
            if robot.last_goal_pose is not None:
                return robot.last_goal_pose
            return self.initial_poses[robot_name]

        return self.initial_poses[robot_name]

    def get_robot_state_color(self, state):
        """
        根据机器人状态返回颜色。
        返回值：(r, g, b)
        """
        if state == RobotState.IDLE:
            return 0.6, 0.6, 0.6
        if state in [RobotState.GOING_TO_WAIT, RobotState.MOVING_TO_FRONT, RobotState.GOING_TO_WORKSTATION, RobotState.GOING_TO_EXIT]:
            return 0.2, 0.6, 1.0
        if state == RobotState.WAITING_IN_QUEUE:
            return 1.0, 0.8, 0.1
        if state == RobotState.WORKING:
            return 0.1, 1.0, 0.2
        if state == RobotState.FINISHED:
            return 0.5, 1.0, 0.5
        if state == RobotState.RECOVERING:
            return 1.0, 0.45, 0.05
        if state == RobotState.FAILED:
            return 1.0, 0.1, 0.1
        return 1.0, 1.0, 1.0

    def publish_markers(self):
        """
        发布 RViz MarkerArray。
        RViz 添加方式：
            Add -> By topic -> /queue_manager/markers
        """
        markers = MarkerArray()
        marker_id = 0

        # 先清空旧 marker，避免重启节点或减少 marker 后 RViz 残留。
        delete_all = Marker()
        delete_all.header.frame_id = 'map'
        delete_all.header.stamp = self.get_clock().now().to_msg()
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        # 等待点 P1/P2/P3
        for slot_name, pose in self.wait_slots.items():
            markers.markers.append(
                self.create_cylinder_marker(
                    marker_id, pose, 0.45, 0.06,
                    0.2, 0.8, 1.0,
                    namespace='wait_slots'
                )
            )
            marker_id += 1

            markers.markers.append(
                self.create_text_marker(
                    marker_id, pose,
                    f'{slot_name}\nWAIT',
                    z=0.55,
                    size=0.22,
                    namespace='wait_slot_labels'
                )
            )
            marker_id += 1

        # 工作站
        workstation_pose = self.workstation_pose
        workstation_text = 'WORKSTATION'
        if self.workstation_occupied_by:
            workstation_text += f'\nLOCK: {self.workstation_occupied_by}'
        else:
            workstation_text += '\nLOCK: FREE'

        markers.markers.append(
            self.create_cube_marker(
                marker_id, workstation_pose, 0.75, 0.55, 0.12,
                1.0, 0.4, 0.1,
                namespace='workstation'
            )
        )
        marker_id += 1

        markers.markers.append(
            self.create_text_marker(
                marker_id, workstation_pose,
                workstation_text,
                z=0.8,
                size=0.22,
                namespace='workstation_label'
            )
        )
        marker_id += 1

        # 出口区域：根据 robots 配置中的 exit_pose 动态显示
        exit_items = [
            (f'EXIT_{robot_name}', self.waypoints[robot_name]['exit'])
            for robot_name in self.robot_names
        ]

        exit_text = 'EXIT_LANE'
        if self.exit_lane_occupied_by:
            exit_text += f'\nLOCK: {self.exit_lane_occupied_by}'
        else:
            exit_text += '\nLOCK: FREE'

        for exit_name, pose in exit_items:
            markers.markers.append(
                self.create_cube_marker(
                    marker_id, pose, 0.45, 0.45, 0.08,
                    0.2, 1.0, 0.5,
                    namespace='exit_points'
                )
            )
            marker_id += 1

            markers.markers.append(
                self.create_text_marker(
                    marker_id, pose,
                    exit_name,
                    z=0.55,
                    size=0.18,
                    namespace='exit_labels'
                )
            )
            marker_id += 1

        # 出口通道锁文字放在所有出口点的中心位置
        avg_x = sum(pose[0] for _, pose in exit_items) / len(exit_items)
        avg_y = sum(pose[1] for _, pose in exit_items) / len(exit_items)
        exit_center = (avg_x, avg_y, -1.57)

        markers.markers.append(
            self.create_text_marker(
                marker_id, exit_center,
                exit_text,
                z=0.95,
                size=0.22,
                namespace='exit_lane_label'
            )
        )
        marker_id += 1

        # 机器人调度状态文字
        for robot_name in self.robot_names:
            robot = self.robots[robot_name]
            pose = self.get_robot_display_pose(robot_name)
            r, g, b = self.get_robot_state_color(robot.state)

            markers.markers.append(
                self.create_cylinder_marker(
                    marker_id, pose, 0.28, 0.16,
                    r, g, b,
                    namespace='robot_state_disks'
                )
            )
            marker_id += 1

            slot = robot.current_wait_slot_name if robot.current_wait_slot_name else '-'
            text = f'{robot_name}\n{robot.state.name}\nslot:{slot}'
            if robot.state == RobotState.RECOVERING:
                text += f'\nretry:{robot.retry_count}/{self.max_retry_count}'

            # 给 robot1/robot2 的文字稍微错开，避免在 WORKSTATION/EXIT 附近完全重叠。
            text_pose = (pose[0], pose[1] + (0.25 if robot_name == 'robot1' else -0.25), pose[2])

            markers.markers.append(
                self.create_text_marker(
                    marker_id, text_pose,
                    text,
                    z=1.05,
                    size=0.18,
                    namespace='robot_state_text'
                )
            )
            marker_id += 1

        # 总状态文字放在地图左上方，方便一眼看整体调度状态。
        queue_str = ' -> '.join(self.queue) if self.queue else 'EMPTY'
        summary_text = (
            f'Queue: {queue_str}\n'
            f'WORKSTATION: {self.workstation_occupied_by if self.workstation_occupied_by else "FREE"}\n'
            f'EXIT_LANE: {self.exit_lane_occupied_by if self.exit_lane_occupied_by else "FREE"}'
        )
        summary_pose = self.summary_pose
        markers.markers.append(
            self.create_text_marker(
                marker_id, summary_pose,
                summary_text,
                z=1.0,
                size=0.22,
                namespace='queue_summary'
            )
        )
        marker_id += 1

        self.marker_pub.publish(markers)

    def publish_status_and_markers(self):
        """
        统一发布状态文本与 RViz Marker。
        """
        self.publish_status()
        self.publish_markers()

    def print_status(self):
        """
        打印当前队列状态
        """
        states = []

        for name in self.robot_names:
            robot = self.robots[name]
            slot = robot.current_wait_slot_name
            if slot is None:
                slot = '-'
            retry_text = ''
            if robot.state == RobotState.RECOVERING:
                retry_text = f',retry={robot.retry_count}/{self.max_retry_count}'
            states.append(f'{name}:{robot.state.name}(slot={slot}{retry_text})')

        queue_str = ' -> '.join(self.queue) if self.queue else 'EMPTY'

        working = self.current_working_robot
        if working is None:
            working = 'None'

        workstation = self.workstation_occupied_by
        if workstation is None:
            workstation = 'None'

        exit_lane = self.exit_lane_occupied_by
        if exit_lane is None:
            exit_lane = 'None'

        self.get_logger().info(
            f'[状态] 队列: {queue_str} | 主流程: {working} | '
            f'WORKSTATION_LOCK: {workstation} | EXIT_LANE_LOCK: {exit_lane} | '
            + ' | '.join(states)
        )


def main(args=None):
    rclpy.init(args=args)

    node = MultiRobotQueueDemo()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('用户中断 multi_robot_queue_demo')
    finally:
        try:
            node.write_results_csv()
            node.get_logger().info(f'实验日志已保存: {node.result_csv_path}')
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()