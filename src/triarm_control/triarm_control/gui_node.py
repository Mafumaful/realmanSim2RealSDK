"""
关节控制GUI模块

独立的GUI界面，通过ROS2话题与控制节点通信
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

try:
    import tkinter as tk
    from tkinter import ttk
    HAS_TK = True
except ImportError:
    HAS_TK = False

from .joint_names import JOINT_NAMES_LIST, JOINT_LIMITS


class JointControlGUI(Node):
    """关节控制GUI节点"""

    def __init__(self):
        super().__init__('triarm_gui')

        # 声明参数
        self.declare_parameter('namespace', '')
        self.declare_parameter('gui_width', 700)
        self.declare_parameter('gui_height', 500)

        # 获取参数
        ns = self.get_parameter('namespace').value
        self.gui_width = self.get_parameter('gui_width').value
        self.gui_height = self.get_parameter('gui_height').value

        # 话题名称
        prefix = f'{ns}/' if ns else '/'
        state_topic = f'{prefix}joint_states'
        target_topic = f'{prefix}target_joints'
        cmd_topic = f'{prefix}joint_command'

        # 订阅实时状态
        self.state_sub = self.create_subscription(
            JointState, state_topic, self._state_callback, 10)

        # 订阅指令反馈
        self.cmd_sub = self.create_subscription(
            JointState, cmd_topic, self._cmd_callback, 10)

        # 发布目标位置
        self.target_pub = self.create_publisher(Float64MultiArray, target_topic, 10)

        # 发布控制参数（用于动态修改）
        from std_msgs.msg import String, Bool
        self.smooth_pub = self.create_publisher(Bool, f'{prefix}enable_smooth', 10)
        self.velocity_mode_pub = self.create_publisher(String, f'{prefix}velocity_mode', 10)

        # 发布夹爪控制指令
        self.gripper_pub = self.create_publisher(String, f'{prefix}gripper_control', 10)

        # 状态
        self.current_positions = [0.0] * 19
        self.cmd_positions = [0.0] * 19
        self.has_state = False

        # GUI组件
        self.entries = []
        self.sliders = []
        self.realtime_labels = []
        self.cmd_labels = []

        self._create_gui()

    def _create_gui(self):
        """创建GUI界面"""
        self.root = tk.Tk()
        self.root.title('三臂机器人关节控制')
        self.root.geometry(f'{self.gui_width}x{self.gui_height}')

        # Notebook分页
        notebook = ttk.Notebook(self.root)
        notebook.pack(fill='both', expand=True, padx=5, pady=5)

        # 创建各组标签页
        self._create_platform_tab(notebook)
        self._create_arm_tab(notebook, 'A', 1)
        self._create_arm_tab(notebook, 'B', 7)
        self._create_arm_tab(notebook, 'S', 13)

        # 按钮区域
        self._create_buttons()

    def _create_buttons(self):
        """创建按钮区域"""
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill='x', padx=5, pady=5)

        # 左侧：执行和归零按钮
        ttk.Button(btn_frame, text='执行',
                   command=self._send_target).pack(side='left', padx=5)
        ttk.Button(btn_frame, text='归零',
                   command=self._reset_all).pack(side='left', padx=5)

        # 中间：平滑处理勾选框
        self.smooth_var = tk.BooleanVar(value=True)
        smooth_check = ttk.Checkbutton(
            btn_frame, text='平滑处理',
            variable=self.smooth_var,
            command=self._on_smooth_changed
        )
        smooth_check.pack(side='left', padx=15)

        # 速度模式选择
        ttk.Label(btn_frame, text='速度:').pack(side='left', padx=(15, 5))
        self.velocity_mode = tk.StringVar(value='normal')
        velocity_combo = ttk.Combobox(
            btn_frame,
            textvariable=self.velocity_mode,
            values=['slow', 'normal', 'fast'],
            state='readonly',
            width=8
        )
        velocity_combo.pack(side='left', padx=5)
        velocity_combo.bind('<<ComboboxSelected>>', self._on_velocity_changed)

        # 右侧：状态标签
        self.status_label = ttk.Label(btn_frame, text='就绪', foreground='green')
        self.status_label.pack(side='right', padx=10)

        # 夹爪控制区域
        self._create_gripper_controls()

    def _create_gripper_controls(self):
        """创建夹爪控制区域"""
        gripper_frame = ttk.LabelFrame(self.root, text='夹爪控制', padding=10)
        gripper_frame.pack(fill='x', padx=5, pady=5)

        # 左夹爪控制
        left_frame = ttk.Frame(gripper_frame)
        left_frame.pack(side='left', padx=20)
        ttk.Label(left_frame, text='左夹爪:', font=('', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)

        # 左夹爪按钮
        btn_frame_left = ttk.Frame(left_frame)
        btn_frame_left.grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_left, text='打开',
                   command=lambda: self._send_gripper_command('left_open')).pack(side='left', padx=2)
        ttk.Button(btn_frame_left, text='闭合',
                   command=lambda: self._send_gripper_command('left_close')).pack(side='left', padx=2)

        # 左夹爪角度设置
        ttk.Label(left_frame, text='打开角度:').grid(row=2, column=0, sticky='e', padx=5, pady=2)
        self.left_open_entry = ttk.Entry(left_frame, width=8)
        self.left_open_entry.insert(0, '0.0')
        self.left_open_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(left_frame, text='闭合角度:').grid(row=3, column=0, sticky='e', padx=5, pady=2)
        self.left_close_entry = ttk.Entry(left_frame, width=8)
        self.left_close_entry.insert(0, '30.0')
        self.left_close_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Button(left_frame, text='应用参数',
                   command=lambda: self._update_gripper_params('left')).grid(row=4, column=0, columnspan=2, pady=5)

        # 右夹爪控制
        right_frame = ttk.Frame(gripper_frame)
        right_frame.pack(side='left', padx=20)
        ttk.Label(right_frame, text='右夹爪:', font=('', 10, 'bold')).grid(row=0, column=0, columnspan=2, pady=5)

        # 右夹爪按钮
        btn_frame_right = ttk.Frame(right_frame)
        btn_frame_right.grid(row=1, column=0, columnspan=2, pady=5)
        ttk.Button(btn_frame_right, text='打开',
                   command=lambda: self._send_gripper_command('right_open')).pack(side='left', padx=2)
        ttk.Button(btn_frame_right, text='闭合',
                   command=lambda: self._send_gripper_command('right_close')).pack(side='left', padx=2)

        # 右夹爪角度设置
        ttk.Label(right_frame, text='打开角度:').grid(row=2, column=0, sticky='e', padx=5, pady=2)
        self.right_open_entry = ttk.Entry(right_frame, width=8)
        self.right_open_entry.insert(0, '0.0')
        self.right_open_entry.grid(row=2, column=1, padx=5, pady=2)

        ttk.Label(right_frame, text='闭合角度:').grid(row=3, column=0, sticky='e', padx=5, pady=2)
        self.right_close_entry = ttk.Entry(right_frame, width=8)
        self.right_close_entry.insert(0, '30.0')
        self.right_close_entry.grid(row=3, column=1, padx=5, pady=2)

        ttk.Button(right_frame, text='应用参数',
                   command=lambda: self._update_gripper_params('right')).grid(row=4, column=0, columnspan=2, pady=5)

    def _add_header_row(self, parent):
        """添加表头"""
        headers = [('关节', 4), ('实时值', 8), ('指令值', 8), ('滑块控制', 20), ('目标角度', 8)]
        colors = [None, 'blue', 'green', None, None]
        for col, (text, width) in enumerate(headers):
            lbl = ttk.Label(parent, text=text, width=width, font=('', 9, 'bold'))
            if colors[col]:
                lbl.config(foreground=colors[col])
            lbl.grid(row=0, column=col, padx=2, pady=5)

    def _create_platform_tab(self, notebook):
        """创建平台标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text='平台')
        self._add_header_row(frame)

        name = JOINT_NAMES_LIST[0]
        limits = JOINT_LIMITS[name]
        min_deg, max_deg = math.degrees(limits[0]), math.degrees(limits[1])
        self._add_joint_row(frame, 1, 'D1', min_deg, max_deg, 0)

    def _create_arm_tab(self, notebook, arm_name, start_idx):
        """创建机械臂标签页"""
        frame = ttk.Frame(notebook)
        notebook.add(frame, text=f'机械臂{arm_name}')
        self._add_header_row(frame)

        for i in range(6):
            idx = start_idx + i
            name = JOINT_NAMES_LIST[idx]
            limits = JOINT_LIMITS[name]
            min_deg, max_deg = math.degrees(limits[0]), math.degrees(limits[1])
            self._add_joint_row(frame, i+1, f'{arm_name}{i+1}', min_deg, max_deg, idx)

    def _add_joint_row(self, parent, row, label, min_val, max_val, idx):
        """添加关节控制行"""
        # 关节名称
        ttk.Label(parent, text=label, width=4).grid(row=row, column=0, padx=2, pady=2)

        # 实时值
        rt_label = ttk.Label(parent, text='--', width=8, foreground='blue')
        rt_label.grid(row=row, column=1, padx=2, pady=2)
        self.realtime_labels.append((idx, rt_label))

        # 指令值
        cmd_label = ttk.Label(parent, text='--', width=8, foreground='green')
        cmd_label.grid(row=row, column=2, padx=2, pady=2)
        self.cmd_labels.append((idx, cmd_label))

        # 滑块
        var = tk.DoubleVar(value=0.0)
        slider = ttk.Scale(parent, from_=min_val, to=max_val, variable=var, length=200)
        slider.grid(row=row, column=3, padx=2, pady=2)
        self.sliders.append((idx, var, min_val, max_val))

        # 输入框
        entry = ttk.Entry(parent, width=8)
        entry.insert(0, '0.0')
        entry.grid(row=row, column=4, padx=2, pady=2)
        self.entries.append((idx, entry, min_val, max_val))

        # 双向同步
        var.trace_add('write', lambda *_, v=var, e=entry: self._sync_entry(v, e))
        entry.bind('<Return>', lambda ev, v=var, e=entry: self._sync_slider(v, e))

    def _sync_entry(self, var, entry):
        entry.delete(0, tk.END)
        entry.insert(0, f'{var.get():.1f}')

    def _sync_slider(self, var, entry):
        try:
            var.set(float(entry.get()))
        except ValueError:
            pass

    def _state_callback(self, msg: JointState):
        """处理实时状态"""
        if msg.name and msg.position:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self.current_positions[idx] = msg.position[i]
            self.has_state = True
            self._update_realtime_labels()

    def _cmd_callback(self, msg: JointState):
        """处理指令反馈"""
        if msg.name and msg.position:
            for i, name in enumerate(msg.name):
                if i < len(msg.position) and name in JOINT_NAMES_LIST:
                    idx = JOINT_NAMES_LIST.index(name)
                    self.cmd_positions[idx] = msg.position[i]
            self._update_cmd_labels()

    def _update_realtime_labels(self):
        for idx, label in self.realtime_labels:
            label.config(text=f'{math.degrees(self.current_positions[idx]):.1f}°')

    def _update_cmd_labels(self):
        for idx, label in self.cmd_labels:
            label.config(text=f'{math.degrees(self.cmd_positions[idx]):.1f}°')

    def _on_smooth_changed(self):
        """平滑处理勾选框变化"""
        from std_msgs.msg import Bool
        msg = Bool()
        msg.data = self.smooth_var.get()
        self.smooth_pub.publish(msg)
        status = "启用" if msg.data else "禁用"
        self.get_logger().info(f'平滑处理已{status}')

    def _on_velocity_changed(self, event=None):
        """速度模式变化"""
        from std_msgs.msg import String
        msg = String()
        msg.data = self.velocity_mode.get()
        self.velocity_mode_pub.publish(msg)
        self.get_logger().info(f'速度模式: {msg.data}')

    def _send_target(self):
        """发送目标位置"""
        if not self.has_state:
            self.status_label.config(text='等待数据...', foreground='red')
            return

        targets = [0.0] * 19
        for idx, entry, min_val, max_val in self.entries:
            try:
                val = float(entry.get())
                val = max(min_val, min(max_val, val))
                targets[idx] = val
            except ValueError:
                pass

        msg = Float64MultiArray()
        msg.data = targets
        self.target_pub.publish(msg)
        self.status_label.config(text='已发送', foreground='orange')

    def _reset_all(self):
        """归零"""
        for idx, entry, _, _ in self.entries:
            entry.delete(0, tk.END)
            entry.insert(0, '0.0')
        # 同步更新滑块
        for idx, var, _, _ in self.sliders:
            var.set(0.0)

    def _send_gripper_command(self, command: str):
        """发送夹爪控制指令"""
        from std_msgs.msg import String
        msg = String()
        msg.data = command
        self.gripper_pub.publish(msg)
        self.get_logger().info(f'发送夹爪指令: {command}')

    def _update_gripper_params(self, side: str):
        """更新夹爪参数（热加载）"""
        from rcl_interfaces.srv import SetParameters
        from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

        try:
            if side == 'left':
                open_angle = float(self.left_open_entry.get())
                close_angle = float(self.left_close_entry.get())

                # 创建参数列表
                params = [
                    Parameter(
                        name='left_gripper.open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name='left_gripper.close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新左夹爪参数: 打开={open_angle}°, 闭合={close_angle}°')

            elif side == 'right':
                open_angle = float(self.right_open_entry.get())
                close_angle = float(self.right_close_entry.get())

                params = [
                    Parameter(
                        name='right_gripper.open_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=open_angle)
                    ),
                    Parameter(
                        name='right_gripper.close_angle',
                        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=close_angle)
                    )
                ]
                self.get_logger().info(f'更新右夹爪参数: 打开={open_angle}°, 闭合={close_angle}°')

            # 创建服务客户端
            client = self.create_client(SetParameters, '/triarm_controller/set_parameters')

            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('参数服务不可用')
                return

            # 发送请求
            request = SetParameters.Request()
            request.parameters = params
            future = client.call_async(request)

            # 简单等待结果
            import time
            start = time.time()
            while not future.done() and (time.time() - start) < 1.0:
                rclpy.spin_once(self, timeout_sec=0.01)

            if future.done():
                response = future.result()
                if response.results[0].successful:
                    self.get_logger().info('参数更新成功')
                else:
                    self.get_logger().warn(f'参数更新失败: {response.results[0].reason}')

        except ValueError:
            self.get_logger().error('输入的角度值无效')
        except Exception as e:
            self.get_logger().error(f'更新参数时出错: {str(e)}')

    def run(self):
        """运行GUI"""
        def ros_spin():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.after(10, ros_spin)

        self.root.after(10, ros_spin)
        self.root.mainloop()


def main(args=None):
    if not HAS_TK:
        print('错误: 需要安装tkinter')
        return

    rclpy.init(args=args)
    node = JointControlGUI()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
