# 基座坐标系变换方案

## 问题描述

三臂机器人系统中，三个 RM65 机械臂安装在一个可绕 Z 轴旋转的基座（D1）上。需要：

1. 标定每个臂相对于基座的固定变换
2. 在基座旋转时，正确计算各臂末端在世界坐标系中的位姿
3. 支持通过 GUI 动态调整变换参数

## 坐标系定义

```
W  - 世界坐标系 (固定不动)
P  - 平台坐标系 (原点在 D1 转轴上，Z 为转轴方向，随 D1 转动)
Bi - 第 i 台机械臂基坐标系 (arm_a/arm_b/arm_s，随平台转动)
T  - 末端/工具坐标系
```

核心关系：
```
T_W_Bi = T_W_P × T_P_Bi
```

目标变换（世界系目标 → 臂基座系目标）：
```
T_Bi_T = inv(T_W_Bi) × T_W_T
```

## 需要标定的固定量

### 1. 平台在世界系下的零位姿 (D1=0 时)

```yaml
pose_W_P0: [x, y, z, rx, ry, rz]  # 位置(m) + 欧拉角(rad)
```

> 如果定义 W 与 P 零位重合，则 `pose_W_P0 = [0, 0, 0, 0, 0, 0]`

### 2. 各臂相对平台的安装位姿 (固定不变)

```yaml
pose_P_arm_a: [x, y, z, rx, ry, rz]  # arm_a 相对平台
pose_P_arm_b: [x, y, z, rx, ry, rz]  # arm_b 相对平台
pose_P_arm_s: [x, y, z, rx, ry, rz]  # arm_s 相对平台
```

## SDK Algo 接口

基于 `Robotic_Arm.rm_robot_interface.Algo`：

| 方法 | 功能 | 参数 | 返回 |
|------|------|------|------|
| `rm_algo_pos2matrix(pose)` | 位姿→4×4矩阵 | `[x,y,z,rx,ry,rz]` mm+rad | `(err, matrix_16)` |
| `rm_algo_matrix2pos(matrix, flag)` | 矩阵→位姿 | 16元素, flag=1欧拉角 | `(err, pose)` |
| `rm_algo_pose_move(pose, delta, mode)` | 位姿叠加 | pose, delta(deg), mode | `(err, new_pose)` |

> 注意：`rm_algo_pose_move` 的 delta 角度单位是 **度**，pose 本身是 **弧度**

## 计算流程

### 每个控制周期

```python
# 1. 读取 D1 当前角度 (度)
d1_angle_deg = get_d1_angle()

# 2. 计算平台当前位姿 T_W_P (绕自身Z轴旋转)
#    顺时针为负 (从+Z俯视)
delta = [0, 0, 0, 0, 0, -d1_angle_deg]
pose_W_P = algo.rm_algo_pose_move(pose_W_P0, delta, 1)  # mode=1 Tool系

# 3. 位姿转矩阵
T_W_P = algo.rm_algo_pos2matrix(pose_W_P)
```

### 下发目标给某臂 (以 arm_a 为例)

```python
# 4. 计算臂基座在世界系下的位姿
T_P_arm_a = algo.rm_algo_pos2matrix(pose_P_arm_a)
T_W_arm_a = T_W_P @ T_P_arm_a

# 5. 世界系目标 → 臂基座系目标
T_W_T = algo.rm_algo_pos2matrix(pose_W_T_target)
T_arm_a_T = inv(T_W_arm_a) @ T_W_T

# 6. 矩阵转位姿，下发给机械臂
pose_arm_a_T = algo.rm_algo_matrix2pos(T_arm_a_T, 1)
sdk.movel(*pose_arm_a_T)
```

## 配置文件

在 `triarm_config.yaml` 中添加：

```yaml
unified_arm_node:
  ros__parameters:
    # 平台零位姿 (W与P重合时可全为0)
    pose_W_P0: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    # 各臂相对平台的安装位姿 [x(m), y(m), z(m), rx(rad), ry(rad), rz(rad)]
    pose_P_arm_a: [0.0, 0.15, 0.0, 0.0, 0.0, 0.0]
    pose_P_arm_b: [0.0, -0.15, 0.0, 0.0, 0.0, 3.14159]
    pose_P_arm_s: [-0.15, 0.0, 0.2, 0.0, 0.0, 1.5708]
```

## GUI 标定界面

在 GUI 添加"坐标标定"标签页：

- 每臂 6 个参数滑块 (x, y, z, rx, ry, rz)
- "应用"按钮：热加载到 `unified_arm_node`
- "保存"按钮：写入配置文件

## 注意事项

1. **方向统一**：所有 `pose_P_Bi` 都是 P→Bi 方向，不要混用
2. **单位统一**：SDK 位置用 mm，代码接口用 m，注意转换
3. **顺时针符号**：从 +Z 俯视顺时针为负，建议用 ±5° 验证

## 世界坐标系 IK 解算

### 问题

当前 `ArmBridge._sim_move_to_pose()` 直接将输入位姿传给 SDK IK，没有考虑：
- D1 基座旋转
- 臂相对平台的安装偏移

### 完整流程

```python
def world_pose_to_joints(self, pose_W_T: list, d1_angle_deg: float) -> list:
    """世界坐标系位姿 → 臂关节角度

    Args:
        pose_W_T: 世界系目标 [x(m), y(m), z(m), rx, ry, rz]
        d1_angle_deg: D1 当前角度 (度)

    Returns:
        关节角度 (弧度), 失败返回 None
    """
    algo = self._sdk._algo

    # 1. 计算平台当前位姿 (绕Z轴旋转)
    delta = [0, 0, 0, 0, 0, -d1_angle_deg]  # 顺时针为负
    err, pose_W_P = algo.rm_algo_pose_move(self._pose_W_P0, delta, 1)

    # 2. 位姿转矩阵
    _, T_W_P = algo.rm_algo_pos2matrix(pose_W_P)
    _, T_P_Bi = algo.rm_algo_pos2matrix(self._pose_P_Bi)

    # 3. 臂基座在世界系下的变换
    T_W_Bi = matrix_multiply(T_W_P, T_P_Bi)

    # 4. 世界目标转臂坐标系
    pose_W_T_mm = [pose_W_T[0]*1000, pose_W_T[1]*1000, pose_W_T[2]*1000,
                   pose_W_T[3], pose_W_T[4], pose_W_T[5]]
    _, T_W_T = algo.rm_algo_pos2matrix(pose_W_T_mm)
    T_Bi_T = matrix_multiply(matrix_inverse(T_W_Bi), T_W_T)

    # 5. 矩阵转位姿
    _, pose_Bi_T = algo.rm_algo_matrix2pos(T_Bi_T, 1)

    # 6. IK 解算
    q_ref_deg = [math.degrees(q) for q in self._current_joints]
    err, joints_deg = algo.rm_algo_inverse_kinematics(
        rm_inverse_kinematics_params_t(q_ref_deg, pose_Bi_T, 1))

    if err != 0:
        return None
    return [math.radians(d) for d in joints_deg]
```

### 可达性检查

```python
def check_reachability(self, pose_W_T: list, d1_angle_deg: float) -> dict:
    """检查世界坐标系位姿是否可达

    Returns:
        {
            'reachable': bool,
            'joints': list or None,  # 关节角度 (弧度)
            'error': str or None
        }
    """
    joints = self.world_pose_to_joints(pose_W_T, d1_angle_deg)
    if joints is None:
        return {'reachable': False, 'joints': None, 'error': 'IK failed'}

    # 检查关节限位
    for i, j in enumerate(joints):
        if j < self._joint_limits[i][0] or j > self._joint_limits[i][1]:
            return {'reachable': False, 'joints': joints,
                    'error': f'Joint {i+1} out of limit'}

    return {'reachable': True, 'joints': joints, 'error': None}
```
