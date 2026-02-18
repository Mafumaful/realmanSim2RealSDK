# 三台机械臂 + 同一旋转平台（绕平台 Z 轴转）：统一下发目标位姿的更新方案（RealMan Algo 思路）

> 场景  
- 有一个旋转平台（转轴为平台坐标系 **P** 的 Z 轴），平台角度可控：θ  
- 平台上安装了 **3 台 6 轴机械臂**，每台机械臂的基座坐标系分别为 **B1 / B2 / B3**  
- 每台机械臂相对平台转轴（平台坐标系 P）都有固定安装偏差：`xyz + rpy`（也就是 **P → Bi** 固定变换）  
- 你希望把“目标末端位姿”发给某一台机械臂，并且平台转动后也能正确到达

---

## 0) 坐标系定义（建议这样定义最不容易乱）

- **W**：世界/工位坐标系（固定不动）
- **P**：平台坐标系（原点在转轴上，Z 为转轴方向；随平台转动）
- **Bi**：第 i 台机械臂的基坐标系（随平台转动，因为安装在平台上）
- **T**：末端/工具坐标系

核心关系：

\[
{}^{W}\!T_{B_i} = {}^{W}\!T_{P} \cdot {}^{P}\!T_{B_i}
\]

你要给第 i 台机械臂下发的目标（相对该机械臂基座）：

\[
{}^{B_i}\!T_{T} = ({}^{W}\!T_{B_i})^{-1} \cdot {}^{W}\!T_{T}
\]

---

## 1) 你需要准备/标定的“固定量”

### 1.1 平台在世界系下的零位姿（θ=0 时）
- `pose_W_P0 = [x, y, z, rx, ry, rz]`

> 如果你把 W 就定义成平台零位时的 P（即 θ=0 时 W 与 P 重合），那可以直接：
- `pose_W_P0 = [0,0,0,0,0,0]`

### 1.2 三台机械臂相对平台的安装位姿（固定不变）
对每台机械臂 i=1..3：
- `pose_P_Bi = [x, y, z, rx, ry, rz]`

这就是“平台坐标系 P 下看到机械臂基座 Bi”的安装偏差（xyzrpy）。

> 注意单位：  
- 文档里 `rm_algo_pose_move()` 的增量旋转单位是 **deg**，平移单位 **m**。([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
- pose 本身（rx/ry/rz）请按你“手册规定”的单位统一（很多示例看起来像 rad）。([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))

---

## 2) 每个控制周期：由平台角 θ 算出当前 `pose_W_P(θ)`

平台绕 **自身 Z 轴**旋转 θ（deg）。用 `rm_algo_pose_move()` 来叠加角度：

- `delta = [0, 0, 0, 0, 0, dRz_deg]`
- `pose_W_P = rm_algo_pose_move(pose_W_P0, delta, frameMode=1)`  （Tool/随体系叠加更符合“绕自身轴转”）  
  frameMode：0=Work，1=Tool。([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))

### 顺时针的符号（很关键）
在标准右手系里：绕 **+Z** 正向，从 +Z 俯视看到的是**逆时针**。  
如果你说的平台“顺时针”是指“从 +Z 方向俯视顺时针”，通常要取：
- `dRz_deg = -theta_deg`

> 强烈建议：用 +5° / -5° 做一次验证，确认你系统对“顺时针”的正负定义。

---

## 3) 合成每台机械臂当前基座在世界系下的位置：`pose_W_Bi(θ)`

Algo 提供 **位姿↔矩阵**转换：
- `rm_algo_pos2matrix(pose)`：位姿 → 4×4 齐次矩阵 ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
- `rm_algo_matrix2pos(matrix, flag=1)`：矩阵 → 位姿（flag=1 返回欧拉角）([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))

步骤：

1) `T_W_P = rm_algo_pos2matrix(pose_W_P)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
2) 对每台机械臂 i：  
   - `T_P_Bi = rm_algo_pos2matrix(pose_P_Bi)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
   - `T_W_Bi = T_W_P @ T_P_Bi`
   - （可选）`pose_W_Bi = rm_algo_matrix2pos(T_W_Bi, 1)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))

---

## 4) 下发某一台机械臂 i 的目标：把世界目标转成该机械臂基座下的目标

假设你希望用世界系给目标末端位姿：
- `pose_W_T_des`

步骤：

1) `T_W_T = rm_algo_pos2matrix(pose_W_T_des)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
2) `T_Bi_T = inv(T_W_Bi) @ T_W_T`
3) `pose_Bi_T_des = rm_algo_matrix2pos(T_Bi_T, 1)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
4) 将 `pose_Bi_T_des` 作为第 i 台机械臂的笛卡尔目标（MoveL/MoveJ 之类）下发

> 这样平台转到任何角度 θ，你都能给同一个“世界系目标”正确换算成对应机械臂的基座目标。

---

## 5) 更工程化的两种“目标表达方式”（看你任务更像哪种）

### A) 目标固定在世界 W（最常用）
比如外部相机标定、工位坐标固定、抓取点在地面不动：  
- 你给 `pose_W_T_des`
- 平台怎么转，都用上面的步骤换算给对应机械臂

### B) 目标固定在平台 P（平台上工装一起转）
比如工装、料盘、治具都在平台上，平台转动时目标也应“跟着转”：  
- 你直接给 `pose_P_T_des`（目标在平台坐标系里）
- 那么对第 i 台机械臂：
  \[
  {}^{B_i}\!T_{T} = ({}^{P}\!T_{B_i})^{-1} \cdot {}^{P}\!T_{T}
  \]
  也就是只用平台内的固定关系，不用每次带 θ（因为 P 本身就随平台转）

这一种会更简洁、数值也更稳定（特别是你平台上做重复工位）。

---

## 6) 最小“流程清单”（你可以直接照这个写工程逻辑）

**初始化（只做一次）**
1. 标定 `pose_W_P0`
2. 标定三台 `pose_P_B1 / pose_P_B2 / pose_P_B3`

**每个周期（或每次要下发目标时）**
1. 读平台角 `theta_deg`
2. `dRz_deg = -theta_deg`（按“俯视顺时针”为负号，先这样；实机用 5°验证）
3. `pose_W_P = rm_algo_pose_move(pose_W_P0, [0,0,0,0,0,dRz_deg], 1)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))
4. `T_W_P = rm_algo_pos2matrix(pose_W_P)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))
5. 对目标要发给的那台机械臂 i：
   - `T_P_Bi = rm_algo_pos2matrix(pose_P_Bi)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))
   - `T_W_Bi = T_W_P @ T_P_Bi`
   - `T_W_T = rm_algo_pos2matrix(pose_W_T_des)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))
   - `T_Bi_T = inv(T_W_Bi) @ T_W_T`
   - `pose_Bi_T_des = rm_algo_matrix2pos(T_Bi_T, 1)` ([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))
   - 下发 `pose_Bi_T_des`

---

## 7) 两个容易踩坑的点（多机械臂时更常见）

1) **每台机械臂的 pose_P_Bi 一定要统一定义方向**  
建议都用 “P 下看到 Bi（P→Bi）”，不要有的用 Bi→P 混着来，否则矩阵乘法顺序会全错。

2) **角度单位混用**  
`rm_algo_pose_move()` 的增量角是 deg；但 pose 的欧拉角单位要按手册统一（很多示例像 rad）。([develop.realman-robotics.com](https://develop.realman-robotics.com/robot4th/apipython/classes/algo/))  
建议你在工程里明确写注释并统一转换，避免“看起来能跑但慢慢漂”。

---

如果你愿意把下面 4 个量贴出来（每台一行也行），我可以直接帮你检查乘法方向和正负号，避免你上机试错：
- `pose_W_P0`
- `pose_P_B1`
- `pose_P_B2`
- `pose_P_B3`
以及你“俯视顺时针”的定义是从 +Z 方向看，还是从 -Z 方向看（这会影响 dRz 的正负）。
