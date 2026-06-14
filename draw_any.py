import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt

# -------------------- 辅助函数 --------------------
def move_to_target(robot_id, target_pos, ee_idx, rest_poses, steps=100):
    """平滑移动末端到目标位置（直线插补）"""
    for _ in range(steps):
        joint_angles = p.calculateInverseKinematics(
            robot_id, ee_idx, target_pos, restPoses=rest_poses)
        # 一次性设置所有手臂关节，保证同步
        p.setJointMotorControlArray(
            robot_id, range(7), p.POSITION_CONTROL,
            targetPositions=joint_angles[:7])
        p.stepSimulation()
        time.sleep(1.0 / 240.0)

def draw_flower_path(robot_id, path, ee_idx, rest_poses,
                     trajectory_log, joint_log):
    """沿着给定路径点画花"""
    for pos in path:
        joint_angles = p.calculateInverseKinematics(
            robot_id, ee_idx, pos, restPoses=rest_poses)
        p.setJointMotorControlArray(
            robot_id, range(7), p.POSITION_CONTROL,
            targetPositions=joint_angles[:7])
        p.stepSimulation()

        link_state = p.getLinkState(robot_id, ee_idx)
        actual_pos = link_state[0]
        trajectory_log.append(actual_pos)
        joint_log.append(list(joint_angles[:7]))
        time.sleep(1.0 / 240.0)

# -------------------- 1. 生成花形路径 --------------------
petal_num = 5
base_radius = 0.04
amplitude = 0.025
num_points = 500
draw_height = 0.15          # 在 z=0.15 的“空中”画花
center_x, center_y = 0.35, 0.0   # 基座前方 0.35 米

theta = np.linspace(0, 2*np.pi, num_points)
r = base_radius + amplitude * np.sin(petal_num * theta)
x = center_x + r * np.cos(theta)
y = center_y + r * np.sin(theta)
z = np.full_like(x, draw_height)
flower_path = np.column_stack((x, y, z))

# -------------------- 2. 初始化仿真 --------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1.0 / 240.0)

p.loadURDF("plane.urdf")
# 基座放在地面上
robot_id = p.loadURDF("franka_panda/panda.urdf", [0, 0, 0], useFixedBase=True)
ee_idx = 11

rest_poses = [0, -0.215, 0, -2.57, 0, 2.356, 2.356, 0.0, 0.0]
for i in range(9):
    p.resetJointState(robot_id, i, rest_poses[i])
for _ in range(100):
    p.stepSimulation()

trajectory_log = []
joint_log = []

# -------------------- 3. 执行绘画 --------------------
print("移动到绘画起始点...")
# 先平滑移动到花形路径的第一个点
move_to_target(robot_id, flower_path[0], ee_idx, rest_poses[:7], steps=120)

print("开始画花...")
draw_flower_path(robot_id, flower_path, ee_idx, rest_poses[:7],
                 trajectory_log, joint_log)

# 画完回到初始姿态（用 move_to_target 平滑回起始关节位）
print("回到初始姿态...")
# 计算初始姿态对应的末端位置
init_joint_pos = rest_poses[:7]
# 先得到初始末端位置（用于移动目标）
for i in range(7):
    p.resetJointState(robot_id, i, rest_poses[i])
for _ in range(100):
    p.stepSimulation()
init_state = p.getLinkState(robot_id, ee_idx)
init_ee_pos = init_state[0]
# 移动回该位置
move_to_target(robot_id, init_ee_pos, ee_idx, rest_poses[:7], steps=120)

print("绘画完成！")

# -------------------- 4. 可视化 --------------------
trajectory_arr = np.array(trajectory_log)
joint_arr = np.array(joint_log)

fig = plt.figure(figsize=(12, 5))
ax1 = fig.add_subplot(1, 2, 1, projection='3d')
ax1.plot(trajectory_arr[:, 0], trajectory_arr[:, 1], trajectory_arr[:, 2],
         'm-', linewidth=2, label='Actual path')
ax1.plot(flower_path[:, 0], flower_path[:, 1], flower_path[:, 2],
         'k--', linewidth=1, label='Desired flower')
ax1.set_xlabel('X'); ax1.set_ylabel('Y'); ax1.set_zlabel('Z')
ax1.set_title('Flower Drawing Path')
ax1.legend()

ax2 = fig.add_subplot(1, 2, 2)
for j in range(7):
    ax2.plot(joint_arr[:, j], label=f'Joint {j+1}')
ax2.set_xlabel('Step')
ax2.set_ylabel('Joint angle (rad)')
ax2.set_title('Joint Angles')
ax2.legend()
plt.tight_layout()
plt.show()

p.disconnect()
