import pybullet as p
import pybullet_data

# 连接 GUI 模式，打开可视化窗口
p.connect(p.GUI)

# 设置模型搜索路径（让 PyBullet 能找到自带模型）
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载地面和示例机器人（R2D2）
p.loadURDF("plane.urdf")
p.loadURDF("r2d2.urdf")

# 持续运行仿真（按 Ctrl+C 或关闭窗口退出）
while True:
    p.stepSimulation()
