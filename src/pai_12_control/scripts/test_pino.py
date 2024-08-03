
import pinocchio as pin

# URDF文件的路径
urdf_path = '/home/hpx/Hector_3/src/hector_description/xacro/pai_sit.urdf'

# 创建一个空的机器人模型
robot_model = pin.Model()

# 读取URDF文件并填充模型
pin.buildModelFromUrdf(urdf_path, robot_model)

# 现在robot_model包含了URDF文件中定义的机器人模型
