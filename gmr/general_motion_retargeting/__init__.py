from rich import print
from .params import IK_CONFIG_ROOT, ASSET_ROOT, ROBOT_XML_DICT, IK_CONFIG_DICT, ROBOT_BASE_DICT, VIEWER_CAM_DISTANCE_DICT

# 这里把“重型仿真依赖”和“纯工具脚本依赖”解耦开。
# 这样像 BVH 对比脚本、结构分析脚本这类只依赖轻量工具函数的入口，
# 在本地没有完整 Mujoco/Mink 环境时也能正常导入和运行。
_core_import_error = None
try:
    from .motion_retarget import GeneralMotionRetargeting
    from .robot_motion_viewer import RobotMotionViewer, draw_frame
    from .data_loader import load_robot_motion
except ImportError as import_error:
    _core_import_error = import_error

    # 这里保留一个统一的报错入口。
    # 如果调用方真正去实例化 retarget / viewer，而环境里又缺少重型依赖，
    # 那么会在调用时抛出清晰错误，而不是在导入纯工具模块时就提前失败。
    def _raise_core_import_error(*args, **kwargs):
        raise ImportError(
            "general_motion_retargeting core components require full runtime dependencies "
            "(for example mink / mujoco)."
        ) from _core_import_error

    GeneralMotionRetargeting = _raise_core_import_error
    RobotMotionViewer = _raise_core_import_error
    draw_frame = _raise_core_import_error
    load_robot_motion = _raise_core_import_error

try:
    from .kinematics_model import KinematicsModel
except ImportError:
    KinematicsModel = None  # optional; requires torch

try:
    from .neck_retarget import human_head_to_robot_neck
except ImportError:
    human_head_to_robot_neck = None

try:
    from .xrobot_utils import XRobotStreamer, XRobotRecorder
except ImportError:
    print("XRobotStreamer is not installed. Please install xrobotoolkit_sdk to use this feature.")
    XRobotStreamer = None
    XRobotRecorder = None
