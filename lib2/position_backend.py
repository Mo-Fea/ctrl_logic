LIDAR_TYPE_ODIN = 1
LIDAR_TYPE_MID360 = 2
FIELD_TYPE_RED = 1
FIELD_TYPE_BLUE = 2

# 位置后端选择:
# 1 使用 position_odin，走 /tf 计算雷达/机器人/weapon 位姿。
# 2 使用 position_mid360，走 /lio/odom 与 /lio/robo/odom 获取位姿。
LIDAR_TYPE = LIDAR_TYPE_ODIN
# 场地半场选择:
# 1 使用红色/右半场坐标。
# 2 使用蓝色/左半场坐标。
FIELD_TYPE = FIELD_TYPE_RED


def set_lidar_type(lidar_type):
    """
    设置当前位姿后端类型。

    lidar_type:
      1: odin
      2: mid360
    """
    lidar_type = int(lidar_type)
    if lidar_type not in (LIDAR_TYPE_ODIN, LIDAR_TYPE_MID360):
        raise ValueError(f"LIDAR_TYPE must be 1 or 2, got {lidar_type}")

    global LIDAR_TYPE
    LIDAR_TYPE = lidar_type
    return LIDAR_TYPE


def get_lidar_type():
    return int(LIDAR_TYPE)


def is_mid360():
    return get_lidar_type() == LIDAR_TYPE_MID360


def set_field_type(field_type):
    """
    设置当前场地半场。

    field_type:
      1: red/right field
      2: blue/left field
    """
    field_type = int(field_type)
    if field_type not in (FIELD_TYPE_RED, FIELD_TYPE_BLUE):
        raise ValueError(f"FIELD_TYPE must be 1 or 2, got {field_type}")

    global FIELD_TYPE
    FIELD_TYPE = field_type
    return FIELD_TYPE


def get_field_type():
    return int(FIELD_TYPE)


def is_red_field():
    return get_field_type() == FIELD_TYPE_RED


def is_blue_field():
    return get_field_type() == FIELD_TYPE_BLUE


def get_position_backend(lidar_type=None):
    """
    根据雷达类型返回位姿后端模块。

    返回模块需提供与 position_odin 对齐的接口，例如:
      TfCacheNode, spin_tf_cache, get_lidar_pose_in_map_synced,
      cal_robot_position, get_weapon_pose_in_map_synced, cal_target_yaw_deg。
    """
    if lidar_type is None:
        lidar_type = LIDAR_TYPE

    lidar_type = int(lidar_type)
    if lidar_type == LIDAR_TYPE_ODIN:
        from lib2 import position_odin

        return position_odin

    if lidar_type == LIDAR_TYPE_MID360:
        from lib2 import position_mid360

        return position_mid360

    raise ValueError(f"LIDAR_TYPE must be 1 or 2, got {lidar_type}")
