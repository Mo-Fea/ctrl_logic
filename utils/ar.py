import cv2
import cv2.aruco as aruco
import pyrealsense2 as rs
import numpy as np

def configure_color_sensor(profile):
    # 屏幕容易在自动曝光下发白，手动降低彩色相机曝光；原本没有设置，使用相机默认自动曝光。
    color_sensor = profile.get_device().first_color_sensor()
    try:
        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 0)
        if color_sensor.supports(rs.option.exposure):
            color_sensor.set_option(rs.option.exposure, 80)
        if color_sensor.supports(rs.option.gain):
            color_sensor.set_option(rs.option.gain, 16)
    except RuntimeError as err:
        print(f"设置曝光失败，继续使用相机默认曝光: {err}")


def restore_color_sensor(profile):
    # 程序退出前恢复自动曝光，避免手动曝光参数影响之后的相机画面；原本退出时没有恢复。
    try:
        color_sensor = profile.get_device().first_color_sensor()
        if color_sensor.supports(rs.option.enable_auto_exposure):
            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
            print("已恢复彩色相机自动曝光")
    except RuntimeError as err:
        print(f"恢复自动曝光失败: {err}")


def start_camera():
    # 优先尝试 1280x720，原本参数是 640x480；如果拿不到帧，自动回退到原本参数。
    color_profiles = [
        (1280, 720),
        (640, 480),
    ]

    for color_width, color_height in color_profiles:
        current_pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, color_width, color_height, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        try:
            profile = current_pipeline.start(config)
            configure_color_sensor(profile)

            # 启动后先等几帧验证；原本直接进入主循环，1280x720 不稳定时会在这里超时。
            for _ in range(5):
                current_pipeline.wait_for_frames()

            print(f"相机启动成功: color={color_width}x{color_height}, depth=640x480")
            return current_pipeline, profile
        except RuntimeError as err:
            print(f"相机配置 color={color_width}x{color_height} 不可用或拿不到帧，尝试回退: {err}")
            try:
                current_pipeline.stop()
            except RuntimeError:
                pass

    raise RuntimeError("相机启动失败：高分辨率和原始 640x480 配置都拿不到帧")


# 初始化相机
pipeline, profile = start_camera()
align = rs.align(rs.stream.color)

# 1. 设置 ArUco 字典（使用最常用的 4x4 网格）
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
# 放宽自适应阈值窗口，原本使用 OpenCV 默认值 min=3, max=23, step=10。
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 53
parameters.adaptiveThreshWinSizeStep = 4
# 允许更小的标记进入检测，原本默认值通常是 0.03。
parameters.minMarkerPerimeterRate = 0.01
# 使用亚像素角点优化，原本默认通常是不做角点优化。
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementWinSize = 5
parameters.cornerRefinementMaxIterations = 30
parameters.cornerRefinementMinAccuracy = 0.01
detector = aruco.ArucoDetector(aruco_dict, parameters)

print("开始 ArUco 检测...")


def get_marker_center(marker_corners):
    pts = marker_corners.reshape((4, 2))
    center_x = int(np.mean(pts[:, 0]))
    center_y = int(np.mean(pts[:, 1]))
    return center_x, center_y


def get_depth_at_center(depth_frame, center_x, center_y, radius=2):
    depths = []
    width = depth_frame.get_width()
    height = depth_frame.get_height()

    for y in range(max(0, center_y - radius), min(height, center_y + radius + 1)):
        for x in range(max(0, center_x - radius), min(width, center_x + radius + 1)):
            depth = depth_frame.get_distance(x, y)
            if depth > 0:
                depths.append(depth)

    if not depths:
        return 0
    return float(np.median(depths))


def detect_markers_with_fallback(gray):
    # 先用原图检测，原本只有这一种检测方式。
    candidates = [gray]

    # 亮屏幕会让黑白边界发灰，CLAHE 增强局部对比度作为备用检测图。
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    candidates.append(clahe.apply(gray))

    # 再加一个轻微压暗版本，原本没有亮度压暗处理。
    darker = cv2.convertScaleAbs(gray, alpha=0.75, beta=0)
    candidates.append(darker)

    best_corners, best_ids, best_rejected = [], None, []
    best_count = 0

    for candidate in candidates:
        corners, ids, rejected = detector.detectMarkers(candidate)
        count = 0 if ids is None else len(ids)
        if count >= 2:
            return corners, ids, rejected
        if count > best_count:
            best_corners, best_ids, best_rejected = corners, ids, rejected
            best_count = count

    return best_corners, best_ids, best_rejected

try:
    while True:
        try:
            frames = pipeline.wait_for_frames()
        except RuntimeError as err:
            print(f"等待相机帧超时，继续等待: {err}")
            continue

        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not color_frame or not depth_frame:
            continue

        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        img = np.asanyarray(color_frame.get_data())

        # 2. 灰度转换（ArUco 识别需要灰度图）
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 3. 检测标记
        corners, ids, rejected = detect_markers_with_fallback(gray)

        # 4. 如果识别到了标记
        if ids is not None:
            # 在图上画出识别框
            aruco.drawDetectedMarkers(img, corners, ids)

            markers = []
            for i in range(len(ids)):
                center_x, center_y = get_marker_center(corners[i][0])
                depth = get_depth_at_center(depth_frame, center_x, center_y)

                if depth <= 0:
                    continue

                x, y, z = rs.rs2_deproject_pixel_to_point(
                    color_intrinsics,
                    [center_x, center_y],
                    depth
                )
                markers.append({
                    "id": int(ids[i][0]),
                    "center": (center_x, center_y),
                    "coord": (x, y, z),
                })

                cv2.circle(img, (center_x, center_y), 4, (0, 255, 0), -1)
                cv2.putText(
                    img,
                    f"ID {ids[i][0]}: ({x:.3f}, {y:.3f}, {z:.3f})m",
                    (center_x + 8, center_y - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.45,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA
                )

            markers.sort(key=lambda marker: marker["center"][0])
            if len(markers) >= 2:
                left = markers[0]
                right = markers[-1]
                left_coord = np.array(left["coord"])
                right_coord = np.array(right["coord"])
                vector = right_coord - left_coord
                length = np.linalg.norm(vector)
                unit_vector = vector / length if length > 0 else np.zeros(3)

                print(
                    f"左码 ID {left['id']} 坐标: "
                    f"x={left_coord[0]:.3f}m, y={left_coord[1]:.3f}m, z={left_coord[2]:.3f}m | "
                    f"右码 ID {right['id']} 坐标: "
                    f"x={right_coord[0]:.3f}m, y={right_coord[1]:.3f}m, z={right_coord[2]:.3f}m | "
                    f"向量 V=右码-左码: "
                    f"vx={vector[0]:.3f}m, vy={vector[1]:.3f}m, vz={vector[2]:.3f}m | "
                    f"向量长度 length={length:.3f}m | "
                    f"单位向量 unit_V=({unit_vector[0]:.3f}, {unit_vector[1]:.3f}, {unit_vector[2]:.3f})"
                )

        cv2.imshow('ArUco Detection', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    restore_color_sensor(profile)
    pipeline.stop()
    cv2.destroyAllWindows()
