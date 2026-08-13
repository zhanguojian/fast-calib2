#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功能：
1) 自动检测 rosbag2 中雷达点云类型：
   - sensor_msgs/msg/PointCloud2  (如 /hesai/pandar, /livox/lidar)
   - livox_ros_driver / livox_ros_driver2 / fast_calib 的 CustomMsg
2) 按各自的解析方式把点云导出成一个带 intensity 的 PCD 文件 (x y z intensity, ASCII)
3) 使用 Open3D 对该 PCD 进行交互选点（至少 4 个点），并根据 4 个点计算包围范围，
   保存为同名 .txt 文件。

依赖：
    source /opt/ros/humble/setup.bash
    python3 -m pip install open3d   # 仅交互选点需要

用法示例：
    python3 scripts/distance_filter_tool.py
    python3 scripts/distance_filter_tool.py calib_data/bag/front output
    python3 scripts/distance_filter_tool.py calib_data/bag/front output /livox/lidar
"""

import os
import sys
from pathlib import Path

import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2 as pc2
except ImportError:
    print(
        "[ERROR] 未找到 ROS2 Python 依赖。请先执行:\n"
        "  source /opt/ros/humble/setup.bash",
        file=sys.stderr,
    )
    sys.exit(1)


# ===================== rosbag2 读取辅助 =====================

def is_bag_path(path):
    """判断是否为 rosbag2 目录或 ROS1 .bag 文件。"""
    p = Path(path)
    if p.is_file() and p.suffix == ".bag":
        return True
    if p.is_dir() and (p / "metadata.yaml").is_file():
        return True
    return False


def is_rosbag2_dir(path):
    p = Path(path)
    return p.is_dir() and (p / "metadata.yaml").is_file()


def normalize_topic(topic):
    """去掉末尾斜杠后比较 topic。"""
    if not topic:
        return topic
    while len(topic) > 1 and topic.endswith("/"):
        topic = topic[:-1]
    return topic


def topic_matches(bag_topic, wanted):
    a = normalize_topic(bag_topic)
    b = normalize_topic(wanted)
    return a == b or a == ("/" + b) or ("/" + a) == b


def is_pointcloud2_type(msgtype):
    return "PointCloud2" in msgtype


def is_livox_custom_type(msgtype):
    return "CustomMsg" in msgtype


def detect_storage_id(bag_dir):
    """从 metadata.yaml 读取存储格式，默认 sqlite3。"""
    meta_path = Path(bag_dir) / "metadata.yaml"
    try:
        data = yaml.safe_load(meta_path.read_text(encoding="utf-8")) or {}
        info = data.get("rosbag2_bagfile_information", data)
        storage_id = info.get("storage_identifier") or "sqlite3"
        return storage_id
    except Exception:
        return "sqlite3"


def open_bag_reader(bag_file):
    """打开 rosbag2 目录。"""
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(
        uri=str(bag_file),
        storage_id=detect_storage_id(bag_file),
    )
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader.open(storage_options, converter_options)
    return reader


def get_topic_type_map(reader):
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


_msg_class_cache = {}


def deserialize_bag_msg(msgtype, data):
    if msgtype not in _msg_class_cache:
        _msg_class_cache[msgtype] = get_message(msgtype)
    return deserialize_message(data, _msg_class_cache[msgtype])


def iter_bag_messages(reader, topic_name=None):
    """
    遍历 bag 消息，返回 (topic, msgtype, msg)。
    topic_name 为空时遍历全部 topic。
    """
    type_map = get_topic_type_map(reader)
    while reader.has_next():
        topic, data, _timestamp = reader.read_next()
        if topic_name and not topic_matches(topic, topic_name):
            continue
        msgtype = type_map.get(topic, "")
        try:
            msg = deserialize_bag_msg(msgtype, data)
        except Exception as e:
            print(f"[ERROR] 反序列化失败 {topic} ({msgtype}): {e}", file=sys.stderr)
            continue
        yield topic, msgtype, msg


def find_first_topic_by_type(type_map, type_checker):
    """在 bag topic 表中查找第一个匹配类型的 topic。"""
    for topic, msgtype in type_map.items():
        if type_checker(msgtype):
            return topic
    return None


def resolve_topic(type_map, topic_name, type_checker, default_candidates):
    """
    解析实际使用的 lidar topic：
    1) 若指定 topic 存在且类型匹配，直接使用
    2) 否则按默认候选列表匹配
    3) 再否则使用 bag 中第一个匹配类型的 topic
    """
    if topic_name:
        for topic, msgtype in type_map.items():
            if topic_matches(topic, topic_name) and type_checker(msgtype):
                return topic
        print(f"[Bag] 未找到指定 topic '{topic_name}'，尝试自动检测...", file=sys.stderr)

    for candidate in default_candidates:
        for topic, msgtype in type_map.items():
            if topic_matches(topic, candidate) and type_checker(msgtype):
                print(f"[Bag] 自动使用 topic: {topic}")
                return topic

    fallback = find_first_topic_by_type(type_map, type_checker)
    if fallback:
        print(f"[Bag] 自动使用 topic: {fallback}")
    return fallback


# ===================== 通用：保存 PCD =====================

def save_pcd_with_intensity(points, intensities, output_path):
    """
    保存点云为带 intensity 字段的 PCD 文件 (ASCII 格式)
    points: list/ndarray of [x, y, z]
    intensities: list/ndarray of intensity
    """
    N = len(points)
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z intensity
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {N}
HEIGHT 1
POINTS {N}
DATA ascii
"""
    with open(output_path, "w") as f:
        f.write(header)
        for (x, y, z), inten in zip(points, intensities):
            f.write(f"{x} {y} {z} {inten}\n")
    print(f"[PCD] 保存带 intensity 字段的点云到: {output_path}")


# ===================== 情况 1：PointCloud2 =====================

def find_intensity_field(msg):
    """在 PointCloud2 的 fields 中自动检测强度字段名称"""
    candidates = ["intensity", "reflectivity", "i", "ref"]
    for field in msg.fields:
        if field.name.lower() in candidates:
            return field.name
    return None


def convert_pointcloud2_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/hesai/pandar",                        # 如有不同，可改成 topic 名称
    pcd_name="sensor_PointCloud2_inten_ascii.pcd"
):
    """
    将 rosbag2 中 PointCloud2 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    print(f"[Bag] 打开 rosbag2: {bag_file}")
    reader = open_bag_reader(bag_file)
    actual_topic = resolve_topic(
        get_topic_type_map(reader),
        topic_name,
        is_pointcloud2_type,
        default_candidates=["/hesai/pandar", "/livox/lidar", "/lidar_points"],
    )
    if not actual_topic:
        print("[ERROR] 未找到 PointCloud2 topic！", file=sys.stderr)
        return None

    # 1) 先检测强度字段
    intensity_field = None
    for _topic, msgtype, msg in iter_bag_messages(reader, actual_topic):
        if is_pointcloud2_type(msgtype):
            intensity_field = find_intensity_field(msg)
            if intensity_field:
                print(f"[Bag] 检测到 intensity 字段: {intensity_field}")
            break

    if not intensity_field:
        print("[ERROR] 未找到强度字段! 退出 PointCloud2 转换。", file=sys.stderr)
        return None

    # 2) 重新打开 bag，读取指定 topic 的所有点云
    reader = open_bag_reader(bag_file)
    all_points = []
    all_intensities = []
    print(f"[Bag] 开始从 topic '{actual_topic}' 读取 PointCloud2 点云...")
    for _topic, msgtype, msg in iter_bag_messages(reader, actual_topic):
        if not is_pointcloud2_type(msgtype):
            continue
        try:
            field_names = ["x", "y", "z", intensity_field]
            for point in pc2.read_points(msg, field_names=field_names, skip_nans=True):
                all_points.append([point[0], point[1], point[2]])
                all_intensities.append(point[3])  # 强度是第四个字段
        except Exception as e:
            print(f"[ERROR] 读取错误: {str(e)}", file=sys.stderr)
            continue

    if not all_points:
        print("[ERROR] 未找到 PointCloud2 点云数据！", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    save_pcd_with_intensity(all_points, all_intensities, output_path)
    return output_path


# ===================== 情况 2：Livox CustomMsg =====================

def parse_livox_custom_msg(msg):
    """
    从 livox_ros_driver/CustomMsg 中解析 x, y, z, reflectivity
    假设 msg.points 是 CustomPoint 对象列表，字段为 x, y, z, reflectivity
    """
    points = []
    intensities = []

    for pt in msg.points:
        points.append([pt.x, pt.y, pt.z])
        intensities.append(pt.reflectivity)

    return points, intensities


def convert_livox_custom_bag_to_pcd(
    bag_file,
    output_dir,
    topic_name="/livox/lidar",                     # 如有不同，可改成 topic 名称
    pcd_name="livox_CustomMsg_inten_ascii.pcd"
):
    """
    将 rosbag2 中 Livox CustomMsg 类型的点云合并导出为一个 PCD 文件。
    保持原始雷达坐标，不做坐标变换。
    """
    print(f"[Bag] 打开 rosbag2: {bag_file}")
    reader = open_bag_reader(bag_file)
    actual_topic = resolve_topic(
        get_topic_type_map(reader),
        topic_name,
        is_livox_custom_type,
        default_candidates=["/livox/lidar"],
    )
    if not actual_topic:
        print("[ERROR] 未找到 Livox CustomMsg topic!", file=sys.stderr)
        return None

    all_points = []
    all_intensities = []
    print(f"[Bag] 开始从 topic '{actual_topic}' 读取 CustomMsg 点云...")
    for _topic, msgtype, msg in iter_bag_messages(reader, actual_topic):
        if not is_livox_custom_type(msgtype):
            continue
        pts, intens = parse_livox_custom_msg(msg)
        all_points.extend(pts)
        all_intensities.extend(intens)

    if not all_points:
        print("[ERROR] 未找到 Livox CustomMsg 点云数据!", file=sys.stderr)
        return None

    output_path = os.path.join(output_dir, pcd_name)
    intensities = np.array(all_intensities, dtype=np.float32)
    save_pcd_with_intensity(all_points, intensities, output_path)
    return output_path


# ===================== 自动检测：这个 bag 用哪种方式 =====================

def detect_lidar_msg_type(bag_file):
    """
    在 bag 里扫一圈，检测是否有 PointCloud2 或 Livox CustomMsg。
    返回：
        "PointCloud2", "CustomMsg", 或 None
    如果两种都有，默认优先 PointCloud2,并打印提示。
    """
    has_pc2 = False
    has_livox = False

    print(f"[Detect] 扫描 bag: {bag_file}")
    reader = open_bag_reader(bag_file)
    for topic, msgtype in get_topic_type_map(reader).items():
        if is_pointcloud2_type(msgtype):
            has_pc2 = True
            print(f"[Detect] PointCloud2 topic: {topic} ({msgtype})")
        elif is_livox_custom_type(msgtype):
            has_livox = True
            print(f"[Detect] CustomMsg topic: {topic} ({msgtype})")
        if has_pc2 and has_livox:
            break

    if has_pc2 and has_livox:
        print("[Detect] 同时检测到 PointCloud2 和 Livox CustomMsg, 默认使用 PointCloud2。")
        return "PointCloud2"
    elif has_pc2:
        print("[Detect] 检测到 PointCloud2 点云。")
        return "PointCloud2"
    elif has_livox:
        print("[Detect] 检测到 Livox CustomMsg 点云。")
        return "CustomMsg"
    else:
        print("[Detect] 未检测到 PointCloud2 或 Livox CustomMsg 点云。")
        return None


# ===================== Open3D 交互选点 & 保存范围 =====================

# def select_and_save_points(pcd_folder, target_pcd_name):
#     """
#     在给定目录中读取指定 PCD 文件，用 Open3D 交互式选点并保存范围。
#     """
#     try:
#         import open3d as o3d
#     except ImportError:
#         print(
#             "[ERROR] 未安装 open3d,无法交互选点。PCD 已生成，可安装后再选点:\n"
#             "  python3 -m pip install open3d",
#             file=sys.stderr,
#         )
#         return

#     pcd_path = os.path.join(pcd_folder, target_pcd_name)
#     if not os.path.isfile(pcd_path):
#         print(f"[ERROR] 指定的 PCD 文件不存在: {pcd_path}", file=sys.stderr)
#         return

#     # 读取点云
#     pcd = o3d.io.read_point_cloud(pcd_path)
#     if not pcd.has_points():
#         print(f"[ERROR] {target_pcd_name} 中没有点云数据，已跳过", file=sys.stderr)
#         return

#     print(f"\n正在处理: {target_pcd_name}")
#     print("请在可视化窗口中按住 Shift 用鼠标左键选择点(至少4个)，然后按 Q 键关闭窗口")

#     # 创建可视化窗口并添加点云
#     vis = o3d.visualization.VisualizerWithEditing()
#     vis.create_window(window_name=f"选择点 - {target_pcd_name}")
#     vis.add_geometry(pcd)

#     # 等待用户交互（Shift+左键选点, Q 退出）
#     vis.run()
#     vis.destroy_window()

#     # 获取用户选择的点的索引
#     selected_indices = vis.get_picked_points()

#     if not selected_indices:
#         print(f"[ERROR] 未选择任何点，{target_pcd_name} 没有保存文件", file=sys.stderr)
#         return

#     if len(selected_indices) < 4:
#         print(f"[ERROR] 只选中了 {len(selected_indices)} 个点，少于 4 个，跳过该文件", file=sys.stderr)
#         return

#     # 只取前 4 个点
#     selected_indices = selected_indices[:4]

#     all_points = np.asarray(pcd.points)
#     selected_points = all_points[selected_indices, :]  # 形状 (4, 3)

#     # 计算四个点在各轴上的最小值和最大值
#     mins = selected_points.min(axis=0)  # [x_min_raw, y_min_raw, z_min_raw]
#     maxs = selected_points.max(axis=0)  # [x_max_raw, y_max_raw, z_max_raw]

#     # 按你的定义扩展 0.2m
#     x_min = mins[0] - 0.2
#     x_max = maxs[0] + 0.2
#     y_min = mins[1] - 0.2
#     y_max = maxs[1] + 0.2
#     z_min = mins[2] - 0.2
#     z_max = maxs[2] + 0.2

#     # 生成保存文件名 (与 PCD 文件同名，改为 txt)
#     base_name = os.path.splitext(target_pcd_name)[0]
#     save_file = os.path.join(pcd_folder, f"{base_name}.txt")

#     with open(save_file, "w") as f:
#         f.write("# 4 selected points (x y z)\n")
#         for p in selected_points:
#             f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

#         f.write("# range values in order:\n")
#         f.write(f"x_min: {x_min:.1f}\n")
#         f.write(f"x_max: {x_max:.1f}\n")
#         f.write(f"y_min: {y_min:.1f}\n")
#         f.write(f"y_max: {y_max:.1f}\n")
#         f.write(f"z_min: {z_min:.1f}\n")
#         f.write(f"z_max: {z_max:.1f}\n")

#     print(f"[Save] 已保存选点与范围到: {save_file}")
#     print("点云处理完成。")

def select_and_save_points(pcd_folder, target_pcd_name):
    """
    读取 ASCII XYZI PCD，
    根据 intensity 生成灰度颜色，
    使用 Open3D 交互选择标定板上的点。
    """

    try:
        import open3d as o3d
    except ImportError:
        print(
            "[ERROR] 未安装 open3d",
            file=sys.stderr,
        )
        return

    pcd_path = os.path.join(
        pcd_folder,
        target_pcd_name
    )

    if not os.path.isfile(pcd_path):
        print(
            f"[ERROR] PCD 不存在: {pcd_path}",
            file=sys.stderr
        )
        return

    # ==========================================
    # 1. 自己读取 ASCII PCD 的 XYZI
    # ==========================================

    points = []
    intensities = []

    data_started = False

    with open(
        pcd_path,
        "r",
        encoding="utf-8"
    ) as f:

        for line in f:

            line = line.strip()

            if not data_started:

                if line.lower().startswith(
                    "data ascii"
                ):
                    data_started = True

                continue

            if not line:
                continue

            values = line.split()

            if len(values) < 4:
                continue

            try:
                x = float(values[0])
                y = float(values[1])
                z = float(values[2])
                intensity = float(values[3])
            except ValueError:
                continue

            if not (
                np.isfinite(x) and
                np.isfinite(y) and
                np.isfinite(z) and
                np.isfinite(intensity)
            ):
                continue

            points.append([
                x,
                y,
                z
            ])

            intensities.append(
                intensity
            )

    if not points:
        print(
            "[ERROR] PCD 中没有有效点",
            file=sys.stderr
        )
        return

    points = np.asarray(
        points,
        dtype=np.float64
    )

    intensities = np.asarray(
        intensities,
        dtype=np.float64
    )

    # ==========================================
# 去掉离雷达太远的点，避免 Open3D 视野被撑大
# ==========================================

    distance = np.linalg.norm(points, axis=1)

    mask = (
        (distance > 0.3) &
        (distance < 10.0)
    )

    points = points[mask]
    intensities = intensities[mask]

    print(
        f"[Open3D] 距离过滤后点数: {len(points)}"
    )

    print(
        f"[Open3D] 点数: {len(points)}"
    )

    print(
        "[Open3D] XYZ范围:"
    )

    print(
        f"  X: {points[:, 0].min():.3f}"
        f" ~ {points[:, 0].max():.3f}"
    )

    print(
        f"  Y: {points[:, 1].min():.3f}"
        f" ~ {points[:, 1].max():.3f}"
    )

    print(
        f"  Z: {points[:, 2].min():.3f}"
        f" ~ {points[:, 2].max():.3f}"
    )

    print(
        f"[Open3D] intensity:"
        f" {intensities.min():.1f}"
        f" ~ {intensities.max():.1f}"
    )

    # ==========================================
    # 2. intensity 鲁棒归一化
    # ==========================================

    i_min = np.percentile(
        intensities,
        5
    )

    i_max = np.percentile(
        intensities,
        98
    )

    if i_max <= i_min:
        i_min = intensities.min()
        i_max = intensities.max()

    normalized = (
        intensities - i_min
    ) / max(
        i_max - i_min,
        1e-6
    )

    normalized = np.clip(
        normalized,
        0.0,
        1.0
    )

    # ==========================================
    # 3. 颜色映射
    #
    # 普通反射点 -> 暗
    # 高反射点   -> 亮
    # ==========================================

    colors = np.zeros(
        (len(points), 3),
        dtype=np.float64
    )

    colors[:, 0] = normalized
    colors[:, 1] = normalized
    colors[:, 2] = normalized

    # ==========================================
    # 4. 创建 Open3D PointCloud
    # ==========================================

    pcd = o3d.geometry.PointCloud()

    pcd.points = (
        o3d.utility.Vector3dVector(
            points
        )
    )

    pcd.colors = (
        o3d.utility.Vector3dVector(
            colors
        )
    )

    print()
    print(
        "请找到标定板:"
    )
    print(
        "Shift + 左键 : 选择点"
    )
    print(
        "至少选 4 个标定板边缘点"
    )
    print(
        "Q : 完成"
    )

    # ==========================================
    # 5. Open3D 显示
    # ==========================================

    vis = (
        o3d.visualization.
        VisualizerWithEditing()
    )

    vis.create_window(
        window_name=(
            f"选择标定板 - "
            f"{target_pcd_name}"
        ),
        width=1280,
        height=800
    )

    vis.add_geometry(pcd)

    render_option = (
        vis.get_render_option()
    )

    # 点大一点
    render_option.point_size = 3.0

    # 黑色背景
    render_option.background_color = (
        np.asarray([
            0.03,
            0.03,
            0.03
        ])
    )

    vis.run()

    selected_indices = (
        vis.get_picked_points()
    )

    vis.destroy_window()

    # ==========================================
    # 6. 检查选点
    # ==========================================

    if not selected_indices:

        print(
            "[ERROR] 没有选择任何点",
            file=sys.stderr
        )

        return

    if len(selected_indices) < 4:

        print(
            f"[ERROR] 只选择了 "
            f"{len(selected_indices)} 个点，"
            f"至少需要 4 个",
            file=sys.stderr
        )

        return

    # 只取前四个
    selected_indices = (
        selected_indices[:4]
    )

    selected_points = (
        points[selected_indices]
    )

    print()
    print(
        "选中的4个点:"
    )

    for i, p in enumerate(
        selected_points
    ):

        print(
            f"  P{i}: "
            f"{p[0]:.3f}, "
            f"{p[1]:.3f}, "
            f"{p[2]:.3f}"
        )

    # ==========================================
    # 7. Bounding Box
    # ==========================================

    mins = selected_points.min(
        axis=0
    )

    maxs = selected_points.max(
        axis=0
    )

    margin = 0.2

    x_min = mins[0] - margin
    x_max = maxs[0] + margin

    y_min = mins[1] - margin
    y_max = maxs[1] + margin

    z_min = mins[2] - margin
    z_max = maxs[2] + margin

    print()
    print(
        "推荐 FAST-Calib2 ROI:"
    )

    print(
        f"x_min: {x_min:.3f}"
    )
    print(
        f"x_max: {x_max:.3f}"
    )

    print(
        f"y_min: {y_min:.3f}"
    )
    print(
        f"y_max: {y_max:.3f}"
    )

    print(
        f"z_min: {z_min:.3f}"
    )
    print(
        f"z_max: {z_max:.3f}"
    )

    # ==========================================
    # 8. 保存
    # ==========================================

    base_name = os.path.splitext(
        target_pcd_name
    )[0]

    save_file = os.path.join(
        pcd_folder,
        f"{base_name}.txt"
    )

    with open(
        save_file,
        "w",
        encoding="utf-8"
    ) as f:

        f.write(
            "# Selected points\n"
        )

        for p in selected_points:

            f.write(
                f"{p[0]:.6f} "
                f"{p[1]:.6f} "
                f"{p[2]:.6f}\n"
            )

        f.write(
            "\n# FAST-Calib2 ROI\n"
        )

        f.write(
            f"x_min: {x_min:.3f}\n"
        )

        f.write(
            f"x_max: {x_max:.3f}\n"
        )

        f.write(
            f"y_min: {y_min:.3f}\n"
        )

        f.write(
            f"y_max: {y_max:.3f}\n"
        )

        f.write(
            f"z_min: {z_min:.3f}\n"
        )

        f.write(
            f"z_max: {z_max:.3f}\n"
        )

    print()
    print(
        f"[Save] ROI 保存到: "
        f"{save_file}"
    )
# ===================== main =====================

if __name__ == "__main__":
    # 1) 解析命令行参数：bag 路径 & 输出目录 & 可选 topic
    if len(sys.argv) > 1:
        bag_file = sys.argv[1]
    else:
        bag_file = os.path.join(os.getcwd(), "calib_data", "bag", "front")
        print(f"未指定 bag 路径，默认使用: {bag_file}")

    if len(sys.argv) > 2:
        output_dir = sys.argv[2]
    else:
        output_dir = os.getcwd()
        print(f"未指定输出目录，使用当前目录: {output_dir}")

    topic_override = sys.argv[3] if len(sys.argv) > 3 else ""

    if Path(bag_file).is_file() and Path(bag_file).suffix == ".bag":
        print(
            "[ERROR] ROS1 .bag 不支持，请先转换为 rosbag2 目录:\n"
            "  pip3 install rosbags\n"
            f"  rosbags-convert --src {bag_file} --dst {bag_file}_ros2",
            file=sys.stderr,
        )
        sys.exit(1)

    if not is_rosbag2_dir(bag_file):
        print(
            f"[ERROR] bag 路径 '{bag_file}' 无效，需要 rosbag2 目录（含 metadata.yaml）",
            file=sys.stderr,
        )
        sys.exit(1)

    if not os.path.isdir(output_dir):
        print(f"[ERROR] 输出目录 '{output_dir}' 不存在", file=sys.stderr)
        sys.exit(1)

    # 3) 自动检测 bag 中点云类型
    msg_type = detect_lidar_msg_type(bag_file)
    if msg_type is None:
        print("[ERROR] 未检测到支持的雷达消息类型，退出。", file=sys.stderr)
        sys.exit(1)

    # 4) 根据类型做对应的 PCD 转换
    if msg_type == "PointCloud2":
        pcd_path = convert_pointcloud2_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name=topic_override or "/hesai/pandar",
            pcd_name="sensor_PointCloud2_inten_ascii.pcd"
        )
    else:  # "CustomMsg"
        pcd_path = convert_livox_custom_bag_to_pcd(
            bag_file=bag_file,
            output_dir=output_dir,
            topic_name=topic_override or "/livox/lidar",
            pcd_name="livox_CustomMsg_inten_ascii.pcd"
        )

    if pcd_path is None:
        print("[ERROR] PCD 生成失败，退出。", file=sys.stderr)
        sys.exit(1)

    # 5) 对刚生成的这个 PCD 做交互式选点 + 范围保存
    select_and_save_points(
        pcd_folder=output_dir,
        target_pcd_name=os.path.basename(pcd_path)
    )
