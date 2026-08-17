<<<<<<< HEAD
# fast-calib2
ros2
=======
# FAST-Calib2


**Key highlights include:**

1. A self-designed 3D reflective annular calibration target that avoids center extraction errors caused by hole-edge inflation and bleeding artifacts in previous circular-hole calibration boards.
2. A robust concentric-circle fitting method that uses the fixed inner and outer annulus radii as geometric constraints.
3. Automatic calibration board ROI extraction without manual pass-through tuning.
4. Geometry and radius quality checks for extracted annulus centers.
5. Single-scene and multi-scene LiDAR-camera extrinsic calibration without initial extrinsic parameters.


## 1. Prerequisites

- ROS 2 Humble (or compatible)
- PCL >= 1.8, OpenCV >= 4.0




Build:

```bash
cd <your_ros2_ws>/src
# 将本仓库放到 src 下
cd ..
source /opt/ros/humble/setup.bash
colcon build  --symlink-install
source install/setup.bash
```

## 使用说明

本节给出 FAST-Calib2 在 **ROS 2** 下的完整使用流程。

### 1. 准备标定数据


| 数据 | 说明 |
|------|------|
| 点云 bag | ROS 2 **rosbag2 目录**（含 `metadata.yaml`），topic 为 LiDAR 点云 |
| 图像 | 与 bag 同一时刻的 `.bmp` / `.png` 等静态图片 |


支持的点云消息类型：

- `sensor_msgs/msg/PointCloud2`（机械式 / 通用 LiDAR）

### 2. 配置参数

编辑 `config/qr_params.yaml`，至少需要确认以下项：

```yaml
# 相机内参（分辨率须与输入图像一致）
camera_width: 2448
camera_height: 2048
fx: ...
fy: ...
cx: ...
cy: ...

# 标定板几何（须与实物一致）
marker_size: 0.20
delta_width_circles: 0.5
delta_height_circles: 0.4
circle_radius: 0.12

# LiDAR 安装方向：分别表示“哪条 LiDAR 轴指向前方 / 上方”
lidar_forward_axis: "+x"
lidar_up_axis: "+z"

# 输入
lidar_topic: "/livox/lidar"
bag_path: "/绝对路径/to/mid_ros2"      # rosbag2 目录
image_path: "/绝对路径/to/mid.bmp"

# 输出
output_path: "/绝对路径/to/output"
use_auto_lidar_roi: true               # 不推荐开启，自动提取标定板 ROI
```

**LiDAR 安装轴说明**：取值必须为带符号的轴名（如 `+x`、`-y`），且 `forward` 与 `up` 必须互相垂直。左方向由右手系 `forward × left = up` 自动推导。

也可在 launch 时覆盖路径，无需改 yaml：

```bash
ros2 launch fast_calib calib.launch.py \
  bag_path:=/home/user/data/mid_ros2 \
  image_path:=/home/user/data/mid.bmp \
  output_path:=/home/user/output \
  lidar_topic:=/livox/lidar
```

### 3. 单场景标定

标定板静止放置，采集 **一组** 点云 + 图像后运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch fast_calib calib.launch.py
```

Launch 参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `rviz` | `true` | 是否启动 RViz2 查看调试点云 |
| `config_file` | 包内 `config/qr_params.yaml` | 自定义配置文件路径 |
| `bag_path` | yaml 中配置 | rosbag2 目录 |
| `image_path` | yaml 中配置 | 图像路径 |
| `lidar_topic` | yaml 中配置 | 点云 topic |
| `output_path` | yaml 中配置 | 结果输出目录 |

终端成功时会打印 **RMSE** 和 **外参矩阵 `T_cam_lidar`**（LiDAR → Camera）。

### 4. 多场景联合标定

将标定板分别置于 **正前、偏右、偏左** 三个位置（见下方示意图），各运行一次单场景标定，生成 3 组圆心记录后再联合求解：

```bash
# 依次运行 3 次单场景标定（修改 bag_path / image_path 或使用 launch 覆盖）
ros2 launch fast_calib calib.launch.py 
# 至少 3 组数据写入 output/circle_center_record.txt 后：
ros2 launch fast_calib multi_calib.launch.py
```

多场景结果保存为 `output/multi_calib_result.txt`（FAST-LIVO2 格式 `Rcl` / `Pcl`）。

### 5. LiDAR 圆心提取测试（可选）

在完整标定前，可单独验证 LiDAR 圆心提取是否正常：

```bash
ros2 run fast_calib lidar_center_test \
  /path/to/mid_ros2 /livox/lidar solid

# 机械式 LiDAR
ros2 run fast_calib lidar_center_test \
  /path/to/left_ros2 /lidar_points mech

# 指定安装轴
ros2 run fast_calib lidar_center_test \
  /path/to/mid_ros2 /livox/lidar solid +x +z
```

参数说明：`bag_path`、`lidar_topic`、模式（`auto` / `solid` / `mech`）、可选 `forward_axis up_axis`。

### 6. 输出文件

单场景标定完成后，`output_path` 目录下通常包含：

| 文件 | 说明 |
|------|------|
| `circle_center_record.txt` | 排序后的 LiDAR / 相机圆心坐标（多场景标定输入） |
| `calib_result.txt` | 单场景外参结果 |
| `colored_cloud.pcd` | 投影着色后的点云 |
| `qr_detect.png` | 相机检测结果可视化 |
| `multi_calib_result.txt` | 多场景联合标定结果（仅多场景流程） |

LiDAR 测试工具额外输出 `*_centers.txt`、`*_debug_cloud.pcd`。

### 7. RViz2 可视化

单场景标定节点运行后会持续发布以下 topic（`frame_id: map`）：

| Topic | 内容 |
|-------|------|
| `qr_cloud` | 相机检测圆心 |
| `center_cloud` | LiDAR 检测圆心 |
| `colored_cloud` | 着色点云 |
| `filtered_cloud` / `plane_cloud` | 标定板 ROI / 平面内点 |
| `annulus_cloud` / `boundary_cloud` | 高反环带 / 边界点 |
| `aligned_cloud` / `edge_cloud` | 对齐板面后的点云 |

关闭 RViz：`ros2 launch fast_calib calib.launch.py rviz:=false`

### 8. 常见问题

| 现象 | 可能原因 / 处理 |
|------|----------------|
| `ROS1 .bag is not supported` | 使用 `rosbags-convert` 转为 rosbag2 目录 |
| `Point cloud is empty` | 检查 `bag_path` 是否为目录、`lidar_topic` 是否与 bag 内一致 |
| `Unknown LiDAR message type` | bag 中 topic 类型不是 `PointCloud2` 或 Livox `CustomMsg` |
| `Invalid camera calibration` | `camera_width/height` 与输入图像分辨率不一致 |
| `Failed to sort target centers` | 检查 `lidar_forward_axis` / `lidar_up_axis` 是否与实际安装一致 |
| 检测不到 4 个圆心 | 确认标定板参数与实物一致；可先用 `lidar_center_test` 排查 LiDAR 侧 |

## 2. Calibration Target

FAST-Calib2 uses four reflective annuli and four visual markers on one board. The annuli are used by LiDAR center extraction, while the visual markers are used by the camera pipeline.

Materials:

- Board: PVC
- Reflective annulus stickers: 3M engineering-grade reflective film


DIY Calibration Target Tips:

1. Fabricate the board based on the schematic. Ensure a minimum thickness of 1 cm to avoid bending.
2. Apply reflective annulus stickers to the designated ring positions on the fabricated board.

## 3. Method Overview

Both LiDAR pipelines first **locate the calibration board automatically**, fit the board plane, and align the plane to `Z=0`. Center extraction is then performed in the aligned board frame.

Solid-state LiDAR pipeline:

1. Extract high-reflectivity annulus points on the fitted board plane.
2. Cluster the extracted annulus points.
3. Fit robust single circles as the default center estimate.
4. Optionally extract annulus boundary points and fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

Mechanical LiDAR pipeline:

1. Use LiDAR ring order within each scan to find intensity transition points on the annulus boundary.
2. Try both interpolated boundary points and high-reflectivity-side boundary points.
3. Cluster the extracted boundary points.
4. Fit fixed inner/outer radius concentric circles.
5. Select the best result by checking four-center geometry consistency against the known target geometry.

The final quality checks include center-to-center geometry error and annulus radius consistency.

## 4. Run Examples

单场景 / 多场景 / LiDAR 测试的详细步骤见上文 **[使用说明](#使用说明)**。快速命令如下：

Prepare static acquisition data in the `calib_data` folder (Download the example data from [Google Drive](https://drive.google.com/drive/folders/1VnMCsGj3Gat7dxe6IION0SfS7jYNMw1g?usp=sharing)):

- rosbag2 directory containing point cloud messages
- corresponding image

Describe the LiDAR mounting in `config/qr_params.yaml`:

```yaml
lidar_forward_axis: "+x"
lidar_up_axis: "+z"
```

The axis values must be signed, perpendicular axes such as `+x` and `-y`.

Run single-scene calibration:

```bash
ros2 launch fast_calib calib.launch.py
```

After collecting at least three scenes, run multi-scene joint calibration:

```bash
ros2 launch fast_calib multi_calib.launch.py
```

Typical multi-scene target placement:


## 5. Standalone LiDAR Center Extraction Test

<details>
<summary>Show Unit Test Usage</summary>

The repository also provides a LiDAR-only test tool for checking annulus center extraction before running full camera-LiDAR calibration. See **[使用说明 §5](#5-lidar-圆心提取测试可选)** for details.

```bash
ros2 run fast_calib lidar_center_test /path/to/mid_ros2 /livox/lidar solid
ros2 run fast_calib lidar_center_test /path/to/left_ros2 /lidar_points mech
```

The test tool writes:

- `*_centers.txt`: extracted annulus center coordinates
- `*_debug_cloud.pcd`: board point cloud, annulus points, boundary points, and center markers for visualization

Debug PCD colors:

- Board points: intensity color map
- Annulus points: green
- Solid-LiDAR boundary points: red
- Centers: white spheres

</details>
