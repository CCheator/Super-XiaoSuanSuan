# Super-XiaoSuanSuan
Run ```python asr_node.py``` and  ```python tts_node.py``` in two terminals first.

Then run ```python talk.py``` to communicate with Super-XiaoSuanSuan. Say "你好" to wake up Super-XiaoSuanSuan.

If you want to wake Super-XiaoSuanSuan up by using face recognition, please run ```python face_recognize_node.py``` before talking.

## 导航整合模块使用指南

本项目包含三个导航整合层模块，用于实现对象检测和路径规划的协作运行。

### 📁 新增文件

- `navigation_coordinator_node.py` - 协调器节点，处理运动检测和数据转发
- `detector_node_integrated.py` - 整合的对象检测节点（支持独立/协调器两种模式）
- `path_planning_node_integrated.py` - 整合的路径规划节点（支持独立/协调器两种模式）

### 🚀 快速开始

#### 方案 A: 独立模式（向后兼容，无需协调器）

```bash
# 终端 1 - 启动对象检测
ros2 run <package_name> detector_node_integrated.py

# 终端 2 - 启动路径规划
ros2 run <package_name> path_planning_node_integrated.py
```

**特点**：两个节点独立工作，直接订阅相机原始数据，无需协调器。

#### 方案 B: 整合模式（推荐，节省资源，适合多人协作）

```bash
# 终端 1 - 启动协调器
ros2 run <package_name> navigation_coordinator_node.py

# 终端 2 - 启动对象检测（整合模式）
ros2 run <package_name> detector_node_integrated.py \
  --ros-args -p use_coordinator:=true

# 终端 3 - 启动路径规划（整合模式）
ros2 run <package_name> path_planning_node_integrated.py \
  --ros-args -p use_coordinator:=true
```

### 🎛️ 配置参数

#### navigation_coordinator_node.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `motion_detection_threshold` | 5000 | 运动区域阈值（像素） |
| `motion_detection_interval` | 0.5 | 检测间隔（秒） |
| `buffer_size` | 10 | 帧缓冲大小 |

**使用示例**：
```bash
ros2 run <package> navigation_coordinator_node.py \
  --ros-args -p motion_detection_threshold:=3000 \
            -p motion_detection_interval:=0.3
```

#### detector_node_integrated.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_coordinator` | false | 是否使用协调器模式 |
| `model_path` | yolov8n.pt | YOLO模型路径 |
| `confidence_threshold` | 0.5 | 检测置信度阈值 |
| `tts_interval` | 3.0 | TTS输出间隔（秒） |

**使用示例**：
```bash
ros2 run <package> detector_node_integrated.py \
  --ros-args -p use_coordinator:=true \
            -p confidence_threshold:=0.6 \
            -p tts_interval:=2.0
```

#### path_planning_node_integrated.py

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `use_coordinator` | false | 是否使用协调器模式 |
| `safety_distance_threshold` | 0.5 | 安全距离（米） |
| `tts_interval_change` | 2.0 | 方向改变时的语音输出间隔（秒） |
| `tts_interval_repeat` | 4.0 | 方向重复时的语音输出间隔（秒） |

**使用示例**：
```bash
ros2 run <package> path_planning_node_integrated.py \
  --ros-args -p use_coordinator:=true \
            -p safety_distance_threshold:=0.8 \
            -p tts_interval_change:=1.5
```
### 📋 预期日志输出

**启动协调器**：
```
[navigation_coordinator] [INFO] Navigation Coordinator started
```

**启动对象检测（整合模式）**：
```
[object_detector_integrated] [INFO] YOLO model loaded from yolov8n.pt
[object_detector_integrated] [INFO] Operating in COORDINATOR mode
```

**启动路径规划（整合模式）**：
```
[path_planner_integrated] [INFO] Operating in COORDINATOR mode
```

**运行中（检测到运动）**：
```
[navigation_coordinator] [INFO] Motion detected!
[object_detector_integrated] [INFO] 检测到: person, chair, table
[object_detector_integrated] [INFO] TTS sent: 检测到: person, chair, table
[path_planner_integrated] [INFO] TTS sent: 前方是空地，可以直走
```