# 【RM2026】部署模式低带宽落点图传 - 五大湖联合大学

用于RoboMaster部署模式下英雄机器人观测落点用的低带宽图传。

[演示视频](https://www.bilibili.com/video/BV12aDMBcEob)

[RM社区 - 开源报告](https://bbs.robomaster.com/article/1883295) (如果加载不出来可能是在审核中)

<img width="502" height="323" alt="视频封面" src="https://github.com/user-attachments/assets/a72e1683-be42-44d2-a3ae-7686517728fd" />

## 环境要求

- Ubuntu Linux
- ROS 2 kilted。 其他版本例如Humble可能要改一些QoS之类的API。
- 大恒相机的 Galaxy SDK。

## 安装依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep \
  python3-opencv python3-av \
  libopencv-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-ugly gstreamer1.0-libav
```

安装ROS相关依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

本工程的相机图像采集代码由rm-vision项目修改而来；
`hik_camera` 依赖以下路径，确保路径里面文件都在就可以。

- 头文件：`/opt/Galaxy_camera/inc`
- 库文件：`/opt/Galaxy_camera/lib`

主要API迁移映射：

- `MV_CC_EnumDevices` -> `GXUpdateDeviceList`
- `MV_CC_CreateHandle` + `MV_CC_OpenDevice` -> `GXOpenDevice`
- `MV_CC_StartGrabbing` / `MV_CC_StopGrabbing` -> `GXStreamOn` / `GXStreamOff`
- `MV_CC_GetImageBuffer` / `MV_CC_FreeImageBuffer` -> `GXGetImage`（用户缓冲区）
- `MV_CC_GetFloatValue` / `MV_CC_SetFloatValue` -> `GXGetFloat` / `GXSetFloat`

可选参数：

- `camera_index`：打开的相机序号（从 1 开始，默认 1）

## 编译启动
先`source`一下ROS的`setup.bash`。然后：
```bash
colcon build

source install/setup.bash
ros2 launch bringup sniper.launch.py
```
`sniper.launch.py`里面可以修改启动参数，比如图传分辨率，准星位置，dump图片用于调试，等等。详见文件内注释。

本工程仅为一个演示工程，开发过程中使用了LLM作为辅助。欢迎大家基于这个思路开发更好的自定义客户端。
