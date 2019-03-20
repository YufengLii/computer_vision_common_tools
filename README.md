# computer_vision_common_tools

### Ubuntu Opencv 一键安装脚本

```shell
cd Ubuntu_opencv_install_script
chmod +x *
./opencv_latest.sh
```

### 彩色图+深度图+相机内参生成点云

- 相机内参请根据自己的相机参数在`generate_pointcloud.py`中更改
- 使用方式
```shell
generate_pointcloud.py ./rgb.jpg ./depth.png ./point_cloud.ply
```


### RGBD视频序列与位姿变换

- 使用相机内参、RGBD视频序列以及相机位姿生成场景地图
- `rgbd_sequence2pointclouds.cpp` 根据相机内参和RGBD视频生成点云序列
- `alignclouds_use_camera_pose.cpp` 使用相机位姿对点云序列进行变换，得到场景地图

- 使用示例：
```
./rgbd_sequence2pointclouds path_to_files
./alignclouds_use_camera_pose path_to_files
```

### XYZ2PointCloud
- 在MATLAB中将三维坐标数据转变成点云格式，详细说明见`plywrite.m`与`stlwrite.m`
