## Project作业说明文档

根据助教老师建议，本课程笔记、作业均已经迁移至我的github上：[链接](https://github.com/ChenJiahao031008/sensor-fusion-for-slam). 

[TOC]

### 1 思路说明

本次作业的核心内容主要是修改接口以适配`UrbanNav`数据集。`UrbanNav`数据集和`KITTI`数据的数据源有所不同，表现如下：

+ 数据频率不同：`KITTI`数据集几个数据频率是保持一致的（10HZ），而`UrbanNav`数据集数据源频率不同，其中雷达数据频率为10HZ，IMU数据为100HZ，GNSS数据为1HZ，因此，需要对数据同步进行修改。
+ IMU和车辆坐标系不对齐，IMU和GNSS坐标系不对齐。尽管官方给的外参显示是对齐的（单位阵），但是实际使用时候发现有一定的偏差，需要手动修改。
+ 缺少速度信息：和KITTI数据集不同，`UrbanNav`数据集没有显式的速度标准，因此需要对代码进行一定修改。

### 2 代码修改

#### 2.1 数据频率修正

主要体现在IMU、GNSS与点云数据进行同步时：

同步时间和频率相关，频率越快的数据源准许的间隔越大，同时对node节点进行一定降频。

```c++
// ————————————————————————————————————IMU————————————————————————————————————— //
if (sync_time - UnsyncedData.front().time > 0.02) {
    UnsyncedData.pop_front();
    return false;
}

if (UnsyncedData.at(1).time - sync_time > 0.02) {
    UnsyncedData.pop_front();
    return false;
}
// ————————————————————————————————————gnss————————————————————————————————————— //
if (sync_time - UnsyncedData.front().time > 2) {
    UnsyncedData.pop_front();
    return false;
}

if (UnsyncedData.at(1).time - sync_time > 2) {
    UnsyncedData.pop_front();
    return false;
}
```

#### 2.2 点云畸变矫正

由于没有速度信息，因此用组合导航中的线速度代替里程计的线速度与IMU的角速度一起进行矫正，但是实践发现效果不是很好（不知道源点云有没有去过畸变），因此后续并不启用。

```c++
// TODO: 检查看是否需要手动去畸变,如果需要则引入组合导航的线速度
// this is lidar velocity:
current_velocity_data_.angular_velocity.x = current_imu_data_.angular_velocity.x;
current_velocity_data_.angular_velocity.y = current_imu_data_.angular_velocity.y;
current_velocity_data_.angular_velocity.z = current_imu_data_.angular_velocity.z;
current_velocity_data_.TransformCoordinate(lidar_to_imu_);
// TODO：矫正效果不好，需要再做检查
distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);
```

#### 2.3 IMU与车辆坐标系进行对齐

+ 由于`UrbanNav`数据集中没有发布任何tf，但是依据`KITTI`数据集规律，还需要发布`/imu_link`到`/velo_link`的变换等，因此在launch文件中写入KITTI中缺少的tf：

  ```xml
  <node pkg="tf" type="static_transform_publisher" name="velo2imu"
          args="0 0 0 2.5 0 0 imu_link velo_link 500"/>
  <node pkg="tf" type="static_transform_publisher" name="base2imu"
          args="0 0 0 0 0 0 1 base_link imu_link 500"/>
  ```

+ 对IMU数据进行矫正：

  ```c++
  current_imu_data_ = imu_data_buff_.front();
  Eigen::Quaterniond q(current_imu_data_.orientation.w, current_imu_data_.orientation.x, current_imu_data_.orientation.y, current_imu_data_.orientation.z);
  Eigen::Quaterniond q_tmp(lidar_to_imu_.block<3, 3>(0, 0).cast<double>());
  Eigen::Quaterniond res = q_tmp * q;
  current_imu_data_.orientation.w = res.w();
  current_imu_data_.orientation.x = res.x();
  current_imu_data_.orientation.y = res.y();
  current_imu_data_.orientation.z = res.z();
  current_imu_data_.orientation.Normlize();
  ```

#### 2.4 雷达和车辆坐标系进行对齐

```c++
Eigen::Matrix3f tmp;
tmp << 0, -1, 0, 1, 0, 0, 0, 0, 1;
current_laser_odom_data_.pose.block<3, 3>(0, 0) = tmp * current_laser_odom_data_.pose.block<3, 3>(0, 0);
transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);
```

这部分不同代码文件表现不一致，因此详细可以看代码。

### 3 结果展示和分析

**由于所给数据集过大，没有流量下完全部，并且网不好，2019的数据集下最后总会失败，因此暂时只用2020的数据集进行测试**

所有测试结果和实验对比（**以RMES为对比指标**）放在了`RESULT`文件夹下。

+ 数据集分析：

  <img src="README.assets/2021-11-11 21-56-40屏幕截图.png" style="zoom:50%;" />

+ 没有融合的建图：

  <img src="README.assets/2021-11-12 10-44-24屏幕截图.png" style="zoom: 33%;" />

+ 没有融合的定位：

  <img src="README.assets/2021-11-13 17-31-32屏幕截图-1636902371605.png" style="zoom:33%;" />

  ```yaml
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
  
         max	3.993084
        mean	2.334445
      median	2.317096
         min	1.202855
        rmse	2.414883
         sse	3359.036393
         std	0.618083
  
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
  
         max	1.626547
        mean	1.187114
      median	1.196348
         min	0.989221
        rmse	1.193483
         sse	820.455839
         std	0.123140
  ```

+ 带融合的滤波（不带约束与不带约束）：

  <img src="README.assets/2021-11-14 23-08-03屏幕截图.png" style="zoom:33%;" />

  ```
  APE w.r.t. full transformation (unit-less)
  (not aligned)
         max	1.746602
        mean	0.461872
      median	0.444019
         min	0.067361
        rmse	0.505940
         sse	727.482544
         std	0.206519
  APE w.r.t. full transformation (unit-less)
  (not aligned)
         max	1.447065
        mean	0.600480
      median	0.596449
         min	0.062172
        rmse	0.642130
         sse	1171.846292
         std	0.227499
  ```

+ 预积分建图：

  <img src="README.assets/2021-11-14 22-24-11屏幕截图.png" style="zoom:33%;" />

  ```
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
  
         max	3.133054
        mean	2.196400
      median	2.184208
         min	1.982757
        rmse	2.201346
         sse	2854.248976
         std	0.147479
  
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
  
         max	7.601884
        mean	2.851620
      median	2.336313
         min	1.777912
        rmse	3.114672
         sse	5713.995123
         std	1.252775
  ```

+ 图优化定位：

  <img src="README.assets/2021-11-14 12-16-06屏幕截图.png" style="zoom: 33%;" />

  ```
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
  
         max	2.545507
        mean	2.100983
      median	2.109585
         min	1.963889
        rmse	2.103065
         sse	12547.713664
         std	0.093558
  APE w.r.t. full transformation (unit-less)
  (with SE(3) Umeyama alignment)
         max	1.865376
        mean	0.270314
      median	0.223100
         min	0.013128
        rmse	0.327434
         sse	304.162503
         std	0.184778
  ```

  

