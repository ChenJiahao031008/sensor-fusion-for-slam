/*
 * @Description:
 * @Author: Ren Qian
 * @Date: 2020-02-06 20:42:23
 */
#include "lidar_localization/sensor_data/gnss_data.hpp"

#include "glog/logging.h"
#include <iostream>

//静态成员变量必须在类外初始化
double lidar_localization::GNSSData::origin_longitude = 0.0;
double lidar_localization::GNSSData::origin_latitude = 0.0;
double lidar_localization::GNSSData::origin_altitude = 0.0;
bool lidar_localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian lidar_localization::GNSSData::geo_converter;

namespace lidar_localization {
void GNSSData::InitOriginPosition() {
    // TODO: orign for /navsat/origin and /novatel_data/inspvax
    // std::cout << "GNSS Data: " << std::endl;
    // std::cout << "Latitude: " << latitude << std::endl;
    // std::cout << "Longitude: " << longitude << std::endl;
    // std::cout << "Altitude: " << altitude << std::endl
    //           << std::endl;
    latitude = 22.3304807923;
    longitude = 114.179526047;
    altitude = 11.7615366792;

    // geo_converter.Reset(48.9827, 8.39046, 116.396);
    geo_converter.Reset(latitude, longitude, altitude);

    origin_longitude = longitude;
    origin_latitude = latitude;
    origin_altitude = altitude;

    // std::cout << "Before GNSS Data: " << std::endl;
    // std::cout << "origin_longitude: " << origin_longitude << std::endl;
    // std::cout << "origin_latitude: " << origin_latitude << std::endl;
    // std::cout << "origin_altitude: " << origin_altitude << std::endl
    //           << std::endl;
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    // std::cout << "[DEBUG] GNSS Data: " << std::endl;
    // std::cout << "Latitude: " << latitude << std::endl;
    // std::cout << "Longitude: " << longitude << std::endl;
    // std::cout << "Altitude: " << altitude << std::endl
    //           << std::endl;

    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
    // std::cout << "[DEBUG]  After Data: " << std::endl;
    // std::cout << "local_E: " << local_E << std::endl;
    // std::cout << "local_N: " << local_N << std::endl;
    // std::cout << "local_U: " << local_U << std::endl
    //           << std::endl;
}

bool GNSSData::SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 2) {
            UnsyncedData.pop_front();
            return false;
        }
        if (UnsyncedData.at(1).time - sync_time > 2) {
            UnsyncedData.pop_front();
            return false;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);

    return true;
}
}
