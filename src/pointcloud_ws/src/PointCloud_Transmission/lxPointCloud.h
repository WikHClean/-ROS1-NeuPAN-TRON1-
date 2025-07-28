// your_library_name.h

#ifndef lxPointCloud
#define lxPointCloud

#include <vector>
#include <cstdint>
#include <iostream>
struct PointCloudData {
    float x, y, z;
    std::uint8_t r, g, b, a;
};

struct PosData {
    float Pos_x, Pos_y, Pos_z, Qua_x, Qua_z, Qua_y, Qua_w;
};

// 添加IMU数据结构
struct ImuData {
    float acc_x, acc_y, acc_z;  // 加速度 (m/s^2)
    float gyr_x, gyr_y, gyr_z;  // 角速度 (rad/s)
    double timestamp;           // 时间戳 (s)
};

extern "C" {
    int sendDataToServer(std::string orin_ip);
    void receiveUdpData();
    void receivePosData();
    std::vector<PointCloudData> getAndClearPointCloudDataList();
    std::vector<PosData> GetPos();
    
    // 添加IMU数据接收函数声明
    void receiveImuData();
    std::vector<ImuData> getAndClearImuDataList();
}

#endif
