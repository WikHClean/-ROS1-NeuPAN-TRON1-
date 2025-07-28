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

extern "C" {
    int sendDataToServer(std::string orin_ip);
    void receiveUdpData();
    void receivePosData();
    std::vector<PointCloudData> getAndClearPointCloudDataList();
    std::vector<PosData> GetPos();
}

#endif
