#include "lxPointCloud.h"
#include <iostream>
#include <thread>
#include <unistd.h>

int main(int argc, char** argv) {
    
    // Link to device and start mapping

    int socket = sendDataToServer(argv[1]);
    // Create a new Thread that receive the data
    std::thread udpThread(receiveUdpData);
    std::thread posThread(receivePosData);

    while (true) {
        // Get and clear the position data
        std::vector<PosData> currentPosDataList = GetPos();
	int i=0;
        // print pos data if needed
        for (const auto &currentPosData : currentPosDataList) {
         std::cout << "pos_x: " << currentPosData.Pos_x
                     << "pos_y: " << currentPosData.Pos_y
                     << "pos_z: " << currentPosData.Pos_z
                     << "qua_x: " << currentPosData.Qua_x
                     << "qua_z: " << currentPosData.Qua_z
                     << "qua_y: " << currentPosData.Qua_y
                     << "qua_w: " << currentPosData.Qua_w << std::endl;
         }

        // Get and clear the point cloud data
        std::vector<PointCloudData> currentData = getAndClearPointCloudDataList();

         //print lidar data if needed
        for (const auto &data : currentData) {
            std::cout << "x: " << data.x << " y: " << data.y << " z: " << data.z
                       << " r: " << static_cast<int>(data.r)
                       << " g: " << static_cast<int>(data.g)
                       << " b: " << static_cast<int>(data.b)
                       << " a: " << static_cast<int>(data.a) << std::endl;
                       i++;
        }
        sleep(1);
	std::cout << "The size of currentData is: " << currentData.size() << std::endl;
        std::cout << "currentData_size:" << i << std::endl;

        
    }
	
    //close Socket to Stop Mapping
    close(socket);

    return 0;
}
