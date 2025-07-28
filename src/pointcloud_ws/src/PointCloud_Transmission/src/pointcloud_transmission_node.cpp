#include <ros/ros.h>
#include <cmath>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <mutex>
#include <random>
#include "../lxPointCloud.h"

// 添加存根函数实现，因为共享库中没有这些函数
std::vector<ImuData> getAndClearImuDataList() {
    // 返回空列表，因为我们使用模拟IMU数据
    return std::vector<ImuData>();
}

void receiveImuData() {
    // 空实现，因为我们使用模拟IMU数据
    ROS_INFO("receiveImuData called, but using simulated IMU data instead");
    ros::Duration(1.0).sleep();
}

class PointCloudTransmissionNode
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    
    // Publishers
    ros::Publisher pointcloud_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher pose_pub_;
    ros::Publisher imu_pub_;
    
    // TF broadcaster
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    
    // Parameters
    std::string server_ip_;
    std::string frame_id_;
    std::string child_frame_id_;
    double publish_rate_;
    bool publish_tf_;
    std::string imu_frame_id_;
    bool simulate_imu_;
    double imu_rate_;
    
    // Threading
    std::thread udp_thread_;
    std::thread pos_thread_;
    std::thread imu_thread_;
    std::mutex data_mutex_;
    
    // Socket handle
    int socket_;
    bool running_;
    
    // IMU simulation parameters
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<float> acc_noise_;
    std::normal_distribution<float> gyro_noise_;
    
public:
    PointCloudTransmissionNode() : pnh_("~"), running_(false)
    {
        // Load parameters
        pnh_.param<std::string>("server_ip", server_ip_, "10.42.0.1");
        pnh_.param<std::string>("frame_id", frame_id_, "map");
        pnh_.param<std::string>("child_frame_id", child_frame_id_, "lidar");
        pnh_.param<double>("publish_rate", publish_rate_, 10.0);
        pnh_.param<bool>("publish_tf", publish_tf_, true);
        pnh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
        pnh_.param<double>("imu_rate", imu_rate_, 200.0);
        pnh_.param<bool>("simulate_imu", simulate_imu_, true);
        
        // Initialize publishers
        pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose", 1);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>("livox/imu", 10);
        
        ROS_INFO("PointCloud Transmission Node initialized");
        ROS_INFO("Server IP: %s", server_ip_.c_str());
        ROS_INFO("Frame ID: %s", frame_id_.c_str());
        ROS_INFO("Child Frame ID: %s", child_frame_id_.c_str());
        ROS_INFO("IMU Frame ID: %s", imu_frame_id_.c_str());
        ROS_INFO("Publish Rate: %.1f Hz", publish_rate_);
    }
    
    ~PointCloudTransmissionNode()
    {
        shutdown();
    }
    
    bool initialize()
    {
        try {
            // Connect to server
            socket_ = sendDataToServer(server_ip_);
            if (socket_ < 0) {
                ROS_ERROR("Failed to connect to server at %s", server_ip_.c_str());
                return false;
            }
            
            running_ = true;
            
            // Start data receiving threads
            udp_thread_ = std::thread(&PointCloudTransmissionNode::udpDataThread, this);
            pos_thread_ = std::thread(&PointCloudTransmissionNode::posDataThread, this);
            imu_thread_ = std::thread(&PointCloudTransmissionNode::imuDataThread, this);
            
            ROS_INFO("Successfully connected to server and started data threads");
            return true;
        }
        catch (const std::exception& e) {
            ROS_ERROR("Failed to initialize: %s", e.what());
            return false;
        }
    }
    
    void shutdown()
    {
        running_ = false;
        
        if (udp_thread_.joinable()) {
            udp_thread_.join();
        }
        
        if (pos_thread_.joinable()) {
            pos_thread_.join();
        }
        
        if (imu_thread_.joinable()) {
            imu_thread_.join();
        }
        
        if (socket_ >= 0) {
            close(socket_);
        }
        
        ROS_INFO("PointCloud Transmission Node shutdown complete");
    }
    
    void spin()
    {
        ros::Rate rate(publish_rate_);
        
        while (ros::ok() && running_) {
            publishData();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void udpDataThread()
    {
        ROS_INFO("UDP data thread started");
        receiveUdpData();
    }
    
    void posDataThread()
    {
        ROS_INFO("Position data thread started");
        receivePosData();
    }
    
    void imuDataThread()
    {
        ROS_INFO("IMU data thread started");
        
        if (simulate_imu_) {
            // 如果使用模拟IMU数据，则在这个线程中生成并发布
            simulateImuData();
        } else {
            // 否则尝试从共享库接收真实IMU数据
            receiveImuData();
        }
    }
    
    void publishData()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        ros::Time current_time = ros::Time::now();
        
        publishPointCloud(current_time);
        
        publishPose(current_time);
        
        publishImuData(current_time);
    }
    
    void publishPointCloud(const ros::Time& timestamp)
    {
        std::vector<PointCloudData> currentData = getAndClearPointCloudDataList();
        
        if (currentData.empty()) {
            return;
        }
        
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        cloud->header.frame_id = child_frame_id_;
        cloud->width = currentData.size();
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(currentData.size());
        
        for (size_t i = 0; i < currentData.size(); ++i) {
            cloud->points[i].x = currentData[i].x;
            cloud->points[i].y = currentData[i].y;
            cloud->points[i].z = currentData[i].z;
            cloud->points[i].r = currentData[i].r;
            cloud->points[i].g = currentData[i].g;
            cloud->points[i].b = currentData[i].b;
            cloud->points[i].a = currentData[i].a;
        }
        
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = child_frame_id_;
        
        pointcloud_pub_.publish(cloud_msg);
        
        ROS_DEBUG("Published point cloud with %zu points", currentData.size());
    }
    
    void publishLaserScan(const ros::Time& timestamp)
    {
        std::vector<PointCloudData> currentData = getAndClearPointCloudDataList();

        if (currentData.empty()) {
            ROS_WARN("No point cloud data received for laser scan");
            return;
        }

        sensor_msgs::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = child_frame_id_;

        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0;
        scan_msg.time_increment = 0.0;
        scan_msg.scan_time = 1.0 / publish_rate_;
        scan_msg.range_min = 0.05;
        scan_msg.range_max = 10.0;

        int num_readings = (scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment;
        scan_msg.ranges.assign(num_readings, std::numeric_limits<float>::infinity());

        int valid_points = 0;
        int range_filtered = 0;
        int z_filtered = 0;
        int total_processed = 0;
        
        for (const auto& point : currentData) {
            total_processed++;
            
            if (point.z < -0.5 || point.z > 0.5) {
                z_filtered++;
                continue;
            }
            
            float angle = atan2(point.y, point.x);
            float range = sqrt(point.x * point.x + point.y * point.y);
            
            if (range < scan_msg.range_min || range > scan_msg.range_max) {
                range_filtered++;
                continue;
            }
            
            int index = (angle - scan_msg.angle_min) / scan_msg.angle_increment;
            if (index >= 0 && index < num_readings) {
                if (range < scan_msg.ranges[index]) {
                    scan_msg.ranges[index] = range;
                    valid_points++;
                }
            }
        }

        for (int i = 0; i < num_readings; i++) {
            if (std::isinf(scan_msg.ranges[i])) {
                scan_msg.ranges[i] = std::numeric_limits<float>::quiet_NaN();
            }
        }

        scan_pub_.publish(scan_msg);

        ROS_INFO("Laser Scan Stats: Input=%d, Z-filtered=%d, Range-filtered=%d, Valid=%d", 
                 total_processed, z_filtered, range_filtered, valid_points);
        
        if (valid_points < 50) {
            ROS_WARN("Very few valid laser points (%d)! Check MID360 connection and environment.", valid_points);
        }
    }
    
    void publishPose(const ros::Time& timestamp)
    {
        std::vector<PosData> currentPosDataList = GetPos();
        
        if (currentPosDataList.empty()) {
            return;
        }
        
        const PosData& posData = currentPosDataList.back();
        
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = timestamp;
        pose_msg.header.frame_id = frame_id_;
        
        pose_msg.pose.position.x = posData.Pos_x;
        pose_msg.pose.position.y = posData.Pos_y;
        pose_msg.pose.position.z = posData.Pos_z;
        
        pose_msg.pose.orientation.x = posData.Qua_x;
        pose_msg.pose.orientation.y = posData.Qua_y;
        pose_msg.pose.orientation.z = posData.Qua_z;
        pose_msg.pose.orientation.w = posData.Qua_w;
        
        pose_pub_.publish(pose_msg);
        
        if (publish_tf_) {
            geometry_msgs::TransformStamped transform_msg;
            transform_msg.header.stamp = timestamp;
            transform_msg.header.frame_id = frame_id_;
            transform_msg.child_frame_id = child_frame_id_;
            
            transform_msg.transform.translation.x = posData.Pos_x;
            transform_msg.transform.translation.y = posData.Pos_y;
            transform_msg.transform.translation.z = posData.Pos_z;
            
            transform_msg.transform.rotation.x = posData.Qua_x;
            transform_msg.transform.rotation.y = posData.Qua_y;
            transform_msg.transform.rotation.z = posData.Qua_z;
            transform_msg.transform.rotation.w = posData.Qua_w;
            
            tf_broadcaster_.sendTransform(transform_msg);
        }
        
        ROS_DEBUG("Published pose: pos(%.2f, %.2f, %.2f) quat(%.2f, %.2f, %.2f, %.2f)",
                  posData.Pos_x, posData.Pos_y, posData.Pos_z,
                  posData.Qua_x, posData.Qua_y, posData.Qua_z, posData.Qua_w);
    }
    
    void publishImuData(const ros::Time& timestamp)
    {
        if (!simulate_imu_) {
            // 只有在不模拟IMU数据时才从共享库获取IMU数据
            std::vector<ImuData> imuDataList = getAndClearImuDataList();
            
            if (imuDataList.empty()) {
                return;
            }
            
            for (const auto& imuData : imuDataList) {
                sensor_msgs::Imu imu_msg;
                
                imu_msg.header.stamp = ros::Time(imuData.timestamp);
                imu_msg.header.frame_id = imu_frame_id_;
                
                imu_msg.linear_acceleration.x = imuData.acc_x;
                imu_msg.linear_acceleration.y = imuData.acc_y;
                imu_msg.linear_acceleration.z = imuData.acc_z;
                
                imu_msg.angular_velocity.x = imuData.gyr_x;
                imu_msg.angular_velocity.y = imuData.gyr_y;
                imu_msg.angular_velocity.z = imuData.gyr_z;
                
                imu_msg.orientation.w = 1.0;
                imu_msg.orientation.x = 0.0;
                imu_msg.orientation.y = 0.0;
                imu_msg.orientation.z = 0.0;
                
                double orientation_cov = 0.01;
                double angular_vel_cov = 0.01;
                double linear_acc_cov = 0.01;
                
                for (int i = 0; i < 9; i++) {
                    imu_msg.orientation_covariance[i] = 0.0;
                    imu_msg.angular_velocity_covariance[i] = 0.0;
                    imu_msg.linear_acceleration_covariance[i] = 0.0;
                }
                
                imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = orientation_cov;
                imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = angular_vel_cov;
                imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = linear_acc_cov;
                
                imu_pub_.publish(imu_msg);
            }
            
            ROS_DEBUG("Published %zu IMU messages", imuDataList.size());
        }
        // 如果是模拟IMU数据，则在simulateImuData线程中直接发布，这里不需要做任何事情
    }
    
    void simulateImuData()
    {
        ROS_INFO("Starting IMU simulation at %.1f Hz", imu_rate_);
        
        // 初始化随机数生成器
        gen_ = std::mt19937(rd_());
        
        // 设置噪声分布 - 加速度噪声约0.01 m/s^2，角速度噪声约0.001 rad/s
        acc_noise_ = std::normal_distribution<float>(0.0f, 0.01f);
        gyro_noise_ = std::normal_distribution<float>(0.0f, 0.001f);
        
        ros::Rate rate(imu_rate_);
        
        while (running_) {
            // 创建模拟IMU数据
            ImuData imuData;
            
            // 设置时间戳
            imuData.timestamp = ros::Time::now().toSec();
            
            // 模拟静止状态下的IMU数据 - 只有重力和噪声
            imuData.acc_x = acc_noise_(gen_);
            imuData.acc_y = acc_noise_(gen_);
            imuData.acc_z = 9.81f + acc_noise_(gen_); // 重力加速度 + 噪声
            
            imuData.gyr_x = gyro_noise_(gen_);
            imuData.gyr_y = gyro_noise_(gen_);
            imuData.gyr_z = gyro_noise_(gen_);
            
            // 发布模拟IMU数据
            sensor_msgs::Imu imu_msg;
            
            imu_msg.header.stamp = ros::Time(imuData.timestamp);
            imu_msg.header.frame_id = imu_frame_id_;
            
            imu_msg.linear_acceleration.x = imuData.acc_x;
            imu_msg.linear_acceleration.y = imuData.acc_y;
            imu_msg.linear_acceleration.z = imuData.acc_z;
            
            imu_msg.angular_velocity.x = imuData.gyr_x;
            imu_msg.angular_velocity.y = imuData.gyr_y;
            imu_msg.angular_velocity.z = imuData.gyr_z;
            
            imu_msg.orientation.w = 1.0;
            imu_msg.orientation.x = 0.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            
            double orientation_cov = 0.01;
            double angular_vel_cov = 0.01;
            double linear_acc_cov = 0.01;
            
            for (int i = 0; i < 9; i++) {
                imu_msg.orientation_covariance[i] = 0.0;
                imu_msg.angular_velocity_covariance[i] = 0.0;
                imu_msg.linear_acceleration_covariance[i] = 0.0;
            }
            
            imu_msg.orientation_covariance[0] = imu_msg.orientation_covariance[4] = imu_msg.orientation_covariance[8] = orientation_cov;
            imu_msg.angular_velocity_covariance[0] = imu_msg.angular_velocity_covariance[4] = imu_msg.angular_velocity_covariance[8] = angular_vel_cov;
            imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4] = imu_msg.linear_acceleration_covariance[8] = linear_acc_cov;
            
            imu_pub_.publish(imu_msg);
            
            rate.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_transmission_node");
    
    PointCloudTransmissionNode node;
    
    if (!node.initialize()) {
        ROS_ERROR("Failed to initialize PointCloud Transmission Node");
        return -1;
    }
    
    ROS_INFO("PointCloud Transmission Node started successfully");
    
    ros::AsyncSpinner spinner(2);
    spinner.start();
    
    node.spin();
    
    spinner.stop();
    node.shutdown();
    
    return 0;
}
