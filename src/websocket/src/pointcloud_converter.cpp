#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher cloud_pub;

// 定义与Fast-LIO兼容的点类型
struct PointXYZIRT {
    PCL_ADD_POINT4D;  // 位置 + 填充
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint16_t, ring, ring)
                                 (float, time, time)
)

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 创建一个新的点云，包含Fast-LIO需要的字段
    pcl::PointCloud<PointXYZIRT>::Ptr cloud_processed(new pcl::PointCloud<PointXYZIRT>);
    cloud_processed->header = cloud->header;
    cloud_processed->width = cloud->width;
    cloud_processed->height = cloud->height;
    cloud_processed->is_dense = cloud->is_dense;
    cloud_processed->points.resize(cloud->points.size());

    // 复制点云数据，添加必要的字段
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        cloud_processed->points[i].x = cloud->points[i].x;
        cloud_processed->points[i].y = cloud->points[i].y;
        cloud_processed->points[i].z = cloud->points[i].z;
        cloud_processed->points[i].intensity = 100.0f;  // 默认强度
        cloud_processed->points[i].ring = 0;            // 默认扫描线
        cloud_processed->points[i].time = 0.0f;         // 默认时间戳
    }

    // 转换回ROS消息
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_processed, output);
    output.header = cloud_msg->header;

    // 发布转换后的消息
    cloud_pub.publish(output);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_converter");
    ros::NodeHandle nh;

    // 创建订阅者和发布者
    ros::Subscriber cloud_sub = nh.subscribe("/livox/lidar", 10, cloudCallback);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox/lidar_processed", 10);

    ROS_INFO("PointCloud converter started - adding required fields for Fast-LIO");

    ros::spin();
    return 0;
}
