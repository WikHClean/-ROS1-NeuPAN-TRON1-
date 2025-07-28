# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;rospy;std_msgs;sensor_msgs;geometry_msgs;tf2;tf2_ros;pcl_ros;pcl_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lpointcloud_transmission".split(';') if "-lpointcloud_transmission" != "" else []
PROJECT_NAME = "pointcloud_transmission"
PROJECT_SPACE_DIR = "/home/wch/neupan_ws/src/pointcloud_ws/install"
PROJECT_VERSION = "1.0.0"
