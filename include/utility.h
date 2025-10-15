#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_
#define PCL_NO_PRECOMPILE 
// <!-- liorf_yjz_lucky_boy -->
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <common_lib.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
// #include <opencv/cv.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <unordered_map>
#include <filesystem>

#define UTILITY_SET_PARAM(TYPE, NAME, DEFAULT, VAR) \
    declare_parameter<TYPE>(NAME, DEFAULT); \
    get_parameter(NAME, VAR);

using namespace std;
namespace fs = std::filesystem;

typedef pcl::PointXYZI PointType;

// <!-- liorf_localization_yjz_lucky_boy -->
std::shared_ptr<CommonLib::common_lib> common_lib_;

enum class SensorType { VELODYNE, OUSTER, LIVOX, ROBOSENSE, MULRAN};
enum class GpsTopicType { 
    SENSOR_MSGS_NAVSATFIX,
    NAV_MSGS_ODOMETRY
};

class ParamServer : public rclcpp::Node
{
public:
    string history_policy;
    string reliability_policy;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;
    GpsTopicType gpsTopicType;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    struct {
        float lat;
        float lon;
        float alt;
        bool useRef;
    } gpsRef;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    int point_filter_num;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    int imuType;
    float imuRate;
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // voxel filter paprams
    float mappingSurfLeafSize ;
    float surroundingKeyframeMapLeafSize;
    float loopClosureICPSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlag;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    // Mapping
    bool saveRawPointClouds;

    // Block Maps
    struct {
      bool save;
      bool saveGlobal;
      bool debug;
      int  width;
    } blockMapParam;

    ParamServer(std::string node_name, const rclcpp::NodeOptions & options) : Node(node_name, options)
    {   
        UTILITY_SET_PARAM(string, "history_policy", "history_keep_last", history_policy);
        UTILITY_SET_PARAM(string, "reliability_policy", "reliability_reliable", reliability_policy);

        UTILITY_SET_PARAM(string, "pointCloudTopic", "/points_raw", pointCloudTopic);
        UTILITY_SET_PARAM(string, "imuTopic", "/imu_correct", imuTopic);
        UTILITY_SET_PARAM(string, "odomTopic", "/odometry/imu", odomTopic);
        UTILITY_SET_PARAM(string, "gpsTopic", "/odometry/gps", gpsTopic);

        int gpsTopicInt;
        UTILITY_SET_PARAM(int, "gpsTopicType", -1, gpsTopicInt);
        if (gpsTopicInt == 0)
        {
            gpsTopicType = GpsTopicType::SENSOR_MSGS_NAVSATFIX;
        }
        else if (gpsTopicInt == 1)
        {
            gpsTopicType = GpsTopicType::NAV_MSGS_ODOMETRY;
        }
        else if (gpsTopicInt == -1) // Not using GPS
        {
            // Do nothing
        }
        else
        {
            RCLCPP_WARN_ONCE(
                get_logger(),
                "Invalid gps topic type (must be either -1 -> no GPS, 0 -> 'sensor_msgs/NavSatFix', or 1 -> 'nav_msgs/Odometry') to use GPS");
            // rclcpp::shutdown();
        }

        UTILITY_SET_PARAM(string, "lidarFrame", "base_link", lidarFrame);
        UTILITY_SET_PARAM(string, "baselinkFrame", "base_link", baselinkFrame);
        UTILITY_SET_PARAM(string, "odometryFrame", "odom", odometryFrame);
        UTILITY_SET_PARAM(string, "mapFrame", "map", mapFrame);
        
        UTILITY_SET_PARAM(bool, "useImuHeadingInitialization", false, useImuHeadingInitialization);
        UTILITY_SET_PARAM(bool, "useGpsElevation", false, useGpsElevation);
        UTILITY_SET_PARAM(float, "gpsCovThreshold", 2.0f, gpsCovThreshold);
        UTILITY_SET_PARAM(float, "poseCovThreshold", 25.0f, poseCovThreshold);

        // Declare the nested parameters with default values
        UTILITY_SET_PARAM(double, "gpsRef.latitude", 0.0, gpsRef.lat);
        UTILITY_SET_PARAM(double, "gpsRef.longitude", 0.0, gpsRef.lon);
        UTILITY_SET_PARAM(double, "gpsRef.altitude", 0.0, gpsRef.alt);
        UTILITY_SET_PARAM(bool, "gpsRef.useRef", false, gpsRef.useRef);

        UTILITY_SET_PARAM(bool, "savePCD", false, savePCD);
        UTILITY_SET_PARAM(string, "savePCDDirectory", "/Downloads/LOAM/", savePCDDirectory);

        std::string sensorStr;
        UTILITY_SET_PARAM(string, "sensor", " ", sensorStr);
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "livox")
        {
            sensor = SensorType::LIVOX;
        } else if  (sensorStr == "robosense") {
            sensor = SensorType::ROBOSENSE;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        } 
        else {
            RCLCPP_ERROR_STREAM(
                get_logger(),
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'livox' or 'robosense' or 'mulran'): " << sensorStr);
            rclcpp::shutdown();
        }

        UTILITY_SET_PARAM(int, "N_SCAN", 16, N_SCAN);
        UTILITY_SET_PARAM(int, "Horizon_SCAN", 1800, Horizon_SCAN);
        UTILITY_SET_PARAM(int, "downsampleRate", 1, downsampleRate);
        UTILITY_SET_PARAM(int, "point_filter_num", 3, point_filter_num);
        UTILITY_SET_PARAM(float, "lidarMinRange", 1.0f, lidarMinRange);
        UTILITY_SET_PARAM(float, "lidarMaxRange", 1000.0f, lidarMaxRange);

        UTILITY_SET_PARAM(int, "imuType", 0, imuType);
        UTILITY_SET_PARAM(float, "imuRate", 500.0f, imuRate);
        UTILITY_SET_PARAM(float, "imuAccNoise", 0.01f, imuAccNoise);
        UTILITY_SET_PARAM(float, "imuGyrNoise", 0.001f, imuGyrNoise);
        UTILITY_SET_PARAM(float, "imuAccBiasN", 0.0002f, imuAccBiasN);
        UTILITY_SET_PARAM(float, "imuGyrBiasN", 0.00003f, imuGyrBiasN);
        UTILITY_SET_PARAM(float, "imuGravity", 9.80511f, imuGravity);
        UTILITY_SET_PARAM(float, "imuRPYWeight", 0.01f, imuRPYWeight);

        double ida[] = { 1.0,  0.0,  0.0,
                         0.0,  1.0,  0.0,
                         0.0,  0.0,  1.0};
        std::vector < double > id(ida, std::end(ida));
        UTILITY_SET_PARAM(vector<double>, "extrinsicRot", id, extRotV);
        UTILITY_SET_PARAM(vector<double>, "extrinsicRPY", id, extRPYV);
        double zea[] = {0.0, 0.0, 0.0};
        std::vector < double > ze(zea, std::end(zea));
        UTILITY_SET_PARAM(vector<double>, "extrinsicTrans", ze, extTransV);

        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY).inverse();

        UTILITY_SET_PARAM(float, "mappingSurfLeafSize", 0.2f, mappingSurfLeafSize);
        UTILITY_SET_PARAM(float, "surroundingKeyframeMapLeafSize", 0.2f, surroundingKeyframeMapLeafSize);
        UTILITY_SET_PARAM(float, "z_tollerance", 1000.0f, z_tollerance);
        UTILITY_SET_PARAM(float, "rotation_tollerance", 1000.0f, rotation_tollerance);

        UTILITY_SET_PARAM(int, "numberOfCores", 2, numberOfCores);
        UTILITY_SET_PARAM(double, "mappingProcessInterval", 0.15f, mappingProcessInterval);

        UTILITY_SET_PARAM(float, "surroundingkeyframeAddingDistThreshold", 1.0f, surroundingkeyframeAddingDistThreshold);
        UTILITY_SET_PARAM(float, "surroundingkeyframeAddingAngleThreshold", 0.2f, surroundingkeyframeAddingAngleThreshold);
        UTILITY_SET_PARAM(float, "surroundingKeyframeDensity", 1.0f, surroundingKeyframeDensity);
        UTILITY_SET_PARAM(float, "loopClosureICPSurfLeafSize", 0.3f, loopClosureICPSurfLeafSize);
        UTILITY_SET_PARAM(float, "surroundingKeyframeSearchRadius", 50.0f, surroundingKeyframeSearchRadius);

        UTILITY_SET_PARAM(bool, "loopClosureEnableFlag", false, loopClosureEnableFlag);
        UTILITY_SET_PARAM(float, "loopClosureFrequency", 1.0f, loopClosureFrequency);
        UTILITY_SET_PARAM(int, "surroundingKeyframeSize", 50, surroundingKeyframeSize);
        UTILITY_SET_PARAM(float, "historyKeyframeSearchRadius", 10.0f, historyKeyframeSearchRadius);
        UTILITY_SET_PARAM(float, "historyKeyframeSearchTimeDiff", 30.0f, historyKeyframeSearchTimeDiff);
        UTILITY_SET_PARAM(int, "historyKeyframeSearchNum", 25, historyKeyframeSearchNum);
        UTILITY_SET_PARAM(float, "historyKeyframeFitnessScore", 0.3f, historyKeyframeFitnessScore);


        UTILITY_SET_PARAM(float, "globalMapVisualizationSearchRadius", 1e3f, globalMapVisualizationSearchRadius);
        UTILITY_SET_PARAM(float, "globalMapVisualizationPoseDensity", 10.0, globalMapVisualizationPoseDensity);
        UTILITY_SET_PARAM(float, "globalMapVisualizationLeafSize", 1.0f, globalMapVisualizationLeafSize);

        // Map
        UTILITY_SET_PARAM(bool, "map.saveRawPointClouds", true, saveRawPointClouds);

        // Block Maps
        UTILITY_SET_PARAM(bool, "block_map.saveBlockMaps", true, blockMapParam.save);
        UTILITY_SET_PARAM(bool, "block_map.saveBlockMapGlobalCloud", true, blockMapParam.saveGlobal);
        UTILITY_SET_PARAM(bool, "block_map.debug", false, blockMapParam.debug);
        UTILITY_SET_PARAM(int, "block_map.width", 50, blockMapParam.width);


        usleep(100);
    }

    sensor_msgs::msg::Imu imuConverter(const sensor_msgs::msg::Imu& imu_in)
    {
        sensor_msgs::msg::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();

        if (imuType) {
            // rotate roll pitch yaw
            Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
            Eigen::Quaterniond q_final = q_from * extQRPY;
            imu_out.orientation.x = q_final.x();
            imu_out.orientation.y = q_final.y();
            imu_out.orientation.z = q_final.z();
            imu_out.orientation.w = q_final.w();

            if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
            {
                RCLCPP_ERROR(get_logger(), "Invalid quaternion, please use a 9-axis IMU!");
                rclcpp::shutdown();
            }
        }

        return imu_out;
    }
};

template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &thisPub, const T& thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);

    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return rclcpp::Time(msg).seconds();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::msg::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::msg::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::msg::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf2::Quaternion orientation;
    tf2::fromMsg(thisImuMsg->orientation, orientation);
    tf2::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

rclcpp::QoS QosPolicy(const string &history_policy, const string &reliability_policy)
{
    rmw_qos_profile_t qos_profile;
    if (history_policy == "history_keep_last")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    else if (history_policy == "history_keep_all")
        qos_profile.history = rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_KEEP_ALL;

    if (reliability_policy == "reliability_reliable")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    else if (reliability_policy == "reliability_best_effort")
        qos_profile.reliability = rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

    qos_profile.depth = 2000;

    qos_profile.durability = rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_VOLATILE;
    qos_profile.deadline = RMW_QOS_DEADLINE_DEFAULT;
    qos_profile.lifespan = RMW_QOS_LIFESPAN_DEFAULT;
    qos_profile.liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
    qos_profile.liveliness_lease_duration = RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT;
    qos_profile.avoid_ros_namespace_conventions = false;

    return rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

#endif
