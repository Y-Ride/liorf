
#include "utility.h"
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>

struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZIRPYT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur =
        pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}

bool saveBlockMaps(vector<pcl::PointCloud<PointType>::Ptr> keyFrames, pcl::PointCloud<PointTypePose>::Ptr keyPoses6D)
{
    if (keyFrames.size() != keyPoses6D->size())
    {
        cerr << "Number of key frames and key poses doesn't match! KeyFrames: " << keyFrames.size() << " KeyPoses: " << keyPoses6D->size() << endl;
        return false;
    }

    const std::string dirPath = "/root/BlockMaps/";

    if (fs::exists(dirPath))
    {
        // fs::remove_all(dirPath);
        cout << "Directory exists" << endl;
    }
    else
    {
        if (!fs::create_directories(dirPath))
        {
            std::cerr << "Failed to create directory: " << dirPath << std::endl;
            return false;
        }
    }

    const int DIST_THRESHOLD = 50; // Meters
    std::unordered_map<std::string, pcl::PointCloud<PointType>::Ptr> blockMaps;

    pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

    printf("Num of keyPoses %d\n", (int)(keyPoses6D->size()));
    std::cout << "Num of keyFrames " << keyFrames.size() << std::endl;

    int tx = 0;
    int ty = 0;
    int cellx = 0;
    int celly = 0;
    string key = "N/A";

    for (int i = 0; i < (int)keyPoses6D->size(); i++)
    {
        tx = keyPoses6D->points[i].x;
        ty = keyPoses6D->points[i].y;
        cellx = (tx >= 0) ? (tx / DIST_THRESHOLD) + 1 : (tx / DIST_THRESHOLD) - 1;
        celly = (ty >= 0) ? (ty / DIST_THRESHOLD) + 1 : (ty / DIST_THRESHOLD) - 1;
        key = "bm_" + to_string(cellx) + "_" + to_string(celly);

        // cout << "tx: " << tx << ", ty: " << ty << ", cellx: " << cellx << ", celly: " << celly << ", key: " << key << endl;

        if (blockMaps.find(key) == blockMaps.end()) // Not found
        {
            blockMaps[key] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
        }

        *blockMaps[key] += *transformPointCloud(keyFrames[i], &keyPoses6D->points[i]);
    }

    for (const auto &kv : blockMaps)
    {
        static int i = 0;
        string filePath = dirPath + kv.first + ".pcd";

        if (pcl::io::savePCDFileBinary(filePath, *kv.second) < 0)
        {
            std::cerr << "Failed to save PCD file: " << filePath << std::endl;
        }

        *globalMapCloud += *kv.second;
        cout << "Adding blockmap " << kv.first << " to global map. " << ++i << " of " << blockMaps.size() << endl;
    }

    if (pcl::io::savePCDFileBinary(dirPath + "/GlobalMap.pcd", *globalMapCloud) < 0)
    {
        std::cerr << "Failed to save PCD file: " << dirPath + "/GlobalMap.pcd" << std::endl;
        return false;
    }

    return true;
}



int main()
{
    //  Compile: g++ -std=c++17 -O2 -Wall testc++.cpp -o test

    // std::unordered_map<std::string, int> map;

    // for (int i = 0; i < 10; i++){
    //     map[std::to_string(i)] = {i*10+5};
    // }

    // for (auto &kv : map){
    //     std::cout << kv.first << " = " << kv.second << std::endl;
    // }

    // for (int i = 5; i < 15; i++){
    //     std::string key = std::to_string(i);
    //     if (map.find(key) == map.end()) { // Not found
    //         std::cout << "No value found for Key \"" << key << "\"" << std::endl;
    //     }
    //     else {
    //         map[key] += 3;
    //         std::cout << "Found Key \"" << key << "\" with value \"" << map[key] << "\"" << std::endl;

    //     }
    // }

    // cout << 10 / 50 << endl;
    // cout << 100 / 50 << endl;
    // cout << 10.0 / 50 << endl;
    // cout << -122 / 50 << endl;
    // cout << (int)-69 / 50 << endl;
    // return true;

    // Load variables
    const std::string path_to_dir = "/root/ros2_ws/maps/longterm_mapping/7_3_2025-2";
    const std::string path_to_scans = path_to_dir + "/Scans/";
    const std::string path_to_transformations = path_to_dir + "/transformations.pcd";
    const std::string path_to_trajectory = path_to_dir + "/trajectory.pcd";

    std::vector<std::filesystem::path> entries;
    for (const auto &entry : std::filesystem::directory_iterator(path_to_scans))
    {
        if (entry.is_regular_file())
        {
            entries.push_back(entry.path());
        }
        else
        {
            std::cout << "Error! Not a regular file " << entry.path() << std::endl;
        }
    }

    std::sort(entries.begin(), entries.end());

    vector<pcl::PointCloud<PointType>::Ptr> keyFrameVect;
    for (const auto &path : entries)
    {
        std::cout << "\rLoading scan " << path;
        pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
        if (pcl::io::loadPCDFile(path, *cloud) < 0)
        {
            std::cout << "\t *** Error loading .pcd file from " << path << std::endl;
        }
        keyFrameVect.push_back(cloud);
    }
    std::cout << std::endl;

    pcl::PointCloud<PointTypePose>::Ptr keyPoses6D(new pcl::PointCloud<PointTypePose>);
    if (pcl::io::loadPCDFile(path_to_transformations, *keyPoses6D) < 0)
    {
        std::cout << "Error loading transformations from " << path_to_transformations << std::endl;
    }
    else
    {
        std::cout << "Loaded transformations successfully from \"" << path_to_transformations << "\"" << std::endl;
    }

    pcl::PointCloud<PointType>::Ptr keyPoses3D(new pcl::PointCloud<PointType>);
    if (pcl::io::loadPCDFile(path_to_trajectory, *keyPoses3D) < 0)
    {
        std::cout << "Error loading transformations from " << path_to_trajectory << std::endl;
    }
    else
    {
        std::cout << "Loaded transformations successfully from \"" << path_to_trajectory << "\"" << std::endl;
    }

    // printf("Num of keyPoses6D %d\n", (int)(keyPoses6D->size()));
    // printf("Num of keyPoses3D %d\n", (int)(keyPoses3D->size()));
    // std::cout << "Num of keyFrames " << keyFrameVect.size() << std::endl;

    saveBlockMaps(keyFrameVect, keyPoses6D);
}