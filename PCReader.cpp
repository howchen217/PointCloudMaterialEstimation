//
// Created by haocheng on 10-5-22.
//

#include <pcl/io/pcd_io.h>
#include "PCReader.h"


pcl::PointCloud<pcl::PointXYZ>::Ptr PCReader::readASCToXYZCloudManual(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    std::cout << "Begin Loading Model" << std::endl;
    FILE *f = fopen(filename.c_str(), "r");

    if (nullptr == f) {
        std::cout << "ERROR: failed to open file: " << filename << std::endl;
        return nullptr;
    }

    float x, y, z;
    int r, g, b;
    float intensity;

    while (!feof(f)) {
        int n_args = fscanf(f, "%f %f %f %i %i %i %f", &x, &y, &z, &r, &g, &b, &intensity);
        if (n_args != 7)
            continue;

        pcl::PointXYZ point;
        point.x = x;
        point.y = y;
        point.z = z;

        cloud->push_back(point);
    }

    fclose(f);

    std::cout << "Loaded cloud with " << cloud->size() << " points." << std::endl;

    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCReader::readASCToXYZCloudPCl(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ASCIIReader reader;
    reader.setSepChars(" ");

    reader.read(filename, *cloud);
    return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCReader::readPCDXYZRGBCloud(const std::string &filename) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return nullptr;
    }
    std::cout << "Loaded XYZRGB PCD Cloud of size: " << cloud->size() << std::endl;
    return cloud;

}