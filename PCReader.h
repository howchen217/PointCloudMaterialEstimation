//
// Created by haocheng on 10-5-22.
//

#ifndef PCMATTEX_PCREADER_H
#define PCMATTEX_PCREADER_H

#include <pcl/point_types.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_cloud.h>

class PCReader {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr parseToXYZCloudManual(const std::string &filename);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr parseToXYZCloudPCl();

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCDXYZRGBCloud(const std::string& filename);
};

#endif //PCMATTEX_PCREADER_H
