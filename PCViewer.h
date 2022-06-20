//
// Created by haocheng on 9-5-22.
//

#ifndef PCMATTEX_PCVIEWER_H
#define PCMATTEX_PCVIEWER_H

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ml/kmeans.h>
#include "PointXYZRGBMaterial.h"
#include "Vector3.hpp"

/**
 * Utility class for point cloud viewing.
 */
class PCViewer {
    public:
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr createVisualizedMaterialCloud(pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material, pcl::Kmeans::Centroids centroids);

        template <typename T>
        static void viewCloud(T cloud);

};


#endif //PCMATTEX_PCVIEWER_H
