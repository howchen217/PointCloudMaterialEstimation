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
#include "PCNormal.h"

/**
 * Utility class for point cloud viewing.
 */
class PCViewer {
    public:
        /**
         * \brief Creates an RGB cloud with visualized material information.
         * @param cloud_with_material The cloud of PointXYZRGBMaterial point type.
         * @param centroids The centroids as a result of kmeans on material vectors.
         * @return An RGBXYZ point cloud with every centroid cluster of a different color.
         */
        static pcl::PointCloud<pcl::PointXYZRGB>::Ptr createVisualizedMaterialCloud(pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material, pcl::Kmeans::Centroids centroids);

        /**
         * \brief Show the input cloud.
         * @tparam T The cloud type.
         * @param cloud The input cloud.
         */
        template <typename T>
        static void viewCloud(T cloud);

        /**
         * \brief View the point normal of a specific point by index.
         * @param cloud_xyz The input cloud.
         * @param cloud_normals The cloud normals.
         * @param point_index The point index.
         */
        static void viewPCPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, int point_index);
};


#endif //PCMATTEX_PCVIEWER_H
