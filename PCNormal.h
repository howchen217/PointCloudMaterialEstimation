//
// Created by haocheng on 10-5-22.
//

#ifndef PCMATTEX_PCNORMAL_H
#define PCMATTEX_PCNORMAL_H

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Vector3.hpp"


class PCNormal {
public:
    static pcl::PointCloud<pcl::Normal>::Ptr getPCNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    static Vector3 getPCPointNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, Vector3 point_coordinate);
private:
    static float getAveragePointDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& tree);
    static int getPCPointIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Vector3 point_coordinate);
};


#endif //PCMATTEX_PCNORMAL_H
