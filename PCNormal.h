//
// Created by haocheng on 10-5-22.
//

#ifndef PCMATTEX_PCNORMAL_H
#define PCMATTEX_PCNORMAL_H

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Vector3.hpp"

/**
 * \brief Utility class for calculating the normals of point clouds.
 */
class PCNormal {
public:

    /**
    * \brief Calculate the cloud normals of a PointXYZ cloud.
    * @param cloud The input cloud.
    * @return The normals of the cloud.
    */
    static pcl::PointCloud<pcl::Normal>::Ptr getPCNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);


    /**
    * \brief Given the input cloud, cloud normals and coordinate of a point, get the cloud normal of that point.
    * @param cloud The input cloud.
    * @param cloud_normals The cloud normals.
    * @param point_coordinate The point coordinate of the point to get normal from.
    * @return The cloud normal of the input point at that coordinate.
    */
    static Vector3 getPCPointNormalByCoordinate(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                                const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                                Vector3 point_coordinate);

    /**
     * \brief Get the normal vector by point index.
     * @param cloud_normals The cloud normals.
     * @param index The point index to get normal vector from.
     * @return The normal vector of the point by its index.
     */
    static Vector3 getNormalVectorByIndex(const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const int index);

private:

    /**
     * Get the average distance between points of a point cloud.
     * @param cloud The point cloud.
     * @param tree The kd-tree.
     * @return The average distance between all points.
     */
    static float getAveragePointDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                         const pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree);

    /**
     * Get the index of a point cloud point.
     * @param cloud The pont cloud.
     * @param point_coordinate The point coordinate of the point.
     * @return The index of the point cloud point.
     */
    static int getPCPointIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Vector3 point_coordinate);

};


#endif //PCMATTEX_PCNORMAL_H
