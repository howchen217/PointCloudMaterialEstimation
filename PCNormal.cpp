//
// Created by haocheng on 10-5-22.
//

#include "PCNormal.h"



pcl::PointCloud<pcl::Normal>::Ptr PCNormal::getPCNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    //create kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    ne.setKSearch(10);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;
}


Vector3 PCNormal::getPCPointNormalByCoordinate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                               const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                               Vector3 point_coordinate){

    int i = getPCPointIndex(cloud, point_coordinate);

    float normal_x = cloud_normals->points[i].normal_x;
    float normal_y = cloud_normals->points[i].normal_y;
    float normal_z = cloud_normals->points[i].normal_z;

    Vector3 point_normal(normal_x, normal_y, normal_z);

    //std::cout << "Got point: " << point_coordinate << " of index " << i << " that has normal: " << point_normal << std::endl;
    return point_normal;
}


Vector3 PCNormal::getNormalVectorByIndex(const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, const int index){
    float normal_x = cloud_normals->points[index].normal_x;
    float normal_y = cloud_normals->points[index].normal_y;
    float normal_z = cloud_normals->points[index].normal_z;

    Vector3 point_normal(normal_x, normal_y, normal_z);
    return point_normal;
}


float PCNormal::getAveragePointDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree) {

    //create array of distances
    int total_count = inputCloud->width * inputCloud->height ;
    auto* euclidian_distance = new float[total_count];

    //set up kd-tree search
    kdtree->setInputCloud (inputCloud);
    int K = 2; //first will be the distance with point itself and second will the nearest point that's why "2"
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    //populate the distances in array
    for (int i = 0; i < total_count; ++i)
    {
        if ( kdtree->nearestKSearch (inputCloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
            {
                euclidian_distance[i] =  pointNKNSquaredDistance[j];
            }
        }
    }

    //get average distance
    float totalDistance = 0;
    for(int i = 0; i < total_count; i++)
    {
        totalDistance = totalDistance + euclidian_distance[i];
    }
    float meanDistance = totalDistance/total_count;

    delete  [] euclidian_distance;
    return meanDistance;
}

int PCNormal::getPCPointIndex(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Vector3 point_coordinate) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    kdtree.setInputCloud (cloud);

    pcl::PointXYZ searchPoint;

    searchPoint.x = point_coordinate[0];
    searchPoint.y = point_coordinate[1];
    searchPoint.z = point_coordinate[2];

    int K = 1;
    //std::vector <int> indices;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
//            std::cout << " Searched output index " << pointIdxNKNSearch[0] << "\n";
            return pointIdxNKNSearch[0];
        }
    }
}
