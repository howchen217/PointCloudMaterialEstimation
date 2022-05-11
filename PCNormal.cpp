//
// Created by haocheng on 10-5-22.
//

#include "PCNormal.h"


pcl::PointCloud<pcl::Normal>::Ptr PCNormal::getPCNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

//    float average_point_distance = getAveragePointDistance(cloud, tree);
//    std::cout << "Average point distance is: " << average_point_distance << std::endl;
//    // Use all neighbors in a sphere of radius 3cm
//    ne.setRadiusSearch (0.03);

    ne.setKSearch(10);

    // Compute the features
    ne.compute (*cloud_normals);

    return cloud_normals;
}

Vector3 PCNormal::getPCPointNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   const pcl::PointCloud<pcl::Normal>::Ptr cloud_normals,
                                   Vector3 point_coordinate){

    int i = getPCPointIndex(cloud, point_coordinate);

    float normal_x = cloud_normals->points[i].normal_x;
    float normal_y = cloud_normals->points[i].normal_y;
    float normal_z = cloud_normals->points[i].normal_z;

    Vector3 point_normal(normal_x, normal_y, normal_z);

    std::cout << "Got point: " << point_coordinate << " of index " << i << " that has normal: " << point_normal << std::endl;
    return Vector3(normal_x, normal_y, normal_z);
}



float PCNormal::getAveragePointDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud, const pcl::search::KdTree<pcl::PointXYZ>::Ptr& kdtree) {
    int total_count = inputCloud->width * inputCloud->height ;

    float* euclidian_distance = new float[total_count];

    kdtree->setInputCloud (inputCloud);

    int K = 2; //first will be the distance with point itself and second will the nearest point that's why "2"

    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);

    for (int i = 0; i < total_count; ++i)
    {
        if ( kdtree->nearestKSearch (inputCloud->points[i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
        {
            for (size_t j = 0; j < pointIdxNKNSearch.size (); ++j)
            {
                //saving all the distance in Vector
                euclidian_distance[i] =  pointNKNSquaredDistance[j];

            }
        }
    }

    float totalDistance = 0;

    for(int i = 0; i < total_count; i++)
    {
        //accumulating all distances
        totalDistance = totalDistance + euclidian_distance[i];
    }

    //calculating the mean distance
    float meanDistance = totalDistance/total_count;

    //freeing the allocated memory
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

//    searchPoint.x= searchPoint.x+ 0.0001;

    if (kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance)> 0)
    {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        {
//            std::cout << " Searched output index " << pointIdxNKNSearch[0] << "\n";
            return pointIdxNKNSearch[0];
        }
    }
}
