//
// Created by haocheng on 9-5-22.
//


#include "PCViewer.h"

template <typename T>
void PCViewer::viewCloud (T cloud) {

    pcl::visualization::CloudViewer viewer("Simple XYZ Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }
}

float distance(Vector3 v1, Vector3 v2){
    return (v1 - v2).absolute();
}

Vector3 HSVtoRGB(float H, float S,float V){
    if(H>360 || H<0 || S>100 || S<0 || V>100 || V<0){
        //cout<<"The givem HSV values are not in valid range"<<endl;
        throw std::invalid_argument("the given HSV values are not in valid range");
    }
    float s = S/100;
    float v = V/100;
    float C = s*v;
    float X = C*(1-abs(fmod(H/60.0, 2)-1));
    float m = v-C;
    float r,g,b;
    if(H >= 0 && H < 60){
        r = C,g = X,b = 0;
    }
    else if(H >= 60 && H < 120){
        r = X,g = C,b = 0;
    }
    else if(H >= 120 && H < 180){
        r = 0,g = C,b = X;
    }
    else if(H >= 180 && H < 240){
        r = 0,g = X,b = C;
    }
    else if(H >= 240 && H < 300){
        r = X,g = 0,b = C;
    }
    else{
        r = C,g = 0,b = X;
    }
    int R = (r+m)*255;
    int G = (g+m)*255;
    int B = (b+m)*255;

    return Vector3(R, G, B);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCViewer::createVisualizedMaterialCloud(pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material, pcl::Kmeans::Centroids centroids){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualized_material_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::size_t i = 0; i < cloud_with_material->size(); ++i)
    {
        //compute distance to each centroid
        PointXYZRGBMaterial current_point = cloud_with_material->points[i];
        Vector3 current_material = Vector3(current_point.emissivity, current_point.albedo, current_point.reflectance);
        //going through each centroid, assign number to min
        int closest_centroid_index = -1;
        float min_distance = FLT_MAX/2;
        for (int i = 0; i < centroids.size(); i++){
            Vector3 centroid_material = Vector3(centroids[i][0], centroids[i][1], centroids[i][2]);
            float current_distance = distance(current_material, centroid_material);
            if (current_distance < min_distance){
                closest_centroid_index = i;
                min_distance = current_distance;
            }
        }
        Vector3 rgb_by_centroid = HSVtoRGB(360/(closest_centroid_index+1), 100, 100);
        pcl::PointXYZRGB visualized_point(rgb_by_centroid[0], rgb_by_centroid[1], rgb_by_centroid[2]);
        visualized_point.x = current_point.x;
        visualized_point.y = current_point.y;
        visualized_point.z = current_point.z;
        visualized_material_cloud->push_back(visualized_point);
    }
    return visualized_material_cloud;
}

