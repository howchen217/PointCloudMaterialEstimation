//
// Created by haocheng on 9-5-22.
//


#include "PCViewer.h"

void PCViewer::viewPCPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, int point_index){

    // visualize point normal
    pcl::visualization::PCLVisualizer viewer("Point Normal Viewer");
    viewer.setBackgroundColor (0.1, 0.1, 0.1);
    viewer.addCoordinateSystem (0.07f, "global");

    //get indices of points that you wish to show
    std::vector<int> point_indices = {point_index};
    pcl::IndicesPtr indices(new std::vector <int>(point_indices));

    //make it into another cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_of_a_point(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyz, point_indices, *cloud_of_a_point);
    pcl::PointCloud<pcl::Normal>::Ptr normal_of_a_point(new pcl::PointCloud<pcl::Normal>);
    pcl::copyPointCloud(*cloud_normals, point_indices, *normal_of_a_point);

    //set the point cloud color
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud_of_a_point, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud_xyz, 255, 0, 0);

    //add to viewer
    viewer.addPointCloud(cloud_of_a_point, green, "cloud of a point");
    viewer.addPointCloud(cloud_xyz, red, "cloud");
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud_of_a_point, normal_of_a_point, 1,  0.03, "cloud_normals");

    //set point size
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "cloud of a point");

    //viewer
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }
}


template <typename T>
void PCViewer::viewCloud (T cloud) {

    pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
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

