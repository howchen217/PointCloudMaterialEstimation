#include <iostream>
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"
#include "PCPointMaterialDirector.h"
#include "PCNormal.h"
#include "PCReader.h"
#include <pcl/visualization/cloud_viewer.h>
#include <cassert>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ml/kmeans.h>
#include <pcl/pcl_macros.h>

#include <pcl/features/normal_3d.h>

#include "PCViewer.cpp"
#include "SimpleFakeIntensityPCPointMaterialBuilder.h"
#include "FakeIntensityPCPointMaterialBuilder.h"

struct PointXYZRGBMaterial {
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    float emissivity;
    float albedo;
    float reflectance;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
    //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBMaterial, (float, x, x)
(float, y, y) (float, z, z) (float, rgb, rgb) (float, emissivity, emissivity) (float, albedo, albedo) (float, reflectance, reflectnce));

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


int main(){

    //std::string filename = "/home/haocheng/Documents/PCD files/Ceramic plates/plate_beige_c.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/Ceramic plates/plate_blue_c.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/Ceramic plates/plate_white_c.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/plastic/plastic_basket.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/plastic/plastic_cooker.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/plastic/plastic_jar.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/wood/wood_bat.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/wood/wood_cutboard_c.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/wood/wood_saltshaker.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/same color dif material/cushion.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/same color dif material/metal_plate.pcd";
    std::string filename = "/home/haocheng/Documents/PCD files/same color dif material/plastic_jar_no_lid.pcd";

    //std::string filename = "/home/haocheng/Documents/PCD files/basketball.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/less_stainless_pan_cleaned.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/stainless_pan_cleaned.pcd";
    //std::string filename = "/home/haocheng/Documents/PCD files/wooden_spoon_cleaned.pcd";

    //load the cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = PCReader::readPCDXYZRGBCloud(filename);
    //get the normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz); //need to dereference the pointer to copy it over..
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud_xyz);
    //cloud_xyz.reset();

    //print normals
//    for (std::size_t i = 0; i < cloud_normals->size (); ++i) {
//        std::cout << cloud_normals->points[i] << std::endl;
//    }


    //can try visualize the normal later..or do normal refinement

    //make them into cloud with materials. normal no longer needed
    pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material(new pcl::PointCloud<PointXYZRGBMaterial>);

    //k means preparation
    std::vector<std::vector<float>> kmeans_data;
    kmeans_data.reserve(cloud_xyzrgb->size());

    //prepare point removal, not even necessary any more since point get automatically excluded.
    //pcl::PointIndices::Ptr points_of_weird_intensity(new pcl::PointIndices());

    for (std::size_t i = 0; i < cloud_xyzrgb->size (); ++i) {

        //get the data
        auto point_xyzrgb = cloud_xyzrgb->points[i];
        pcl::Normal normal = cloud_normals->points[i];

        //calculate the material
        //prepare data
        Vector3 RGB(point_xyzrgb.r, point_xyzrgb.g, point_xyzrgb.b);
        Vector3 point_coordinate(point_xyzrgb.x, point_xyzrgb.y, point_xyzrgb.z);
        Vector3 scanner_position(0,0,0);
        Vector3 point_normal(normal.normal_x, normal.normal_y, normal.normal_z);

        //with intensity correction
        //PCPointMaterialBuilder* builder = new FakeIntensityPCPointMaterialBuilder(RGB, point_coordinate, scanner_position, point_normal);
        //without intensity correction
        PCPointMaterialBuilder* builder = new SimpleFakeIntensityPCPointMaterialBuilder(RGB, point_coordinate);
        PCPointMaterialDirector director;
        director.setBuilder(builder);
        director.buildPCPointMaterial();
        PCPointMaterial mat = builder->getPCPointMaterial();

        //std::cout << mat << std::endl;

        if(mat.getCorrectedIntensity() > 1 || mat.getCorrectedIntensity() < 0 || mat.materialIsNaN()){
            //points_of_weird_intensity->indices.push_back(i);
        } else {

            //create new point, put it in material cloud
            PointXYZRGBMaterial new_point;
            new_point.x = point_xyzrgb.x;
            new_point.y = point_xyzrgb.y;
            new_point.z = point_xyzrgb.z;
            new_point.r = point_xyzrgb.r;
            new_point.g = point_xyzrgb.g;
            new_point.b = point_xyzrgb.b;
            new_point.emissivity = mat.getEmissivity();
            new_point.albedo = mat.getAlbedo();
            new_point.reflectance = mat.getReflectance();
            cloud_with_material->push_back(new_point);

            //prepare for k means
            std::vector<float> point_material(3);
            point_material[0] = mat.getEmissivity();
            point_material[1] = mat.getAlbedo();
            point_material[2] = mat.getReflectance();
            kmeans_data.emplace_back(point_material);
        }

        delete builder;
    }


    //check how many points weren't included
    //point removal is unnecessary currently since material cloud does not even include them
    //std::cout << "to be removed: "<< points_of_weird_intensity->indices.size() << std::endl;
    std::cout << "original cloud has: " << cloud_xyzrgb->size() << " points,";
    std::cout << "material cloud has: " << cloud_with_material->size() << " points.";

    cloud_normals.reset();

    //do kmeans
    int k = 1;
    pcl::Kmeans kmeans(kmeans_data.size(), 3); //3 is dimension of vector
    kmeans.setClusterSize(k);
    kmeans.setInputData(kmeans_data);
    kmeans.kMeans();
    pcl::Kmeans::Centroids centroids = kmeans.get_centroids();
    std::cout << "points used for kmeans: " << kmeans_data.size() << std::endl;
    std::cout << "centroid count: " << centroids.size() << std::endl;
    for (int i = 0; i<centroids.size(); i++)
    {
        std::cout << i << "_cent output: emissivity: " << centroids[i][0] << " ,";
        std::cout << "albedo: " << centroids[i][1] << " ,";
        std::cout << "reflectance: " << centroids[i][2] << std::endl;
    }

    //iterate through every point of the cloud, calculate distance from all centroids, and assign it a label?
    //need to know the material associated with point, make a structure that stores the material and xyz
    //this is just brute force for now
    //make a new XYZRGB Cloud, this will be populated instead of the cloud_with_material cloud since that cannot be visualized
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualized_material_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::size_t i = 0; i < cloud_with_material->size (); ++i)
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
        //std::cout << visualized_point << std::endl;
    }

    cloud_with_material.reset();

    //track centroid point counts to see which cluster is bigger
    int cluster1_size = 0;
    int cluster2_size = 0;
    Vector3 cluster1_rgb = HSVtoRGB(360, 100, 100);
    for (std::size_t i = 0; i < visualized_material_cloud->size (); ++i){
        pcl::PointXYZRGB visualized_point = visualized_material_cloud->points[i];
        if(visualized_point.r == cluster1_rgb[0] && visualized_point.g == cluster1_rgb[1] && visualized_point.b == cluster1_rgb[2]){
            cluster1_size++;
        } else {
            cluster2_size++;
        }
    }
    std::cout << "total points: " << visualized_material_cloud->size() << std::endl;
    std::cout << "centroid 1 cluster size: " << cluster1_size << std::endl;
    std::cout << "centroid 2 cluster size: " << cluster2_size << std::endl;

    pcl::io::savePCDFileASCII ("test_pcd.pcd", *visualized_material_cloud);

    //See if it possible to display custom cloud
    pcl::visualization::CloudViewer viewer3("RGBXYZ Viewer");
    viewer3.showCloud(visualized_material_cloud);
    while (!viewer3.wasStopped()) {

    }

    visualized_material_cloud.reset();
    return 0;
}



