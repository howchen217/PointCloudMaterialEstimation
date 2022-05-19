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

    std::string filename = "/home/haocheng/Documents/ASC Cloud files/house4.txt";

    //load the cloud and get the normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PCReader::parseToXYZCloudManual(
    filename);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud);

    //Reading the file line by line again for material information
    std::cout << "Begin calculating materials of entire cloud" << std::endl;
    FILE* f = fopen(filename.c_str(), "r");
    if(nullptr == f)
    {
        std::cout << "ERROR: failed to open file: " << filename << std::endl;
        return -1;
    }
    float x, y, z;
    int r, g, b;
    float raw_intensity;
    //the index is which line we are on in the file and should match the index of point in the cloud
    int current_index = 0;
    //indices of points to be removed, since they have weird corrected intensity values, <0 or >1.
    pcl::PointIndices::Ptr points_of_weird_intensity(new pcl::PointIndices());

    //get the data needed for k means, reserve enough spots
    std::vector<std::vector<float>> kmeans_data;
    kmeans_data.reserve(cloud->size());

    //this vector is the data for the cloud with material
    //pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material;
    pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material(new pcl::PointCloud<PointXYZRGBMaterial>);

    while (!feof(f)){
        int n_args = fscanf(f, "%f %f %f %i %i %i %f", &x, &y, &z, &r, &g, &b, &raw_intensity);
        if(n_args != 7)
            continue;
        //to check if index consistent with line of file
        assert (x == cloud->points[current_index].x); //might fail due to float point errors? Or would it?

        //parse the data appropriately
        Vector3 RGB(r, g, b);
        Vector3 point_coordinate(x, y, z);
        Vector3 point_normal = PCNormal::getNormalVectorByIndex(cloud_normals, current_index);
        Vector3 scanner_position(0, 0, 0);

        //calculate the material
        PCPointMaterialBuilder* builder = new PCPointMaterialBuilder(raw_intensity, RGB, point_coordinate, point_normal, scanner_position);
        PCPointMaterialDirector director;
        director.setBuilder(builder);
        director.buildPCPointMaterial();
        pcmattex::PCPointMaterial mat = builder->getPCPointMaterial();

        //remove weird points, nan
        if(mat.getCorrectedIntensity() > 1 || mat.getCorrectedIntensity() < 0 || mat.materialIsNaN()){
            points_of_weird_intensity->indices.push_back(current_index);
        } else {
            //print the material only if good
            //std::cout << mat << std::endl;

            //only add the good points to k means clustering, {emissivity, albedo, reflectance}
            std::vector<float> point_material(3);
            point_material[0] = mat.getEmissivity();
            point_material[1] = mat.getAlbedo();
            point_material[2] = mat.getReflectance();
            kmeans_data.emplace_back(point_material);

            //store xyz material pair so it can be displayed later?
            //or make custom point type
            Vector3 point_coordinate = mat.getPointCoordinate();
            Vector3 point_rgb = mat.getRgb();
            PointXYZRGBMaterial point_with_material;
            point_with_material.x = point_coordinate[0];
            point_with_material.y = point_coordinate[1];
            point_with_material.z = point_coordinate[2];
            point_with_material.r = point_rgb[0];
            point_with_material.g = point_rgb[1];
            point_with_material.b = point_rgb[2];
            point_with_material.emissivity = mat.getEmissivity();
            point_with_material.albedo = mat.getAlbedo();
            point_with_material.reflectance = mat.getReflectance();
            cloud_with_material->push_back(point_with_material);
        }

        //update index
        current_index++;

        delete builder;

    }
    fclose(f);



    std::cout << "to be removed: "<< points_of_weird_intensity->indices.size() << std::endl;

    //delete points that have corrected intensity > 1 or < 0
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(points_of_weird_intensity);
    extract.setNegative(true);
    extract.filter(*cloud);


    //do kmeans
    int k = 3;
    pcl::Kmeans kmeans(kmeans_data.size(), 3);
    kmeans.setClusterSize(k);
    kmeans.setInputData(kmeans_data);
    kmeans.kMeans();
    pcl::Kmeans::Centroids centroids = kmeans.get_centroids();
    std::cout << "points used for kmeans: " << cloud->size() << std::endl;
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
        Vector3 rgb_by_centroid = HSVtoRGB(360/(closest_centroid_index+1), 100, 50);
        pcl::PointXYZRGB visualized_point(rgb_by_centroid[0], rgb_by_centroid[1], rgb_by_centroid[2]);
        visualized_point.x = current_point.x;
        visualized_point.y = current_point.y;
        visualized_point.z = current_point.z;
        visualized_material_cloud->push_back(visualized_point);
        //std::cout << visualized_point << std::endl;
    }



    //See if it possible to display custom cloud
    pcl::visualization::CloudViewer viewer("RGBXYZ Viewer");
    viewer.showCloud(visualized_material_cloud);
    while (!viewer.wasStopped()) {

    }




    //display the cloud
//    simpleXYZViewer(cloud);

    //save the cloud to pcd
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

    std::cout << "Loaded material with " << cloud->size() << " points." << std::endl;

    cloud.reset();
    cloud_normals.reset();

    return 0;
}



