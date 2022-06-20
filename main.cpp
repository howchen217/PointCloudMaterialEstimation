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
#include "MockIntensityNoCorrectionPCPointMaterialBuilder.h"
#include "MockIntensityPCPointMaterialBuilder.h"
#include "PointXYZRGBMaterial.h"

int main(){

    std::string filename = "/home/haocheng/Documents/PCD files/same color dif material/plastic_jar_no_lid.pcd";

    //load cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = PCReader::readPCDXYZRGBCloud(filename);

    //get cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud_xyz);

    //make them into cloud with materials. normal no longer needed
    pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material(new pcl::PointCloud<PointXYZRGBMaterial>);

    //k means preparation
    std::vector<std::vector<float>> kmeans_data;
    kmeans_data.reserve(cloud_xyzrgb->size());

    for (std::size_t i = 0; i < cloud_xyzrgb->size (); ++i) {

        //get the data
        auto point_xyzrgb = cloud_xyzrgb->points[i];
        pcl::Normal normal = cloud_normals->points[i];

        //calculate the material
        //prepare data for material estimation
        Vector3 RGB(point_xyzrgb.r, point_xyzrgb.g, point_xyzrgb.b);
        Vector3 point_coordinate(point_xyzrgb.x, point_xyzrgb.y, point_xyzrgb.z);
        Vector3 scanner_position(0,0,0);
        Vector3 point_normal(normal.normal_x, normal.normal_y, normal.normal_z);

        //Material estimation without intensity correction
        PCPointMaterialBuilder* builder = new MockIntensityNoCorrectionPCPointMaterialBuilder(RGB, point_coordinate);
        PCPointMaterialDirector director;
        director.setBuilder(builder);
        director.buildPCPointMaterial();
        PCPointMaterial mat = builder->getPCPointMaterial();

        //std::cout << mat << std::endl;

        //check for unsuitable values
        if(!mat.getCorrectedIntensity() > 1 || !mat.getCorrectedIntensity() < 0 || !mat.materialIsNaN()){

            //create PointXYZRGBMaterialPoint and populate the cloud
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

    //kmeans
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

    //Point visualization.
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr visualized_material_cloud = PCViewer::createVisualizedMaterialCloud(
            cloud_with_material, centroids);

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

    //Display cloud
    PCViewer::viewCloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(visualized_material_cloud);

    return 0;
}



