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

#include <pcl/features/normal_3d.h>

#include "PCViewer.cpp"
#include "MockIntensityNoCorrectionPCPointMaterialBuilder.h"
#include "MockIntensityPCPointMaterialBuilder.h"
#include "PointXYZRGBMaterial.h"
#include <pcl/pcl_config.h>

int main(){
    //check pcl version
    std::cout << PCL_VERSION_PRETTY << std::endl;

    std::string filename = "../sample pcd files/same color dif material/metal_plate.pcd";

    //load cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb = PCReader::readPCDXYZRGBCloud(filename);

    //get cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_xyzrgb, *cloud_xyz);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud_xyz);

    //view normal of a single point
    PCViewer::viewPCPointNormal(cloud_xyz, cloud_normals, 1000);

    //make them into cloud with materials.
    pcl::PointCloud<PointXYZRGBMaterial>::Ptr cloud_with_material(new pcl::PointCloud<PointXYZRGBMaterial>);

    //k means preparation
    std::vector<std::vector<float>> kmeans_data;
    kmeans_data.reserve(cloud_xyzrgb->size());

    for (std::size_t i = 0; i < cloud_xyzrgb->size (); ++i) {

        //get the data
        auto point_xyzrgb = cloud_xyzrgb->points[i];
        pcl::Normal normal = cloud_normals->points[i];

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

    cloud_normals.reset();

    //kmeans, choose number of centroids
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

    //save cloud
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *visualized_material_cloud);

    //Display cloud
    PCViewer::viewCloud<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(visualized_material_cloud);

    return 0;
}



