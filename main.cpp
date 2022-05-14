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

    //display the cloud
//    simpleXYZViewer(cloud);

    //save the cloud to pcd
    pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);

    std::cout << "Loaded material with " << cloud->size() << " points." << std::endl;

    cloud.reset();
    cloud_normals.reset();

    return 0;
}



