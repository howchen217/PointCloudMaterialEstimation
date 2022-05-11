#include <iostream>
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"
#include "PCPointMaterialDirector.h"
#include "PCNormal.h"
#include "PCReader.h"
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/normal_3d.h>

#include "PCViewer.cpp"






int main() {

    //load the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PCReader::parseToXYZCloudManual(
            "/home/haocheng/Documents/ASC Cloud files/house4.txt");

    //set up the values and normals
    float raw_intensity = 0.705063;
    Vector3 RGB(176, 185, 182);
    Vector3 point_coordinate(-14.903739, 18.695122, 6.623764);
    Vector3 scanner_position(0, 0, 0);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud);
    Vector3 point_normal = PCNormal::getPCPointNormal(cloud, cloud_normals, point_coordinate);

    //visualize the normals

    // visualize normals
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.1, 0.1, 0.1);
    viewer.addCoordinateSystem (0.07f, "global");
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals, 1, 0.03);
    while (!viewer.wasStopped ())
    {
        viewer.spinOnce ();
    }


    //calculate the material
    PCPointMaterialBuilder* builder = new PCPointMaterialBuilder(raw_intensity, RGB, point_coordinate, point_normal, scanner_position);
    PCPointMaterialDirector director;
    director.setBuilder(builder);
    director.buildPCPointMaterial();
    pcmattex::PCPointMaterial mat = builder->getPCPointMaterial();

    //print the material
    std::cout << mat << std::endl;

    //to free up memory
    delete builder;
    cloud.reset();
    cloud_normals.reset();

    return 0;
}


