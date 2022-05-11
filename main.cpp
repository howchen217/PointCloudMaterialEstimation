#include <iostream>
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"
#include "PCPointMaterialDirector.h"
#include "PCNormal.h"
#include "PCReader.h"


#include <pcl/features/normal_3d.h>

#include "PCViewer.cpp"






int main() {
    std::cout << "Hello, World!" << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = PCReader::parseToXYZCloudManual(
            "/home/haocheng/Documents/ASC Cloud files/house4.txt");


//    for (const auto& point: *cloud_normals){
//        std::cout << point.normal_x << " " << point.normal_y << " " << point.normal_z << std::endl;
//    }


    float raw_intensity = 0.705063;
    Vector3 RGB(176, 185, 182);
    Vector3 point_coordinate(-14.903739, 18.695122, 6.623764);
    Vector3 scanner_position(0, 0, 0);

    Vector3 test_point_coordinate(-17.91761833, 17.79057989, -0.62501817);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals = PCNormal::getPCNormals(cloud);
    Vector3 point_normal = PCNormal::getPCPointNormal(cloud, cloud_normals, test_point_coordinate);

    PCPointMaterialBuilder* builder = new PCPointMaterialBuilder(raw_intensity, RGB, point_coordinate, point_normal, scanner_position);
    PCPointMaterialDirector director;

    director.setBuilder(builder);
    director.buildPCPointMaterial();

    pcmattex::PCPointMaterial mat = builder->getPCPointMaterial();

    std::cout << mat << std::endl;

    //to free up memory
    delete builder;
    cloud.reset();
    cloud_normals.reset();

    return 0;
}


