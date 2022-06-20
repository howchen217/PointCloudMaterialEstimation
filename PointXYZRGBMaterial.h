//
// Created by haocheng on 20-6-22.
//

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>


#ifndef PCMATTEX_POINTXYZRGBMATERIAL_H
#define PCMATTEX_POINTXYZRGBMATERIAL_H


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


#endif //PCMATTEX_POINTXYZRGBMATERIAL_H
