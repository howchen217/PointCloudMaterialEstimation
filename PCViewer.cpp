//
// Created by haocheng on 9-5-22.
//
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "PCViewer.h"

void simpleXYZViewer (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {

    pcl::visualization::CloudViewer viewer("Simple XYZ Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }
}

