//
// Created by haocheng on 10-5-22.
//

#ifndef PCMATTEX_PCREADER_H
#define PCMATTEX_PCREADER_H

#include <pcl/point_types.h>
#include <pcl/io/ascii_io.h>
#include <pcl/point_cloud.h>

/**
 * Utility class for point cloud reading.
 */
class PCReader {
public:

    /**
     * \brief Given a cloud of (x y z r g b Intensity) point format in ASCII, separated by single spaces,
     * parse it into a PCL PointXYZ cloud.
     *
     * The cloud is of ASCII format with each row data of a point. The method reads row by row and add each point
     * into a PCL PointXYZ cloud.
     *
     * @param filename File name of the ASCII cloud in (x y z r g b Intensity) point format.
     * @return PCL Point Cloud shared pointer of point type PointXYZ.
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr readASCToXYZCloudManual(const std::string &filename);

    /**
     * \brief Given a cloud of (x y z) point format in ASCII, separated by single spaces, parse it into a PCL PointXYZ
     * cloud.
     *
     * It uses PCL cloud reader, however it cannot handle if each row has more than 3 values. It cannot read if the
     * ASC file has (x y z r g b) as data on each row for example.
     *
     * @param filename File name of the ASCII cloud in (x y z) point format.
     * @return PCL Point Cloud shared pointer of point type PointXYZ.
     */
    static pcl::PointCloud<pcl::PointXYZ>::Ptr readASCToXYZCloudPCl(const std::string &filename);

    /**
     * \brief Given a PCD file of point cloud of PointXYZRGB format, it reads it into a PCL PointXYZRGB cloud.
     *
     * @param filename File name of the PCD cloud of PointXYZRGB format.
     * @return PCL Point Cloud shared pointer of point type PointXYZRGB.
     */
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr readPCDXYZRGBCloud(const std::string& filename);
};

#endif //PCMATTEX_PCREADER_H
