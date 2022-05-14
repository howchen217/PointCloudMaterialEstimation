//
// Created by haocheng yang on 2-5-22.
//

#include <iostream>
#include "PCPointMaterial.h"
#include <cmath>
#include "Vector3.hpp"

namespace pcmattex {

    void printHelp() {
        std::cout << "This library calculates material properties from point cloud color and intensity." << std::endl;
    }

    std::ostream &operator<<(std::ostream &os, const PCPointMaterial &material) {
        os << "====================" << std::endl
           << "raw_intensity: " << material.raw_intensity << std::endl
           << "rgb: " << material.rgb << std::endl
           << "point_coordinate: " << material.point_coordinate << std::endl
           << "corrected_intensity: " << material.corrected_intensity << std::endl
           << "angle_of_incidence: " << material.angle_of_incidence << std::endl
           << "scanner_position: " << material.scanner_position << std:: endl
           << "point_normal: " << material.point_normal << std::endl
        <<"laser_direction: " << material.laser_direction << std::endl
        << "emissivity: " << material.emissivity << std::endl
        << "albedo: " << material.albedo << std::endl
        << "reflectance: " << material.reflectance << std::endl
        << "====================" << std::endl;
        return os;
    }

    float PCPointMaterial::getRawIntensity() const {
        return raw_intensity;
    }

    void PCPointMaterial::setRawIntensity(float raw_intensity) {
        PCPointMaterial::raw_intensity = raw_intensity;
    }

    const Vector3 &PCPointMaterial::getRgb() const {
        return rgb;
    }

    void PCPointMaterial::setRgb(const Vector3 &rgb) {
        PCPointMaterial::rgb = rgb;
    }

    const Vector3 &PCPointMaterial::getPointCoordinate() const {
        return point_coordinate;
    }

    void PCPointMaterial::setPointCoordinate(const Vector3 &point_coordinate) {
        PCPointMaterial::point_coordinate = point_coordinate;
    }

    float PCPointMaterial::getCorrectedIntensity() const {
        return corrected_intensity;
    }

    void PCPointMaterial::setCorrectedIntensity(float corrected_intensity) {
        PCPointMaterial::corrected_intensity = corrected_intensity;
    }

    float PCPointMaterial::getAngleOfIncidence() const {
        return angle_of_incidence;
    }

    void PCPointMaterial::setAngleOfIncidence(float angle_of_incidence) {
        PCPointMaterial::angle_of_incidence = angle_of_incidence;
    }

    const Vector3 &PCPointMaterial::getScannerPosition() const {
        return scanner_position;
    }

    void PCPointMaterial::setScannerPosition(const Vector3 &scanner_position) {
        PCPointMaterial::scanner_position = scanner_position;
    }

    const Vector3 &PCPointMaterial::getNormalDirection() const {
        return point_normal;
    }

    void PCPointMaterial::setNormalDirection(const Vector3 &normal_direction) {
        PCPointMaterial::point_normal = normal_direction;
    }

    const Vector3 &PCPointMaterial::getLaserDirection() const {
        return laser_direction;
    }

    void PCPointMaterial::setLaserDirection(const Vector3 &laser_direction) {
        PCPointMaterial::laser_direction = laser_direction;
    }

    float PCPointMaterial::getEmissivity() const {
        return emissivity;
    }

    void PCPointMaterial::setEmissivity(float emissivity) {
        PCPointMaterial::emissivity = emissivity;
    }

    float PCPointMaterial::getAlbedo() const {
        return albedo;
    }

    void PCPointMaterial::setAlbedo(float albedo) {
        PCPointMaterial::albedo = albedo;
    }

    float PCPointMaterial::getReflectance() const {
        return reflectance;
    }

    void PCPointMaterial::setReflectance(float reflectance) {
        PCPointMaterial::reflectance = reflectance;
    }

    bool PCPointMaterial::materialIsNaN() {
        if (isnan(albedo) || isnan(emissivity) || isnan(reflectance)){
            return true;
        }
        return false;
    }

    PCPointMaterial::PCPointMaterial() = default;


}