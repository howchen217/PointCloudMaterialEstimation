//
// Created by haocheng on 7-5-22.
//

#include "PCPointMaterialBuilder.h"

void PCPointMaterialBuilder::buildBasicData() {
    pcPointMaterial.setRawIntensity(raw_intensity);
    pcPointMaterial.setRgb(rgb);
    pcPointMaterial.setPointCoordinate(point_coordinate);
    pcPointMaterial.setScannerPosition(scanner_position);
}

void PCPointMaterialBuilder::buildLaserDirection() {
    pcPointMaterial.setLaserDirection(point_coordinate - scanner_position);
}

/**
 * Calculate the normal direction of the point from a set of points.
 * Requires cloud input
 */
void PCPointMaterialBuilder::buildNormalDirection() {
    pcPointMaterial.setNormalDirection(point_normal);
}

void PCPointMaterialBuilder::buildIncidenceAngle() {
    Vector3 normal_direction = pcPointMaterial.getNormalDirection();
    Vector3 laser_direction = pcPointMaterial.getLaserDirection();
    float dot_product = Vector3::dotProduct(normal_direction, laser_direction);
    float magnitude_product = normal_direction.absolute() * laser_direction.absolute();
    pcPointMaterial.setAngleOfIncidence(acos(dot_product / magnitude_product));
}

void PCPointMaterialBuilder::buildCorrectedIntensity() {
    float corrected_intensity = raw_intensity * 1 / cos(pcPointMaterial.getAngleOfIncidence());
    pcPointMaterial.setCorrectedIntensity(corrected_intensity);
}

void PCPointMaterialBuilder::buildReflectance() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    pcPointMaterial.setReflectance((0.2125f * R / 255) + (0.7154f * G / 255) + (0.0721f * B / 255));
}

void PCPointMaterialBuilder::buildAlbedo() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float square_sum = (powf(R, 2) + powf(G, 2) + powf(B, 2));
    float albedo = sqrt(square_sum / 3) / 255 * pcPointMaterial.getCorrectedIntensity();
    pcPointMaterial.setAlbedo(albedo);
}

void PCPointMaterialBuilder::buildEmissivity() {
    pcPointMaterial.setEmissivity(1 - pcPointMaterial.getCorrectedIntensity());
}







pcmattex::PCPointMaterial PCPointMaterialBuilder::getPCPointMaterial() {
    return pcPointMaterial;
}

PCPointMaterialBuilder::PCPointMaterialBuilder(float raw_intensity, const Vector3 &rgb, const Vector3 &point_coordinate,
                                               const Vector3 &point_normal, const Vector3 &scanner_position)
        : raw_intensity(raw_intensity), rgb(rgb), point_coordinate(point_coordinate), point_normal(point_normal),
          scanner_position(scanner_position) {}


PCPointMaterialBuilder::PCPointMaterialBuilder() = default;



