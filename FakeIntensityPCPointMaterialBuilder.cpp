//
// Created by haocheng on 19-5-22.
//

#include "FakeIntensityPCPointMaterialBuilder.h"

void FakeIntensityPCPointMaterialBuilder::buildBasicData() {
    pcPointMaterial.setRgb(rgb);
    pcPointMaterial.setPointCoordinate(point_coordinate);
    pcPointMaterial.setScannerPosition(scanner_position);
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float raw_intensity = 0.3f * R/255 + 0.59f * G/255 + 0.11f * B/255;
    pcPointMaterial.setRawIntensity(raw_intensity);
}

void FakeIntensityPCPointMaterialBuilder::buildLaserDirection() {
    //pcPointMaterial.setLaserDirection(point_coordinate - scanner_position);
    pcPointMaterial.setLaserDirection(scanner_position - point_coordinate);
}

/**
 * Calculate the normal direction of the point from a set of points.
 * Requires cloud input
 */
void FakeIntensityPCPointMaterialBuilder::buildNormalDirection() {
    pcPointMaterial.setNormalDirection(point_normal);
}

void FakeIntensityPCPointMaterialBuilder::buildIncidenceAngle() {
    Vector3 normal_direction = pcPointMaterial.getNormalDirection();
    Vector3 laser_direction = pcPointMaterial.getLaserDirection();
    float dot_product = Vector3::dotProduct(normal_direction, laser_direction);
    float magnitude_product = normal_direction.absolute() * laser_direction.absolute();
    pcPointMaterial.setAngleOfIncidence(acos(dot_product / magnitude_product));
}

void FakeIntensityPCPointMaterialBuilder::buildCorrectedIntensity() {
    float corrected_intensity = pcPointMaterial.getRawIntensity() * 1 / cos(pcPointMaterial.getAngleOfIncidence());
    pcPointMaterial.setCorrectedIntensity(corrected_intensity);
}

void FakeIntensityPCPointMaterialBuilder::buildReflectance() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    pcPointMaterial.setReflectance((0.2125f * R / 255) + (0.7154f * G / 255) + (0.0721f * B / 255));
}

void FakeIntensityPCPointMaterialBuilder::buildAlbedo() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float square_sum = (powf(R, 2) + powf(G, 2) + powf(B, 2));
    float albedo = sqrt(square_sum / 3) / 255 * pcPointMaterial.getCorrectedIntensity();
    pcPointMaterial.setAlbedo(albedo);
}

void FakeIntensityPCPointMaterialBuilder::buildEmissivity() {
    pcPointMaterial.setEmissivity(1 - pcPointMaterial.getCorrectedIntensity());
}


PCPointMaterial FakeIntensityPCPointMaterialBuilder::getPCPointMaterial() {
    return pcPointMaterial;
}

FakeIntensityPCPointMaterialBuilder::FakeIntensityPCPointMaterialBuilder(const Vector3 &rgb,
                                                                         const Vector3 &point_coordinate,
                                                                         const Vector3 &scanner_position,
                                                                         const Vector3 &point_normal) : rgb(rgb),
                                                                                                        point_coordinate(
                                                                                                                point_coordinate),
                                                                                                        scanner_position(
                                                                                                                scanner_position),
                                                                                                        point_normal(
                                                                                                                point_normal) {}



FakeIntensityPCPointMaterialBuilder::FakeIntensityPCPointMaterialBuilder() = default;