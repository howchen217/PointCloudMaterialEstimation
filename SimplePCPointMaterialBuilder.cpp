//
// Created by haocheng on 19-5-22.
//

#include "SimplePCPointMaterialBuilder.h"


void SimplePCPointMaterialBuilder::buildBasicData() {
    pcPointMaterial.setRawIntensity(raw_intensity);
    pcPointMaterial.setRgb(rgb);
    pcPointMaterial.setPointCoordinate(point_coordinate);
}

void SimplePCPointMaterialBuilder::buildLaserDirection() {
//    pcPointMaterial.setLaserDirection(Vector3(0));
}

/**
 * Calculate the normal direction of the point from a set of points.
 * Requires cloud input
 */
void SimplePCPointMaterialBuilder::buildNormalDirection() {
//    pcPointMaterial.setNormalDirection(Vector3(0));
}

void SimplePCPointMaterialBuilder::buildIncidenceAngle() {
}

/**
 * Use matlab method to convert to grayscale, models human perception
 */
void SimplePCPointMaterialBuilder::buildCorrectedIntensity() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float corrected_intensity = 0.3f * R + 0.59f * G + 0.11f * B;
    pcPointMaterial.setCorrectedIntensity(corrected_intensity);
}

void SimplePCPointMaterialBuilder::buildReflectance() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    pcPointMaterial.setReflectance((0.2125f * R / 255) + (0.7154f * G / 255) + (0.0721f * B / 255));
}

void SimplePCPointMaterialBuilder::buildAlbedo() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float square_sum = (powf(R, 2) + powf(G, 2) + powf(B, 2));
    float albedo = sqrt(square_sum / 3) / 255 * pcPointMaterial.getCorrectedIntensity();
    pcPointMaterial.setAlbedo(albedo);
}

void SimplePCPointMaterialBuilder::buildEmissivity() {
    pcPointMaterial.setEmissivity(1 - pcPointMaterial.getCorrectedIntensity());
}



pcmattex::PCPointMaterial SimplePCPointMaterialBuilder::getPCPointMaterial() {
    return pcPointMaterial;
}

SimplePCPointMaterialBuilder::SimplePCPointMaterialBuilder(float raw_intensity, const Vector3 &rgb,
                                                           const Vector3 &point_coordinate) : raw_intensity(
        raw_intensity), rgb(rgb), point_coordinate(point_coordinate) {}


SimplePCPointMaterialBuilder::SimplePCPointMaterialBuilder() = default;



