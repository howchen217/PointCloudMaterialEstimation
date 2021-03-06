//
// Created by haocheng on 19-5-22.
//

#include "MockIntensityNoCorrectionPCPointMaterialBuilder.h"


void MockIntensityNoCorrectionPCPointMaterialBuilder::buildBasicData() {
    pcPointMaterial.setRgb(rgb);
    pcPointMaterial.setPointCoordinate(point_coordinate);
}


void MockIntensityNoCorrectionPCPointMaterialBuilder::buildLaserDirection() {
//    pcPointMaterial.setLaserDirection(Vector3(0));
}

/**
 * Calculate the normal direction of the point from a set of points.
 * Requires cloud input
 */
void MockIntensityNoCorrectionPCPointMaterialBuilder::buildNormalDirection() {
//    pcPointMaterial.setNormalDirection(Vector3(0));
}

void MockIntensityNoCorrectionPCPointMaterialBuilder::buildIncidenceAngle() {
}

/**
 * Use matlab method to convert to grayscale, models human perception
 */
void MockIntensityNoCorrectionPCPointMaterialBuilder::buildCorrectedIntensity() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float corrected_intensity = 0.3f * R/255 + 0.59f * G/255 + 0.11f * B/255;
    pcPointMaterial.setCorrectedIntensity(corrected_intensity);
}

void MockIntensityNoCorrectionPCPointMaterialBuilder::buildReflectance() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    pcPointMaterial.setReflectance((0.2125f * R / 255) + (0.7154f * G / 255) + (0.0721f * B / 255));
}

void MockIntensityNoCorrectionPCPointMaterialBuilder::buildAlbedo() {
    Vector3 RGB = pcPointMaterial.getRgb();
    float R = RGB[0], G = RGB[1], B = RGB[2];
    float square_sum = (powf(R, 2) + powf(G, 2) + powf(B, 2));
    float albedo = sqrt(square_sum / 3) / 255 * pcPointMaterial.getCorrectedIntensity();
    pcPointMaterial.setAlbedo(albedo);
}

void MockIntensityNoCorrectionPCPointMaterialBuilder::buildEmissivity() {
    pcPointMaterial.setEmissivity(1 - pcPointMaterial.getCorrectedIntensity());
}



PCPointMaterial MockIntensityNoCorrectionPCPointMaterialBuilder::getPCPointMaterial() {
    return pcPointMaterial;
}

MockIntensityNoCorrectionPCPointMaterialBuilder::MockIntensityNoCorrectionPCPointMaterialBuilder(const Vector3 &rgb,
                                                                                                 const Vector3 &point_coordinate) : rgb(rgb), point_coordinate(point_coordinate) {}


MockIntensityNoCorrectionPCPointMaterialBuilder::MockIntensityNoCorrectionPCPointMaterialBuilder() = default;



