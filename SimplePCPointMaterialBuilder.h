//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_SIMPLEPCPOINTMATERIALBUILDER_H
#define PCMATTEX_SIMPLEPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"

class SimplePCPointMaterialBuilder {
private:
    float raw_intensity;
    Vector3 rgb;
    Vector3 point_coordinate;

    pcmattex::PCPointMaterial pcPointMaterial;


public:
    SimplePCPointMaterialBuilder();

    SimplePCPointMaterialBuilder(float raw_intensity, const Vector3 &rgb, const Vector3 &point_coordinate);

    void buildBasicData();
    void buildLaserDirection();
    void buildNormalDirection();
    void buildIncidenceAngle();
    void buildCorrectedIntensity();
    void buildReflectance();
    void buildAlbedo();
    void buildEmissivity();

    pcmattex::PCPointMaterial getPCPointMaterial();


};


#endif //PCMATTEX_SIMPLEPCPOINTMATERIALBUILDER_H
