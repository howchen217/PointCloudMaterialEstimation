//
// Created by haocheng on 7-5-22.
//

#ifndef PCMATTEX_PCPOINTMATERIALBUILDER_H
#define PCMATTEX_PCPOINTMATERIALBUILDER_H

#include "Vector3.hpp"
#include "PCPointMaterial.h"

class PCPointMaterialBuilder {
private:
    float raw_intensity{};
    Vector3 rgb;
    Vector3 point_coordinate;
    Vector3 point_normal;
    Vector3 scanner_position;


    PCPointMaterial pcPointMaterial;


public:
    PCPointMaterialBuilder();

    PCPointMaterialBuilder(float raw_intensity, const Vector3 &rgb, const Vector3 &point_coordinate,
                           const Vector3 &point_normal, const Vector3 &scanner_position);


    virtual void buildBasicData();

    virtual void buildLaserDirection();

    virtual void buildNormalDirection();

    virtual void buildIncidenceAngle();

    virtual void buildCorrectedIntensity();

    virtual void buildReflectance();

    virtual void buildAlbedo();

    virtual void buildEmissivity();

    virtual PCPointMaterial getPCPointMaterial();



};


#endif //PCMATTEX_PCPOINTMATERIALBUILDER_H
