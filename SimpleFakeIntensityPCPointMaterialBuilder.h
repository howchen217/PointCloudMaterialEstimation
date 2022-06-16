//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_SIMPLEFAKEINTENSITYPCPOINTMATERIALBUILDER_H
#define PCMATTEX_SIMPLEFAKEINTENSITYPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"

class SimpleFakeIntensityPCPointMaterialBuilder : public PCPointMaterialBuilder{
private:
    Vector3 rgb;
    Vector3 point_coordinate;

    pcmattex::PCPointMaterial pcPointMaterial;


public:
    SimpleFakeIntensityPCPointMaterialBuilder();

    SimpleFakeIntensityPCPointMaterialBuilder(const Vector3 &rgb, const Vector3 &point_coordinate);

    void buildBasicData() override;
    void buildLaserDirection() override;
    void buildNormalDirection() override;
    void buildIncidenceAngle() override;
    void buildCorrectedIntensity() override;
    void buildReflectance() override;
    void buildAlbedo() override;
    void buildEmissivity() override;

    pcmattex::PCPointMaterial getPCPointMaterial() override;


};


#endif //PCMATTEX_SIMPLEFAKEINTENSITYPCPOINTMATERIALBUILDER_H
