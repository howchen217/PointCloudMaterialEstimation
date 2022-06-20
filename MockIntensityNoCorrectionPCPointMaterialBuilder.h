//
// Created by haocheng on 19-5-22.
//

#ifndef PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H
#define PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H


#include "Vector3.hpp"
#include "PCPointMaterial.h"
#include "PCPointMaterialBuilder.h"


class MockIntensityNoCorrectionPCPointMaterialBuilder : public PCPointMaterialBuilder{
private:
    Vector3 rgb;
    Vector3 point_coordinate;

    PCPointMaterial pcPointMaterial;


public:
    MockIntensityNoCorrectionPCPointMaterialBuilder();

    MockIntensityNoCorrectionPCPointMaterialBuilder(const Vector3 &rgb, const Vector3 &point_coordinate);

    void buildBasicData() override;
    void buildLaserDirection() override;
    void buildNormalDirection() override;
    void buildIncidenceAngle() override;
    void buildCorrectedIntensity() override;
    void buildReflectance() override;
    void buildAlbedo() override;
    void buildEmissivity() override;

    PCPointMaterial getPCPointMaterial() override;


};


#endif //PCMATTEX_MOCKINTENSITYNOCORRECTIONPCPOINTMATERIALBUILDER_H
